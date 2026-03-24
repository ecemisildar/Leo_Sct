// image_processor.cpp
//
// Depth obstacle zones + ArUco detector.
// Publishes:
// - std_msgs/String on "detected_zones" with one of: LEFT, RIGHT, CORNER, CLEAR
// - std_msgs/Bool on "aruco_id1_detected" (true when target ArUco is detected)
// - std_msgs/String on "aruco_id1_direction" with LEFT/CENTER/RIGHT/NONE
// - std_msgs/Float32 on "aruco_id1_offset" with normalized center error in [-0.5, 0.5]
// - std_msgs/Float32 on "front_obstacle_distance" with front ROI near distance in meters

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

class DepthZoneDetector : public rclcpp::Node
{
public:
  DepthZoneDetector() : Node("image_processor")
  {
    clock_ = this->get_clock();
    const auto now = clock_->now();

    zones_pub_ = this->create_publisher<std_msgs::msg::String>("detected_zones", 10);
    aruco_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("aruco_id1_detected", 10);
    aruco_direction_pub_ = this->create_publisher<std_msgs::msg::String>("aruco_id1_direction", 10);
    aruco_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("aruco_id1_distance", 10);
    aruco_offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("aruco_id1_offset", 10);
    front_obstacle_distance_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("front_obstacle_distance", 10);

    enter_thresh_ = this->declare_parameter<double>("enter_thresh", 0.60);
    exit_thresh_ = this->declare_parameter<double>("exit_thresh", 0.80);
    emergency_thresh_ = this->declare_parameter<double>("emergency_thresh", 0.30);
    side_enter_thresh_ = this->declare_parameter<double>("side_enter_thresh", enter_thresh_);
    side_exit_thresh_ = this->declare_parameter<double>("side_exit_thresh", exit_thresh_);
    left_enter_thresh_ = this->declare_parameter<double>("left_enter_thresh", side_enter_thresh_);
    right_enter_thresh_ = this->declare_parameter<double>("right_enter_thresh", side_enter_thresh_);
    left_exit_thresh_ = this->declare_parameter<double>("left_exit_thresh", side_exit_thresh_);
    right_exit_thresh_ = this->declare_parameter<double>("right_exit_thresh", side_exit_thresh_);

    min_depth_ = this->declare_parameter<double>("min_depth", 0.08);
    max_depth_ = this->declare_parameter<double>("max_depth", 10.0);

    stride_ = this->declare_parameter<int>("stride", 2);
    percentile_ = this->declare_parameter<double>("percentile", 0.10);
    near_count_k_ = this->declare_parameter<int>("near_count_k", 4);
    valid_count_min_ = this->declare_parameter<int>("valid_count_min", 60);

    crop_y0_frac_ = this->declare_parameter<double>("crop_y0_frac", 0.30);
    crop_y1_frac_ = this->declare_parameter<double>("crop_y1_frac", 0.80);
    front_gain_ = this->declare_parameter<double>("front_gain", 1.40);

    hold_ms_ = this->declare_parameter<int>("hold_ms", 250);
    show_debug_ = this->declare_parameter<bool>("show_debug", false);
    clear_skip_ = this->declare_parameter<int>("clear_skip", 3);
    safe_frames_required_ = this->declare_parameter<int>("safe_frames_required", 5);

    aruco_enabled_ = this->declare_parameter<bool>("aruco_enabled", aruco_enabled_);
    aruco_debug_ = this->declare_parameter<bool>("aruco_debug", aruco_debug_);
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", depth_topic_);
    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", rgb_topic_);
    aruco_dictionary_id_ = this->declare_parameter<int>("aruco_dictionary_id", aruco_dictionary_id_);
    aruco_target_id_ = this->declare_parameter<int>("aruco_target_id", aruco_target_id_);
    aruco_center_tolerance_ = this->declare_parameter<double>("aruco_center_tolerance", aruco_center_tolerance_);
    aruco_seen_hold_ms_ = this->declare_parameter<int>("aruco_seen_hold_ms", aruco_seen_hold_ms_);
    aruco_depth_mask_enabled_ = this->declare_parameter<bool>("aruco_depth_mask_enabled", aruco_depth_mask_enabled_);
    aruco_depth_mask_margin_px_ = this->declare_parameter<int>("aruco_depth_mask_margin_px", aruco_depth_mask_margin_px_);
    aruco_min_marker_perimeter_rate_ = this->declare_parameter<double>(
      "aruco_min_marker_perimeter_rate", aruco_min_marker_perimeter_rate_);
    aruco_max_marker_perimeter_rate_ = this->declare_parameter<double>(
      "aruco_max_marker_perimeter_rate", aruco_max_marker_perimeter_rate_);
    aruco_adaptive_thresh_win_min_ = this->declare_parameter<int>(
      "aruco_adaptive_thresh_win_min", aruco_adaptive_thresh_win_min_);
    aruco_adaptive_thresh_win_max_ = this->declare_parameter<int>(
      "aruco_adaptive_thresh_win_max", aruco_adaptive_thresh_win_max_);
    aruco_adaptive_thresh_win_step_ = this->declare_parameter<int>(
      "aruco_adaptive_thresh_win_step", aruco_adaptive_thresh_win_step_);

    // ArUco defaults: DICT_4X4_100, target id=1, RGB topic camera/camera/color/image_raw.
    aruco_dict_ = cv::aruco::getPredefinedDictionary(aruco_dictionary_id_);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    aruco_params_->minMarkerPerimeterRate =
      static_cast<float>(std::clamp(aruco_min_marker_perimeter_rate_, 0.001, 1.0));
    aruco_params_->maxMarkerPerimeterRate =
      static_cast<float>(std::clamp(aruco_max_marker_perimeter_rate_, 1.0, 20.0));
    aruco_params_->adaptiveThreshWinSizeMin = std::max(3, aruco_adaptive_thresh_win_min_ | 1);
    aruco_params_->adaptiveThreshWinSizeMax = std::max(
      aruco_params_->adaptiveThreshWinSizeMin + 2, aruco_adaptive_thresh_win_max_ | 1);
    aruco_params_->adaptiveThreshWinSizeStep = std::max(2, aruco_adaptive_thresh_win_step_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      qos,
      std::bind(&DepthZoneDetector::depthCallback, this, std::placeholders::_1));

    if (aruco_enabled_) {
      rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_,
        qos,
        std::bind(&DepthZoneDetector::rgbCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "image_processor topics: depth='%s' rgb='%s' aruco_enabled=%s target_id=%d dictionary=%d",
      depth_topic_.c_str(),
      rgb_topic_.c_str(),
      aruco_enabled_ ? "true" : "false",
      aruco_target_id_,
      aruco_dictionary_id_);

    last_state_ = "CLEAR";
    last_non_clear_zone_ = "CORNER";
    last_non_clear_time_ = now;
  }

private:
  struct RoiStats
  {
    float p;
    int near_count;
    int valid_count;
  };

  struct ZoneStats
  {
    RoiStats left;
    RoiStats front;
    RoiStats right;
    cv::Rect left_roi;
    cv::Rect front_roi;
    cv::Rect right_roi;
  };

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    const auto now = clock_->now();

    if (last_state_ == "CLEAR" && clear_skip_ > 1) {
      clear_skip_counter_ = (clear_skip_counter_ + 1) % clear_skip_;
      if (clear_skip_counter_ != 0) {
        std_msgs::msg::String out;
        out.data = last_state_;
        zones_pub_->publish(out);
        return;
      }
    } else {
      clear_skip_counter_ = 0;
    }

    cv::Mat depth;
    std::string encoding;
    try {
      auto depth_ptr = cv_bridge::toCvShare(depth_msg);
      depth = depth_ptr->image;
      encoding = depth_ptr->encoding;

      if (encoding == "16UC1") {
        depth.convertTo(depth, CV_32FC1, 0.001);
      } else if (encoding == "32FC1") {
        if (depth.type() != CV_32FC1) {
          depth.convertTo(depth, CV_32FC1);
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Unsupported depth encoding: %s (expected 16UC1 or 32FC1)", encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (depth.empty() || depth.cols < 6 || depth.rows < 6) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Empty/small depth image.");
      return;
    }
    last_depth_ = depth.clone();
    last_depth_time_ = now;

    cv::Mat depth_for_zones = depth.clone();
    applyArucoDepthMask(depth_for_zones, now);

    ZoneStats zs = computeZoneStats(depth_for_zones);
    std::string zone_now = determineZoneHysteresis(zs);

    std_msgs::msg::Float32 front_msg;
    front_msg.data =
      (zs.front.p > 0.0f) ? zs.front.p : std::numeric_limits<float>::quiet_NaN();
    front_obstacle_distance_pub_->publish(front_msg);

    if (zone_now != "CLEAR") {
      last_non_clear_zone_ = zone_now;
      last_non_clear_time_ = now;
    } else {
      const double age_ms = (now - last_non_clear_time_).seconds() * 1000.0;
      if (age_ms >= 0.0 && age_ms < static_cast<double>(hold_ms_)) {
        zone_now = last_non_clear_zone_;
      }
    }

    std_msgs::msg::String out;
    out.data = zone_now;
    zones_pub_->publish(out);

    if (show_debug_) {
      cv::Mat vis;
      cv::normalize(depth, vis, 0, 255, cv::NORM_MINMAX);
      vis.convertTo(vis, CV_8U);
      cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
      drawROIs(vis);
      cv::imshow("Depth (debug)", vis);
      cv::waitKey(1);
    }
  }

  void rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
  {
    cv::Mat bgr;
    try {
      auto rgb_ptr = cv_bridge::toCvShare(rgb_msg);
      const auto & enc = rgb_ptr->encoding;
      if (enc == "bgr8") {
        bgr = rgb_ptr->image;
      } else if (enc == "rgb8") {
        cv::cvtColor(rgb_ptr->image, bgr, cv::COLOR_RGB2BGR);
      } else if (enc == "bgra8") {
        cv::cvtColor(rgb_ptr->image, bgr, cv::COLOR_BGRA2BGR);
      } else if (enc == "rgba8") {
        cv::cvtColor(rgb_ptr->image, bgr, cv::COLOR_RGBA2BGR);
      } else if (enc == "mono8" || enc == "8UC1") {
        cv::cvtColor(rgb_ptr->image, bgr, cv::COLOR_GRAY2BGR);
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Unsupported RGB encoding for ArUco: %s", enc.c_str());
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
      return;
    }

    if (bgr.empty()) {
      return;
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

    bool detected = false;
    std::string direction = "NONE";
    float marker_offset = std::numeric_limits<float>::quiet_NaN();
    float distance_m = std::numeric_limits<float>::quiet_NaN();
    bool distance_valid = false;
    cv::Point2f target_center(0.0f, 0.0f);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != aruco_target_id_) {
        continue;
      }
      detected = true;
      if (i < corners.size() && !corners[i].empty()) {
        float cx = 0.0f;
        float cy = 0.0f;
        for (const auto & p : corners[i]) {
          cx += p.x;
          cy += p.y;
        }
        cx /= static_cast<float>(corners[i].size());
        cy /= static_cast<float>(corners[i].size());
        target_center = cv::Point2f(cx, cy);
        const float w = static_cast<float>(gray.cols);
        if (w > 1.0f) {
          const float norm = (cx / w) - 0.5f;
          marker_offset = norm;
          const float tol = static_cast<float>(std::clamp(aruco_center_tolerance_, 0.01, 0.45));
          if (norm < -tol) {
            direction = "LEFT";
          } else if (norm > tol) {
            direction = "RIGHT";
          } else {
            direction = "CENTER";
          }
        } else {
          direction = "CENTER";
        }
      }
      break;
    }

    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      if (detected && !corners.empty()) {
        last_marker_corners_ = corners;
        last_marker_ids_ = ids;
      } else if (!detected) {
        last_marker_corners_.clear();
        last_marker_ids_.clear();
      }
    }

    if (detected && !last_depth_.empty()) {
      const auto now = this->get_clock()->now();
      const double depth_age_ms = (now - last_depth_time_).seconds() * 1000.0;
      if (depth_age_ms >= 0.0 && depth_age_ms <= 350.0) {
        if (last_depth_.rows == gray.rows && last_depth_.cols == gray.cols) {
          const int radius = 2;
          const int cx = std::clamp(static_cast<int>(std::round(target_center.x)), 0, last_depth_.cols - 1);
          const int cy = std::clamp(static_cast<int>(std::round(target_center.y)), 0, last_depth_.rows - 1);
          const int x0 = std::max(0, cx - radius);
          const int x1 = std::min(last_depth_.cols - 1, cx + radius);
          const int y0 = std::max(0, cy - radius);
          const int y1 = std::min(last_depth_.rows - 1, cy + radius);

          float best = std::numeric_limits<float>::infinity();
          for (int y = y0; y <= y1; ++y) {
            const float * row = last_depth_.ptr<float>(y);
            for (int x = x0; x <= x1; ++x) {
              const float d = row[x];
              if (std::isfinite(d) && d > static_cast<float>(min_depth_) && d < static_cast<float>(max_depth_)) {
                best = std::min(best, d);
              }
            }
          }
          if (std::isfinite(best)) {
            distance_m = best;
            distance_valid = true;
          }
        }
      }
    }

    const auto now = this->get_clock()->now();
    if (detected) {
      last_aruco_seen_time_ = now;
      aruco_seen_once_ = true;
      if (direction != "NONE") {
        last_aruco_direction_ = direction;
      }
      if (distance_valid) {
        last_aruco_distance_m_ = distance_m;
      }
    } else if (aruco_seen_once_) {
      const double age_ms = (now - last_aruco_seen_time_).seconds() * 1000.0;
      if (age_ms >= 0.0 && age_ms <= static_cast<double>(aruco_seen_hold_ms_)) {
        detected = true;
        if (direction == "NONE" && last_aruco_direction_ != "NONE") {
          direction = last_aruco_direction_;
        }
        if (!distance_valid && std::isfinite(last_aruco_distance_m_)) {
          distance_m = last_aruco_distance_m_;
          distance_valid = true;
        }
      }
    }

    std_msgs::msg::Bool msg;
    msg.data = detected;
    aruco_detected_pub_->publish(msg);
    std_msgs::msg::String dir_msg;
    dir_msg.data = direction;
    aruco_direction_pub_->publish(dir_msg);
    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = distance_valid ? distance_m : std::numeric_limits<float>::quiet_NaN();
    aruco_distance_pub_->publish(dist_msg);
    std_msgs::msg::Float32 offset_msg;
    offset_msg.data = marker_offset;
    aruco_offset_pub_->publish(offset_msg);

    // RCLCPP_INFO_THROTTLE(
    //   this->get_logger(), *this->get_clock(), 500,
    //   "Aruco detected=%s direction=%s offset=%.3f distance=%.3f",
    //   detected ? "true" : "false",
    //   direction.c_str(),
    //   std::isfinite(marker_offset) ? marker_offset : -9.0f,
    //   distance_valid ? distance_m : -1.0f);

    if (aruco_debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Aruco dictionary=%d target_id=%d total_detected=%zu",
        aruco_dictionary_id_,
        aruco_target_id_,
        ids.size());
    }
  }

  ZoneStats computeZoneStats(const cv::Mat & depth)
  {
    const int w = depth.cols;
    const int h = depth.rows;

    int y0 = static_cast<int>(std::round(std::clamp(crop_y0_frac_, 0.0, 0.95) * h));
    int y1 = static_cast<int>(std::round(std::clamp(crop_y1_frac_, 0.05, 1.0) * h));
    if (y1 <= y0 + 1) {
      y0 = 0;
      y1 = h;
    }
    int roi_h = std::max(1, y1 - y0);

    const double fg = std::max(1.0, front_gain_);
    int front_w = static_cast<int>(std::round((w / 3.0) * fg));
    front_w = std::clamp(front_w, 1, w - 2);

    int side_total = w - front_w;
    int left_w = std::max(1, side_total / 2);
    int right_w = std::max(1, w - front_w - left_w);
    int front_x = left_w;

    cv::Rect left(0, y0, left_w, roi_h);
    cv::Rect front(front_x, y0, front_w, roi_h);
    cv::Rect right(front_x + front_w, y0, right_w, roi_h);

    ZoneStats zs;
    zs.left_roi = left;
    zs.front_roi = front;
    zs.right_roi = right;

    const float front_near_thr = static_cast<float>(std::max(enter_thresh_, exit_thresh_));
    const float left_near_thr = static_cast<float>(
      std::max({left_enter_thresh_, left_exit_thresh_, side_enter_thresh_, side_exit_thresh_}));
    const float right_near_thr = static_cast<float>(
      std::max({right_enter_thresh_, right_exit_thresh_, side_enter_thresh_, side_exit_thresh_}));
    zs.left = roiStats(depth, left, percentile_, stride_, left_near_thr);
    zs.front = roiStats(depth, front, percentile_, stride_, front_near_thr);
    zs.right = roiStats(depth, right, percentile_, stride_, right_near_thr);

    return zs;
  }

  RoiStats roiStats(
    const cv::Mat & depth, const cv::Rect & roi, double percentile, int stride, float near_thresh)
  {
    std::vector<float> vals;
    const int st = std::max(1, stride);
    vals.reserve((roi.width / st + 1) * (roi.height / st + 1));

    const float min_d = static_cast<float>(min_depth_);
    const float max_d = static_cast<float>(max_depth_);

    int near_cnt = 0;
    int valid_cnt = 0;

    for (int y = roi.y; y < roi.y + roi.height; y += st) {
      const float * row = depth.ptr<float>(y);
      for (int x = roi.x; x < roi.x + roi.width; x += st) {
        float d = row[x];
        if (std::isfinite(d) && d > min_d && d < max_d) {
          valid_cnt++;
          if (d < near_thresh) {
            near_cnt++;
          }
          vals.push_back(d);
        }
      }
    }

    if (vals.empty()) {
      return {-1.0f, 0, 0};
    }

    double p = std::clamp(percentile, 0.0, 1.0);
    size_t k = static_cast<size_t>(std::round(p * static_cast<double>(vals.size() - 1)));
    k = std::min(k, vals.size() - 1);

    std::nth_element(vals.begin(), vals.begin() + k, vals.end());
    return {vals[k], near_cnt, valid_cnt};
  }

  bool zoneIsObstacle(const RoiStats & rs, double thresh) const
  {
    if (rs.p <= 0.0f) {
      return false;
    }
    if (rs.valid_count < valid_count_min_) {
      return false;
    }
    if (rs.p >= static_cast<float>(thresh)) {
      return false;
    }
    return rs.near_count >= near_count_k_;
  }

  std::string determineZoneHysteresis(const ZoneStats & zs)
  {
    const bool front_emergency =
      (zs.front.p > 0.0f) &&
      (zs.front.valid_count >= valid_count_min_) &&
      (zs.front.p < static_cast<float>(emergency_thresh_)) &&
      (zs.front.near_count >= near_count_k_);

    if (front_emergency) {
      last_state_ = "CORNER";
      safe_frames_ = 0;
      return last_state_;
    }

    if (last_state_ != "CLEAR") {
      const bool front_close = zoneIsObstacle(zs.front, exit_thresh_);
      const bool left_close = zoneIsObstacle(zs.left, left_exit_thresh_);
      const bool right_close = zoneIsObstacle(zs.right, right_exit_thresh_);

      if (front_close || left_close || right_close) {
        safe_frames_ = 0;
        if (front_close) {
          last_state_ = "CORNER";
          return last_state_;
        }

        const float lp = (zs.left.p > 0.0f) ? zs.left.p : 1e9f;
        const float rp = (zs.right.p > 0.0f) ? zs.right.p : 1e9f;

        if (left_close && !right_close) {
          last_state_ = "LEFT";
        } else if (right_close && !left_close) {
          last_state_ = "RIGHT";
        } else {
          last_state_ = (lp <= rp) ? "LEFT" : "RIGHT";
        }
        return last_state_;
      }

      safe_frames_++;
      if (safe_frames_ >= std::max(1, safe_frames_required_)) {
        last_state_ = "CLEAR";
        safe_frames_ = 0;
        return last_state_;
      }
      return last_state_;
    }

    if (zoneIsObstacle(zs.front, enter_thresh_)) {
      last_state_ = "CORNER";
      safe_frames_ = 0;
      return last_state_;
    }
    if (zoneIsObstacle(zs.left, left_enter_thresh_)) {
      last_state_ = "LEFT";
      safe_frames_ = 0;
      return last_state_;
    }
    if (zoneIsObstacle(zs.right, right_enter_thresh_)) {
      last_state_ = "RIGHT";
      safe_frames_ = 0;
      return last_state_;
    }

    return "CLEAR";
  }

  void applyArucoDepthMask(cv::Mat & depth, const rclcpp::Time & now)
  {
    if (!aruco_depth_mask_enabled_) {
      return;
    }

    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      marker_corners = last_marker_corners_;
      marker_ids = last_marker_ids_;
    }

    if (marker_corners.empty() || marker_ids.empty()) {
      return;
    }

    const double age_ms = (now - last_aruco_seen_time_).seconds() * 1000.0;
    if (age_ms < 0.0 || age_ms > static_cast<double>(aruco_seen_hold_ms_)) {
      return;
    }

    const int margin = std::max(0, aruco_depth_mask_margin_px_);
    for (size_t i = 0; i < marker_ids.size() && i < marker_corners.size(); ++i) {
      if (marker_ids[i] != aruco_target_id_ || marker_corners[i].empty()) {
        continue;
      }

      int min_x = depth.cols - 1;
      int max_x = 0;
      int min_y = depth.rows - 1;
      int max_y = 0;
      for (const auto & p : marker_corners[i]) {
        min_x = std::min(min_x, static_cast<int>(std::floor(p.x)));
        max_x = std::max(max_x, static_cast<int>(std::ceil(p.x)));
        min_y = std::min(min_y, static_cast<int>(std::floor(p.y)));
        max_y = std::max(max_y, static_cast<int>(std::ceil(p.y)));
      }

      min_x = std::clamp(min_x - margin, 0, depth.cols - 1);
      max_x = std::clamp(max_x + margin, 0, depth.cols - 1);
      min_y = std::clamp(min_y - margin, 0, depth.rows - 1);
      max_y = std::clamp(max_y + margin, 0, depth.rows - 1);
      if (min_x > max_x || min_y > max_y) {
        continue;
      }

      const cv::Rect roi(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
      depth(roi).setTo(std::numeric_limits<float>::quiet_NaN());
    }
  }

  void drawROIs(cv::Mat & vis)
  {
    const int w = vis.cols;
    const int h = vis.rows;

    int y0 = static_cast<int>(std::round(std::clamp(crop_y0_frac_, 0.0, 0.95) * h));
    int y1 = static_cast<int>(std::round(std::clamp(crop_y1_frac_, 0.05, 1.0) * h));
    if (y1 <= y0 + 1) {
      y0 = 0;
      y1 = h;
    }
    int roi_h = std::max(1, y1 - y0);

    const double fg = std::max(1.0, front_gain_);
    int front_w = static_cast<int>(std::round((w / 3.0) * fg));
    front_w = std::clamp(front_w, 1, w - 2);

    int side_total = w - front_w;
    int left_w = std::max(1, side_total / 2);
    int right_w = std::max(1, w - front_w - left_w);
    int front_x = left_w;

    cv::rectangle(vis, cv::Rect(0, y0, left_w, roi_h), cv::Scalar(255, 255, 255), 1);
    cv::rectangle(vis, cv::Rect(front_x, y0, front_w, roi_h), cv::Scalar(255, 255, 255), 1);
    cv::rectangle(vis, cv::Rect(front_x + front_w, y0, right_w, roi_h), cv::Scalar(255, 255, 255), 1);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zones_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aruco_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aruco_direction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aruco_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aruco_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_obstacle_distance_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Clock::SharedPtr clock_;

  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  double enter_thresh_{0.60};
  double exit_thresh_{0.80};
  double emergency_thresh_{0.30};
  double side_enter_thresh_{0.60};
  double side_exit_thresh_{0.80};
  double left_enter_thresh_{0.60};
  double right_enter_thresh_{0.60};
  double left_exit_thresh_{0.80};
  double right_exit_thresh_{0.80};

  double min_depth_{0.08};
  double max_depth_{10.0};

  int stride_{2};
  double percentile_{0.10};
  int near_count_k_{4};
  int valid_count_min_{60};

  double crop_y0_frac_{0.30};
  double crop_y1_frac_{0.80};
  double front_gain_{1.40};

  int hold_ms_{250};
  bool show_debug_{false};
  int clear_skip_{1};
  int clear_skip_counter_{0};

  int safe_frames_required_{5};
  int safe_frames_{0};

  bool aruco_enabled_{true};
  bool aruco_debug_{true};
  std::string depth_topic_{"depth_camera/depth_image"};
  std::string rgb_topic_{"camera/camera/color/image_raw"};
  int aruco_dictionary_id_{cv::aruco::DICT_4X4_100};
  int aruco_target_id_{1};
  double aruco_center_tolerance_{0.15};
  int aruco_seen_hold_ms_{150};
  bool aruco_depth_mask_enabled_{true};
  int aruco_depth_mask_margin_px_{12};
  double aruco_min_marker_perimeter_rate_{0.015};
  double aruco_max_marker_perimeter_rate_{4.0};
  int aruco_adaptive_thresh_win_min_{3};
  int aruco_adaptive_thresh_win_max_{43};
  int aruco_adaptive_thresh_win_step_{4};
  cv::Mat last_depth_;
  rclcpp::Time last_depth_time_;
  rclcpp::Time last_aruco_seen_time_;
  bool aruco_seen_once_{false};
  std::string last_aruco_direction_{"NONE"};
  float last_aruco_distance_m_{std::numeric_limits<float>::quiet_NaN()};
  std::mutex marker_mutex_;
  std::vector<std::vector<cv::Point2f>> last_marker_corners_;
  std::vector<int> last_marker_ids_;

  std::string last_state_{"CLEAR"};
  std::string last_non_clear_zone_{"CORNER"};
  rclcpp::Time last_non_clear_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthZoneDetector>());
  rclcpp::shutdown();
  return 0;
}
