// image_processor.cpp
//
// Depth obstacle zones + ArUco detector.
// Publishes:
// - std_msgs/String  "detected_zones"          : LEFT | RIGHT | CORNER | CLEAR
// - std_msgs/Bool    "aruco_id1_detected"       : true when target ArUco visible
// - std_msgs/String  "aruco_id1_direction"      : LEFT | CENTER | RIGHT | NONE
// - std_msgs/Float32 "aruco_id1_offset"         : normalised centre error [-0.5, 0.5]
// - std_msgs/Float32 "front_obstacle_distance"  : front ROI near distance (m)

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
    // Publishers
    zones_pub_                  = create_publisher<std_msgs::msg::String>("detected_zones", 10);
    aruco_detected_pub_         = create_publisher<std_msgs::msg::Bool>("aruco_id1_detected", 10);
    aruco_direction_pub_        = create_publisher<std_msgs::msg::String>("aruco_id1_direction", 10);
    aruco_distance_pub_         = create_publisher<std_msgs::msg::Float32>("aruco_id1_distance", 10);
    aruco_offset_pub_           = create_publisher<std_msgs::msg::Float32>("aruco_id1_offset", 10);
    front_obstacle_distance_pub_ = create_publisher<std_msgs::msg::Float32>("front_obstacle_distance", 10);

    // Parameters
    auto p = [this](auto name, auto def) { return declare_parameter<decltype(def)>(name, def); };

    enter_thresh_         = p("enter_thresh",         0.60);
    exit_thresh_          = p("exit_thresh",          0.80);
    emergency_thresh_     = p("emergency_thresh",     0.30);
    side_enter_thresh_    = p("side_enter_thresh",    enter_thresh_);
    side_exit_thresh_     = p("side_exit_thresh",     exit_thresh_);
    left_enter_thresh_    = p("left_enter_thresh",    side_enter_thresh_);
    right_enter_thresh_   = p("right_enter_thresh",   side_enter_thresh_);
    left_exit_thresh_     = p("left_exit_thresh",     side_exit_thresh_);
    right_exit_thresh_    = p("right_exit_thresh",    side_exit_thresh_);
    min_depth_            = p("min_depth",            0.08);
    max_depth_            = p("max_depth",            10.0);
    stride_               = p("stride",               2);
    percentile_           = p("percentile",           0.10);
    near_count_k_         = p("near_count_k",         4);
    valid_count_min_      = p("valid_count_min",      60);
    crop_y0_frac_         = p("crop_y0_frac",         0.30);
    crop_y1_frac_         = p("crop_y1_frac",         0.80);
    front_gain_           = p("front_gain",           1.40);
    hold_ms_              = p("hold_ms",              50);
    corner_hold_ms_       = p("corner_hold_ms",       30);
    side_hold_ms_         = p("side_hold_ms",         50);
    show_debug_           = p("show_debug",           false);
    clear_skip_           = p("clear_skip",           1);
    safe_frames_required_        = p("safe_frames_required",        5);
    corner_safe_frames_required_ = p("corner_safe_frames_required", 1);
    side_safe_frames_required_   = p("side_safe_frames_required",   2);
    zone_log_enabled_     = p("zone_log_enabled",     true);
    zone_log_throttle_ms_ = p("zone_log_throttle_ms", 500);
    process_every_nth_depth_frame_ = p("process_every_nth_depth_frame", 1);
    depth_stall_timeout_s_  = p("depth_stall_timeout_s",  1.5);
    depth_watchdog_enabled_ = p("depth_watchdog_enabled", true);

    aruco_enabled_          = p("aruco_enabled",          true);
    aruco_debug_            = p("aruco_debug",            true);
    depth_topic_            = p("depth_topic",            std::string("depth_camera/depth_image"));
    rgb_topic_              = p("rgb_topic",              std::string("camera/camera/color/image_raw"));
    aruco_dictionary_id_    = p("aruco_dictionary_id",    (int)cv::aruco::DICT_4X4_100);
    aruco_target_id_        = p("aruco_target_id",        1);
    aruco_center_tolerance_ = p("aruco_center_tolerance", 0.15);
    aruco_seen_hold_ms_     = p("aruco_seen_hold_ms",     150);
    aruco_depth_mask_enabled_   = p("aruco_depth_mask_enabled",    true);
    aruco_depth_mask_margin_px_ = p("aruco_depth_mask_margin_px",  12);
    aruco_min_marker_perimeter_rate_ = p("aruco_min_marker_perimeter_rate", 0.015);
    aruco_max_marker_perimeter_rate_ = p("aruco_max_marker_perimeter_rate", 4.0);
    aruco_adaptive_thresh_win_min_   = p("aruco_adaptive_thresh_win_min",   3);
    aruco_adaptive_thresh_win_max_   = p("aruco_adaptive_thresh_win_max",   43);
    aruco_adaptive_thresh_win_step_  = p("aruco_adaptive_thresh_win_step",  4);

    // ArUco detector setup
    aruco_dict_   = cv::aruco::getPredefinedDictionary(aruco_dictionary_id_);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->cornerRefinementMethod  = cv::aruco::CORNER_REFINE_SUBPIX;
    aruco_params_->minMarkerPerimeterRate  = (float)std::clamp(aruco_min_marker_perimeter_rate_, 0.001, 1.0);
    aruco_params_->maxMarkerPerimeterRate  = (float)std::clamp(aruco_max_marker_perimeter_rate_, 1.0, 20.0);
    aruco_params_->adaptiveThreshWinSizeMin  = std::max(3, aruco_adaptive_thresh_win_min_ | 1);
    aruco_params_->adaptiveThreshWinSizeMax  = std::max(aruco_params_->adaptiveThreshWinSizeMin + 2,
                                                         aruco_adaptive_thresh_win_max_ | 1);
    aruco_params_->adaptiveThreshWinSizeStep = std::max(2, aruco_adaptive_thresh_win_step_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      std::bind(&DepthZoneDetector::depthCallback, this, std::placeholders::_1));

    if (aruco_enabled_) {
      rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, qos,
        std::bind(&DepthZoneDetector::rgbCallback, this, std::placeholders::_1));
    }

    depth_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&DepthZoneDetector::depthWatchdogCallback, this));

    RCLCPP_INFO(get_logger(),
      "image_processor: depth='%s' rgb='%s' aruco=%s target_id=%d dict=%d",
      depth_topic_.c_str(), rgb_topic_.c_str(),
      aruco_enabled_ ? "true" : "false", aruco_target_id_, aruco_dictionary_id_);

    const auto now = get_clock()->now();
    last_non_clear_time_ = now;
    last_depth_time_     = now;
  }

private:
  // ── Data types ────────────────────────────────────────────────────────────

  struct RoiStats { float p; int near_count; int valid_count; };

  struct ZoneStats {
    RoiStats left, front, right;
    cv::Rect left_roi, front_roi, right_roi;
  };

  // ── ROI geometry (single source of truth) ─────────────────────────────────

  struct RoiGeometry { cv::Rect left, front, right; };

  RoiGeometry computeRoiGeometry(int w, int h) const
  {
    int y0 = (int)std::round(std::clamp(crop_y0_frac_, 0.0, 0.95) * h);
    int y1 = (int)std::round(std::clamp(crop_y1_frac_, 0.05, 1.0) * h);
    if (y1 <= y0 + 1) { y0 = 0; y1 = h; }
    const int roi_h   = std::max(1, y1 - y0);
    int front_w = std::clamp((int)std::round((w / 3.0) * std::max(1.0, front_gain_)), 1, w - 2);
    const int left_w  = std::max(1, (w - front_w) / 2);
    const int right_w = std::max(1, w - front_w - left_w);
    return {
      cv::Rect(0,               y0, left_w,  roi_h),
      cv::Rect(left_w,          y0, front_w, roi_h),
      cv::Rect(left_w + front_w, y0, right_w, roi_h)
    };
  }

  // ── Depth callback ─────────────────────────────────────────────────────────

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const auto now = get_clock()->now();
    depth_received_once_  = true;
    depth_missing_warned_ = false;
    last_depth_time_      = now;

    if (++depth_frame_counter_ % std::max(1, process_every_nth_depth_frame_) != 0) return;

    cv::Mat depth;
    try {
      auto ptr = cv_bridge::toCvShare(msg);
      depth    = ptr->image;
      if (ptr->encoding == "16UC1") {
        depth.convertTo(depth, CV_32FC1, 0.001);
      } else if (ptr->encoding == "32FC1") {
        if (depth.type() != CV_32FC1) depth.convertTo(depth, CV_32FC1);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Unsupported depth encoding: %s", ptr->encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what()); return;
    }

    if (depth.empty() || depth.cols < 6 || depth.rows < 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty/small depth image."); return;
    }

    if (aruco_enabled_) last_depth_ = depth.clone();

    cv::Mat depth_for_zones = depth;
    if (aruco_enabled_ && aruco_depth_mask_enabled_) {
      depth_for_zones = depth.clone();
      applyArucoDepthMask(depth_for_zones, now);
    }

    const ZoneStats zs = computeZoneStats(depth_for_zones);

    std_msgs::msg::Float32 front_msg;
    front_msg.data = zs.front.p > 0.0f ? zs.front.p : std::numeric_limits<float>::quiet_NaN();
    front_obstacle_distance_pub_->publish(front_msg);

    std::string zone = determineZoneHysteresis(zs);

    // Time-based hold: keep last non-CLEAR zone for a short window
    if (zone != "CLEAR") {
      last_non_clear_zone_ = zone;
      last_non_clear_time_ = now;
    } else {
      const double age_ms = (now - last_non_clear_time_).seconds() * 1000.0;
      const int hold = (last_non_clear_zone_ == "CORNER") ? corner_hold_ms_
                     : (last_non_clear_zone_ == "LEFT" || last_non_clear_zone_ == "RIGHT") ? side_hold_ms_
                     : hold_ms_;
      if (age_ms >= 0.0 && age_ms < hold) zone = last_non_clear_zone_;
    }

    std_msgs::msg::String out;
    out.data = zone;
    zones_pub_->publish(out);
    logZoneState(zone, &zs);

    if (show_debug_) {
      cv::Mat vis;
      cv::normalize(depth, vis, 0, 255, cv::NORM_MINMAX);
      vis.convertTo(vis, CV_8U);
      cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
      const auto g = computeRoiGeometry(vis.cols, vis.rows);
      cv::rectangle(vis, g.left,  cv::Scalar(255, 255, 255), 1);
      cv::rectangle(vis, g.front, cv::Scalar(255, 255, 255), 1);
      cv::rectangle(vis, g.right, cv::Scalar(255, 255, 255), 1);
      cv::imshow("Depth (debug)", vis);
      cv::waitKey(1);
    }
  }

  // ── Watchdog ───────────────────────────────────────────────────────────────

  void depthWatchdogCallback()
  {
    if (!depth_watchdog_enabled_) return;
    const auto now    = get_clock()->now();
    const double age  = (now - last_depth_time_).seconds();
    if (!depth_received_once_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No depth frames received yet on '%s'", depth_topic_.c_str());
      return;
    }
    if (age > depth_stall_timeout_s_ && !depth_missing_warned_) {
      RCLCPP_WARN(get_logger(), "Depth stream stalled on '%s' (%.2f s ago)",
        depth_topic_.c_str(), age);
      depth_missing_warned_ = true;
    }
  }

  // ── RGB / ArUco callback ───────────────────────────────────────────────────

  void rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat bgr;
    try {
      auto ptr = cv_bridge::toCvShare(msg);
      const auto & enc = ptr->encoding;
      if      (enc == "bgr8")               bgr = ptr->image;
      else if (enc == "rgb8")               cv::cvtColor(ptr->image, bgr, cv::COLOR_RGB2BGR);
      else if (enc == "bgra8")              cv::cvtColor(ptr->image, bgr, cv::COLOR_BGRA2BGR);
      else if (enc == "rgba8")              cv::cvtColor(ptr->image, bgr, cv::COLOR_RGBA2BGR);
      else if (enc == "mono8" || enc == "8UC1") cv::cvtColor(ptr->image, bgr, cv::COLOR_GRAY2BGR);
      else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Unsupported RGB encoding for ArUco: %s", enc.c_str());
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception (RGB): %s", e.what()); return;
    }
    if (bgr.empty()) return;

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

    bool        detected      = false;
    std::string direction     = "NONE";
    float       marker_offset = std::numeric_limits<float>::quiet_NaN();
    float       distance_m    = std::numeric_limits<float>::quiet_NaN();
    bool        dist_valid    = false;
    cv::Point2f target_center;
    cv::Rect    target_bbox;

    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != aruco_target_id_) continue;
      detected = true;

      if (i < corners.size() && !corners[i].empty()) {
        float cx = 0, cy = 0;
        for (const auto & p : corners[i]) { cx += p.x; cy += p.y; }
        cx /= corners[i].size(); cy /= corners[i].size();
        target_center = {cx, cy};

        int x0 = gray.cols - 1, x1 = 0, y0 = gray.rows - 1, y1 = 0;
        for (const auto & p : corners[i]) {
          x0 = std::min(x0, (int)std::floor(p.x)); x1 = std::max(x1, (int)std::ceil(p.x));
          y0 = std::min(y0, (int)std::floor(p.y)); y1 = std::max(y1, (int)std::ceil(p.y));
        }
        x0 = std::clamp(x0 - 4, 0, gray.cols - 1); x1 = std::clamp(x1 + 4, 0, gray.cols - 1);
        y0 = std::clamp(y0 - 4, 0, gray.rows - 1); y1 = std::clamp(y1 + 4, 0, gray.rows - 1);
        target_bbox = cv::Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

        if (gray.cols > 1) {
          marker_offset = (cx / gray.cols) - 0.5f;
          const float tol = (float)std::clamp(aruco_center_tolerance_, 0.01, 0.45);
          direction = marker_offset < -tol ? "LEFT" : marker_offset > tol ? "RIGHT" : "CENTER";
        } else {
          direction = "CENTER";
        }
      }
      break;
    }

    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      if (detected) { last_marker_corners_ = corners; last_marker_ids_ = ids; }
      else          { last_marker_corners_.clear(); last_marker_ids_.clear(); }
    }

    // Depth distance estimation for detected marker
    if (detected && !last_depth_.empty()) {
      const auto now = get_clock()->now();
      if ((now - last_depth_time_).seconds() * 1000.0 <= 350.0 &&
          last_depth_.rows == gray.rows && last_depth_.cols == gray.cols)
      {
        std::vector<float> dvals;
        if (target_bbox.area() > 0) {
          dvals.reserve(target_bbox.area());
          for (int y = target_bbox.y; y < target_bbox.y + target_bbox.height; ++y) {
            const float * row = last_depth_.ptr<float>(y);
            for (int x = target_bbox.x; x < target_bbox.x + target_bbox.width; ++x) {
              float d = row[x];
              if (std::isfinite(d) && d > (float)min_depth_ && d < (float)max_depth_)
                dvals.push_back(d);
            }
          }
        }

        if (!dvals.empty()) {
          size_t k = std::min(dvals.size() - 1,
            (size_t)std::floor(0.20 * (dvals.size() - 1)));
          std::nth_element(dvals.begin(), dvals.begin() + k, dvals.end());
          distance_m = dvals[k]; dist_valid = true;
        } else {
          // Fallback: nearest valid pixel around marker centre
          const int r  = 2;
          const int cx = std::clamp((int)std::round(target_center.x), 0, last_depth_.cols - 1);
          const int cy = std::clamp((int)std::round(target_center.y), 0, last_depth_.rows - 1);
          float best = std::numeric_limits<float>::infinity();
          for (int y = std::max(0, cy - r); y <= std::min(last_depth_.rows - 1, cy + r); ++y)
            for (int x = std::max(0, cx - r); x <= std::min(last_depth_.cols - 1, cx + r); ++x) {
              float d = last_depth_.ptr<float>(y)[x];
              if (std::isfinite(d) && d > (float)min_depth_ && d < (float)max_depth_)
                best = std::min(best, d);
            }
          if (std::isfinite(best)) { distance_m = best; dist_valid = true; }
        }
      }
    }

    // Hold last known state briefly after marker leaves frame
    const auto now = get_clock()->now();
    if (detected) {
      last_aruco_seen_time_ = now;
      aruco_seen_once_      = true;
      if (direction != "NONE") last_aruco_direction_ = direction;
      if (dist_valid)          last_aruco_distance_m_ = distance_m;
    } else if (aruco_seen_once_) {
      const double age_ms = (now - last_aruco_seen_time_).seconds() * 1000.0;
      if (age_ms >= 0.0 && age_ms <= aruco_seen_hold_ms_) {
        detected = true;
        if (direction == "NONE")  direction  = last_aruco_direction_;
        if (!dist_valid && std::isfinite(last_aruco_distance_m_))
          { distance_m = last_aruco_distance_m_; dist_valid = true; }
      }
    }

    // RCLCPP_INFO_THROTTLE(
    //   this->get_logger(), *this->get_clock(), 500,
    //   "Aruco detected=%s direction=%s offset=%.3f distance=%.3f",
    //   detected ? "true" : "false",
    //   direction.c_str(),
    //   std::isfinite(marker_offset) ? marker_offset : -9.0f,
    //   distance_valid ? distance_m : -1.0f);
    // if (aruco_debug_) {
    //   RCLCPP_INFO_THROTTLE(
    //     this->get_logger(), *this->get_clock(), 1000,
    //     "Aruco dictionary=%d target_id=%d total_detected=%zu",
    //     aruco_dictionary_id_,
    //     aruco_target_id_,
    //     ids.size());
    // }

    std_msgs::msg::Bool b; b.data = detected;
    aruco_detected_pub_->publish(b);
    std_msgs::msg::String d; d.data = direction;
    aruco_direction_pub_->publish(d);
    std_msgs::msg::Float32 dist;
    dist.data = dist_valid ? distance_m : std::numeric_limits<float>::quiet_NaN();
    aruco_distance_pub_->publish(dist);
    std_msgs::msg::Float32 off; off.data = marker_offset;
    aruco_offset_pub_->publish(off);
  }

  // ── Zone stats ─────────────────────────────────────────────────────────────

  ZoneStats computeZoneStats(const cv::Mat & depth)
  {
    const auto g = computeRoiGeometry(depth.cols, depth.rows);
    const float fn = (float)std::max({enter_thresh_, exit_thresh_});
    const float ln = (float)std::max({left_enter_thresh_,  left_exit_thresh_,
                                      side_enter_thresh_,  side_exit_thresh_});
    const float rn = (float)std::max({right_enter_thresh_, right_exit_thresh_,
                                      side_enter_thresh_,  side_exit_thresh_});
    ZoneStats zs;
    zs.left_roi  = g.left;  zs.front_roi = g.front; zs.right_roi = g.right;
    zs.left  = roiStats(depth, g.left,  percentile_, stride_, ln);
    zs.front = roiStats(depth, g.front, percentile_, stride_, fn);
    zs.right = roiStats(depth, g.right, percentile_, stride_, rn);
    return zs;
  }

  RoiStats roiStats(
    const cv::Mat & depth, const cv::Rect & roi,
    double percentile, int stride, float near_thresh)
  {
    std::vector<float> vals;
    const int   st    = std::max(1, stride);
    const float min_d = (float)min_depth_, max_d = (float)max_depth_;
    int near_cnt = 0, valid_cnt = 0;

    vals.reserve((roi.width / st + 1) * (roi.height / st + 1));
    for (int y = roi.y; y < roi.y + roi.height; y += st) {
      const float * row = depth.ptr<float>(y);
      for (int x = roi.x; x < roi.x + roi.width; x += st) {
        const float d = row[x];
        if (!std::isfinite(d) || d <= min_d || d >= max_d) continue;
        ++valid_cnt;
        if (d < near_thresh) ++near_cnt;
        vals.push_back(d);
      }
    }
    if (vals.empty()) return {-1.0f, 0, 0};

    size_t k = std::min(
      vals.size() - 1,
      (size_t)std::round(std::clamp(percentile, 0.0, 1.0) * (vals.size() - 1)));
    std::nth_element(vals.begin(), vals.begin() + k, vals.end());
    return {vals[k], near_cnt, valid_cnt};
  }

  // Returns true if the ROI contains an obstacle given the threshold
  bool isObstacle(const RoiStats & rs, double thresh) const
  {
    return rs.p > 0.0f
        && rs.valid_count >= valid_count_min_
        && rs.p < (float)thresh
        && rs.near_count >= near_count_k_;
  }

  // ── Hysteresis state machine ───────────────────────────────────────────────

  std::string determineZoneHysteresis(const ZoneStats & zs)
  {
    // Emergency: front obstacle very close
    const bool emergency =
      zs.front.p > 0.0f && zs.front.valid_count >= valid_count_min_ &&
      zs.front.p < (float)emergency_thresh_ && zs.front.near_count >= near_count_k_;
    if (emergency) { last_state_ = "CORNER"; safe_frames_ = 0; return last_state_; }

    if (last_state_ != "CLEAR") {
      // Use exit thresholds while already in an obstacle state
      const bool fc = isObstacle(zs.front, exit_thresh_);
      const bool lc = isObstacle(zs.left,  left_exit_thresh_);
      const bool rc = isObstacle(zs.right, right_exit_thresh_);
      if (fc || lc || rc) {
        safe_frames_ = 0;
        if (fc) { last_state_ = "CORNER"; return last_state_; }
        const float lp = zs.left.p  > 0.0f ? zs.left.p  : 1e9f;
        const float rp = zs.right.p > 0.0f ? zs.right.p : 1e9f;
        last_state_ = (lc && !rc) ? "LEFT"
                    : (rc && !lc) ? "RIGHT"
                    : (lp <= rp)  ? "LEFT" : "RIGHT";
        return last_state_;
      }
      const int required = (last_state_ == "CORNER") ? corner_safe_frames_required_
                         : (last_state_ == "LEFT" || last_state_ == "RIGHT") ? side_safe_frames_required_
                         : safe_frames_required_;
      if (++safe_frames_ >= std::max(1, required)) {
        last_state_ = "CLEAR"; safe_frames_ = 0;
      }
      return last_state_;
    }

    // Enter from CLEAR state
    if (isObstacle(zs.front, enter_thresh_)) { last_state_ = "CORNER"; safe_frames_ = 0; return last_state_; }
    if (isObstacle(zs.left,  left_enter_thresh_))  { last_state_ = "LEFT";   safe_frames_ = 0; return last_state_; }
    if (isObstacle(zs.right, right_enter_thresh_)) { last_state_ = "RIGHT";  safe_frames_ = 0; return last_state_; }
    return "CLEAR";
  }

  // ── Logging ────────────────────────────────────────────────────────────────

  void logZoneState(const std::string & zone, const ZoneStats * zs)
  {
    if (!zone_log_enabled_) return;
    if (zone == last_logged_zone_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(),
        (int64_t)std::max(1, zone_log_throttle_ms_), "Zone=%s", zone.c_str());
      return;
    }
    last_logged_zone_ = zone;
    if (!zs) { RCLCPP_INFO(get_logger(), "Zone=%s", zone.c_str()); return; }
    RCLCPP_INFO(get_logger(),
      "Zone=%s  left[p=%.3f near=%d v=%d]  front[p=%.3f near=%d v=%d]  right[p=%.3f near=%d v=%d]",
      zone.c_str(),
      zs->left.p,  zs->left.near_count,  zs->left.valid_count,
      zs->front.p, zs->front.near_count, zs->front.valid_count,
      zs->right.p, zs->right.near_count, zs->right.valid_count);
  }

  // ── ArUco depth masking ────────────────────────────────────────────────────

  void applyArucoDepthMask(cv::Mat & depth, const rclcpp::Time & now)
  {
    std::vector<std::vector<cv::Point2f>> mc;
    std::vector<int> mi;
    { std::lock_guard<std::mutex> lock(marker_mutex_); mc = last_marker_corners_; mi = last_marker_ids_; }
    if (mc.empty()) return;

    const double age_ms = (now - last_aruco_seen_time_).seconds() * 1000.0;
    if (age_ms < 0.0 || age_ms > aruco_seen_hold_ms_) return;

    const int m = std::max(0, aruco_depth_mask_margin_px_);
    for (size_t i = 0; i < mi.size() && i < mc.size(); ++i) {
      if (mi[i] != aruco_target_id_ || mc[i].empty()) continue;
      int x0 = depth.cols - 1, x1 = 0, y0 = depth.rows - 1, y1 = 0;
      for (const auto & p : mc[i]) {
        x0 = std::min(x0, (int)std::floor(p.x)); x1 = std::max(x1, (int)std::ceil(p.x));
        y0 = std::min(y0, (int)std::floor(p.y)); y1 = std::max(y1, (int)std::ceil(p.y));
      }
      x0 = std::clamp(x0 - m, 0, depth.cols - 1); x1 = std::clamp(x1 + m, 0, depth.cols - 1);
      y0 = std::clamp(y0 - m, 0, depth.rows - 1); y1 = std::clamp(y1 + m, 0, depth.rows - 1);
      if (x0 > x1 || y0 > y1) continue;
      depth(cv::Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1)).setTo(std::numeric_limits<float>::quiet_NaN());
    }
  }

  // ── Publishers / Subscribers ───────────────────────────────────────────────

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    zones_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      aruco_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    aruco_direction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   aruco_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   aruco_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   front_obstacle_distance_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::TimerBase::SharedPtr depth_watchdog_timer_;

  cv::Ptr<cv::aruco::Dictionary>         aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  // ── Parameters ─────────────────────────────────────────────────────────────

  double enter_thresh_{0.60}, exit_thresh_{0.80}, emergency_thresh_{0.30};
  double side_enter_thresh_{0.60}, side_exit_thresh_{0.80};
  double left_enter_thresh_{0.60}, right_enter_thresh_{0.60};
  double left_exit_thresh_{0.80},  right_exit_thresh_{0.80};
  double min_depth_{0.08}, max_depth_{10.0};
  int    stride_{2};
  double percentile_{0.10};
  int    near_count_k_{4}, valid_count_min_{60};
  double crop_y0_frac_{0.30}, crop_y1_frac_{0.80}, front_gain_{1.40};
  int    hold_ms_{50}, corner_hold_ms_{30}, side_hold_ms_{50};
  bool   show_debug_{false};
  int    clear_skip_{1};
  int    safe_frames_required_{5}, corner_safe_frames_required_{1}, side_safe_frames_required_{2};
  bool   zone_log_enabled_{true};
  int    zone_log_throttle_ms_{500};
  int    process_every_nth_depth_frame_{1};
  double depth_stall_timeout_s_{1.5};
  bool   depth_watchdog_enabled_{true};

  bool        aruco_enabled_{true}, aruco_debug_{true};
  std::string depth_topic_{"depth_camera/depth_image"};
  std::string rgb_topic_{"camera/camera/color/image_raw"};
  int    aruco_dictionary_id_{cv::aruco::DICT_4X4_100};
  int    aruco_target_id_{1};
  double aruco_center_tolerance_{0.15};
  int    aruco_seen_hold_ms_{150};
  bool   aruco_depth_mask_enabled_{true};
  int    aruco_depth_mask_margin_px_{12};
  double aruco_min_marker_perimeter_rate_{0.015}, aruco_max_marker_perimeter_rate_{4.0};
  int    aruco_adaptive_thresh_win_min_{3}, aruco_adaptive_thresh_win_max_{43},
         aruco_adaptive_thresh_win_step_{4};

  // ── Runtime state ──────────────────────────────────────────────────────────

  cv::Mat            last_depth_;
  rclcpp::Time       last_depth_time_;
  bool               depth_received_once_{false}, depth_missing_warned_{false};
  uint64_t           depth_frame_counter_{0};

  rclcpp::Time       last_aruco_seen_time_;
  bool               aruco_seen_once_{false};
  std::string        last_aruco_direction_{"NONE"};
  float              last_aruco_distance_m_{std::numeric_limits<float>::quiet_NaN()};
  std::mutex         marker_mutex_;
  std::vector<std::vector<cv::Point2f>> last_marker_corners_;
  std::vector<int>   last_marker_ids_;

  std::string last_state_{"CLEAR"}, last_logged_zone_{""}, last_non_clear_zone_{"CORNER"};
  rclcpp::Time last_non_clear_time_;
  int  safe_frames_{0};
  int  clear_skip_counter_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthZoneDetector>());
  rclcpp::shutdown();
}