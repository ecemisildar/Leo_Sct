// image_processor.cpp
//
// Lightweight depth-only obstacle zone detector (Ignition/Gazebo Sim) for multi-robot runs.
// Publishes std_msgs/String on "detected_zones" with exactly one of:
//   "LEFT", "RIGHT", "CORNER", "CLEAR", "marker"
// Publishes std_msgs/Bool on "marker_front" when ArUco is detected in front-center view.
//
// Improvements vs naive mean/min:
// - Depth-only subscription (no RGB sync)
// - Middle-band ROI crop (avoids floor/ceiling artifacts)
// - Robust statistic: 10th percentile (p10) instead of min
// - "Near pixel count" gate to avoid single-pixel false obstacles
// - Hysteresis + hold/debounce to reduce flicker
// - Optional debug visualization (off by default)
//
// Tuned starting values for v=0.2 m/s, w=0.5 rad/s.
// You can override any parameter at runtime with --ros-args -p name:=value.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
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
    marker_seen_pub_ = this->create_publisher<std_msgs::msg::Bool>("marker_seen", 10);
    marker_lost_pub_ = this->create_publisher<std_msgs::msg::Bool>("marker_lost", 10);
    // marker_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("marker_distance", 10);
    marker_front_pub_ = this->create_publisher<std_msgs::msg::Bool>("marker_front", 10);

    // --- Parameters (safe conservative defaults for real robots) ---
    enter_thresh_     = this->declare_parameter<double>("enter_thresh", 0.80);  // start avoiding earlier
    exit_thresh_      = this->declare_parameter<double>("exit_thresh",  1.05);  // return to clear later
    emergency_thresh_ = this->declare_parameter<double>("emergency_thresh", 0.45);
    side_enter_thresh_ = this->declare_parameter<double>("side_enter_thresh", enter_thresh_);
    side_exit_thresh_  = this->declare_parameter<double>("side_exit_thresh", exit_thresh_);
    left_enter_thresh_ = this->declare_parameter<double>("left_enter_thresh", side_enter_thresh_);
    right_enter_thresh_ = this->declare_parameter<double>("right_enter_thresh", side_enter_thresh_);
    left_exit_thresh_ = this->declare_parameter<double>("left_exit_thresh", side_exit_thresh_);
    right_exit_thresh_ = this->declare_parameter<double>("right_exit_thresh", side_exit_thresh_);

    min_depth_ = this->declare_parameter<double>("min_depth", 0.08);
    max_depth_ = this->declare_parameter<double>("max_depth", 10.0);

    // Sampling / robustness
    stride_ = this->declare_parameter<int>("stride", 2);                // sampling step in px
    percentile_ = this->declare_parameter<double>("percentile", 0.10);  // 0.10 = 10th percentile
    near_count_k_ = this->declare_parameter<int>("near_count_k", 4);    // near pixels required
    valid_count_min_ = this->declare_parameter<int>("valid_count_min", 60);

    // Crop middle band to reduce floor/ceiling artifacts
    crop_y0_frac_ = this->declare_parameter<double>("crop_y0_frac", 0.30); // start at 30% height
    crop_y1_frac_ = this->declare_parameter<double>("crop_y1_frac", 0.80); // end at 80% height

    // ROI geometry: widen the front region a bit (helps corner cases)
    front_gain_ = this->declare_parameter<double>("front_gain", 1.40);

    // Stability: hold last non-clear decision briefly to prevent flicker
    hold_ms_ = this->declare_parameter<int>("hold_ms", 250);

    // Debug controls
    show_debug_ = this->declare_parameter<bool>("show_debug", false);
    debug_throttle_ms_ = this->declare_parameter<int>("debug_throttle_ms", 500);

    // When CLEAR, only process every Nth depth frame to save CPU.
    clear_skip_ = this->declare_parameter<int>("clear_skip", 3);

    // QoS: match your bridge publisher.
    // If your depth publisher is BEST_EFFORT, switch this to .best_effort().
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto rgb_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "depth_camera/depth_image",
      qos,
      std::bind(&DepthZoneDetector::depthCallback, this, std::placeholders::_1)
    );

    // ArUco detection (publishes on existing "marker_*" topics for compatibility)
    marker_enabled_ = this->declare_parameter<bool>("marker_enabled", false);
    marker_debug_ = this->declare_parameter<bool>("marker_debug", false);
    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "depth_camera/image");
    marker_label_ = this->declare_parameter<std::string>("marker_label", "marker");
    aruco_dictionary_id_ = this->declare_parameter<int>("aruco_dictionary_id", cv::aruco::DICT_6X6_250);
    aruco_corner_refine_ = this->declare_parameter<bool>("aruco_corner_refine", true);
    marker_window_radius_ = this->declare_parameter<int>("aruco_window_radius", 2);
    aruco_try_inverted_ = this->declare_parameter<bool>("aruco_try_inverted", true);
    aruco_fallback_dicts_ = this->declare_parameter<std::vector<int64_t>>(
      "aruco_fallback_dicts",
      std::vector<int64_t>{
        cv::aruco::DICT_6X6_250,
        cv::aruco::DICT_6X6_50,
        cv::aruco::DICT_6X6_100,
        cv::aruco::DICT_6X6_1000
      });
    aruco_min_perimeter_rate_ = this->declare_parameter<double>("aruco_min_perimeter_rate", 0.01);
    aruco_adaptive_win_min_ = this->declare_parameter<int>("aruco_adaptive_win_min", 3);
    aruco_adaptive_win_max_ = this->declare_parameter<int>("aruco_adaptive_win_max", 53);
    aruco_adaptive_win_step_ = this->declare_parameter<int>("aruco_adaptive_win_step", 10);
    aruco_corner_refine_win_ = this->declare_parameter<int>("aruco_corner_refine_win", 7);
    aruco_polygonal_accuracy_ = this->declare_parameter<double>("aruco_polygonal_accuracy", 0.05);

    // Legacy marker color parameters (kept for launch-file compatibility)
    marker_min_ = this->declare_parameter<int>("marker_min", 80);
    marker_ratio_ = this->declare_parameter<double>("marker_ratio", 1.6);
    marker_pixel_ratio_ = this->declare_parameter<double>("marker_pixel_ratio", 0.002);
    marker_stride_ = this->declare_parameter<int>("marker_stride", 2);
    marker_hold_ms_ = this->declare_parameter<int>("marker_hold_ms", 2000);
    marker_lost_hold_ms_ = this->declare_parameter<int>("marker_lost_hold_ms", 600);
    marker_distance_min_count_ = this->declare_parameter<int>("marker_distance_min_count", 6);
    marker_depth_max_age_ms_ = this->declare_parameter<int>("marker_depth_max_age_ms", 200);
    marker_front_hold_ms_ = this->declare_parameter<int>("marker_front_hold_ms", 600);
    marker_forward_id_ = this->declare_parameter<int>("marker_forward_id", 0);

    aruco_dict_ = cv::aruco::getPredefinedDictionary(aruco_dictionary_id_);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->cornerRefinementMethod =
      aruco_corner_refine_ ? cv::aruco::CORNER_REFINE_SUBPIX : cv::aruco::CORNER_REFINE_NONE;
    aruco_params_->minMarkerPerimeterRate = static_cast<float>(aruco_min_perimeter_rate_);
    aruco_params_->adaptiveThreshWinSizeMin = std::max(3, aruco_adaptive_win_min_);
    aruco_params_->adaptiveThreshWinSizeMax = std::max(aruco_params_->adaptiveThreshWinSizeMin, aruco_adaptive_win_max_);
    aruco_params_->adaptiveThreshWinSizeStep = std::max(1, aruco_adaptive_win_step_);
    aruco_params_->cornerRefinementWinSize = std::max(1, aruco_corner_refine_win_);
    aruco_params_->polygonalApproxAccuracyRate = static_cast<float>(aruco_polygonal_accuracy_);

    if (marker_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Marker detection enabled. RGB topic: %s", rgb_topic_.c_str());
      rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_,
        rgb_qos,
        std::bind(&DepthZoneDetector::rgbCallback, this, std::placeholders::_1)
      );
    } else {
      RCLCPP_WARN(this->get_logger(), "Marker detection disabled (marker_enabled=false).");
    }

    last_state_ = "CLEAR";
    last_non_clear_zone_ = "CORNER";
    last_non_clear_time_ = now;
    last_marker_enabled_ = false;
    last_marker_time_ = now;
    last_marker_lost_time_ = now;
    last_marker_front_time_ = now;
    last_depth_time_ = now;
    // last_marker_distance_time_ = now;
    // Stability: require N consecutive safe frames before exiting avoid mode
    safe_frames_required_ = this->declare_parameter<int>("safe_frames_required", 5);

    // RCLCPP_INFO(this->get_logger(),
    //   "DepthZoneDetector: enter=%.2f exit=%.2f emergency=%.2f p=%.2f stride=%d K=%d crop=[%.2f..%.2f] front_gain=%.2f",
    //   enter_thresh_, exit_thresh_, emergency_thresh_, percentile_, stride_, near_count_k_,
    //   crop_y0_frac_, crop_y1_frac_, front_gain_);
  }

private:
    int safe_frames_required_{5};
    int safe_frames_{0};
  struct RoiStats
  {
    float p;        // percentile depth
    int near_count; // number of samples below enter_thresh
    int valid_count;
  };

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
    const auto now = clock_->now();
    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), debug_throttle_ms_,
    //   "Depth frame received: marker_enabled=%s", marker_enabled_ ? "true" : "false");

    if (last_state_ == "CLEAR" && clear_skip_ > 1) {
      clear_skip_counter_ = (clear_skip_counter_ + 1) % clear_skip_;
      if (clear_skip_counter_ != 0) {
        std_msgs::msg::String out;
        if (markerEnabledNow(now)) {
          out.data = marker_label_;
        } else {
          out.data = last_state_;
        }
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
        // mm -> m float
        depth.convertTo(depth, CV_32FC1, 0.001);
      } else if (encoding == "32FC1") {
        if (depth.type() != CV_32FC1) depth.convertTo(depth, CV_32FC1);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Unsupported depth encoding: %s (expected 16UC1 or 32FC1)", encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (depth.empty() || depth.cols < 6 || depth.rows < 6) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Empty/small depth image.");
      return;
    }

    last_depth_ = depth.clone();
    last_depth_time_ = now;

    // Compute stats for each zone
    ZoneStats zs = computeZoneStats(depth);

    // Decide zone using gated percentile distances + hysteresis
    std::string zone_now = determineZoneHysteresis(zs);

    // Hold/debounce to avoid CLEAR flicker
    if (zone_now != "CLEAR") {
      last_non_clear_zone_ = zone_now;
      last_non_clear_time_ = now;
    } else {
      const double age_ms = (now - last_non_clear_time_).seconds() * 1000.0;
      if (age_ms >= 0.0 && age_ms < static_cast<double>(hold_ms_)) {
        zone_now = last_non_clear_zone_;
      }
    }

    // Publish final state (marker only when depth says CLEAR)
    const bool marker_now = markerEnabledNow(now);
    if (last_marker_enabled_ && !marker_now && marker_enabled_) {
      last_marker_lost_time_ = now;
    }
    last_marker_enabled_ = marker_now;

    std_msgs::msg::String out;
    if (zone_now == "CLEAR" && marker_now) {
      out.data = marker_label_;
    } else {
      out.data = zone_now;
    }
    zones_pub_->publish(out);

    std_msgs::msg::Bool marker_seen_msg;
    marker_seen_msg.data = marker_now;
    // marker_seen_pub_->publish(marker_seen_msg);

    std_msgs::msg::Bool marker_lost_msg;
    marker_lost_msg.data = markerLostNow(now);
    // marker_lost_pub_->publish(marker_lost_msg);

    // Marker distance path disabled intentionally.
    // std_msgs::msg::Float32 marker_distance_msg;
    // if (marker_seen_msg.data && markerDistanceValid(now)) {
    //   marker_distance_msg.data = last_marker_distance_;
    // } else {
    //   marker_distance_msg.data = std::numeric_limits<float>::quiet_NaN();
    // }
    // marker_distance_pub_->publish(marker_distance_msg);
    if (marker_enabled_) {
      std_msgs::msg::Bool marker_front_msg;
      marker_front_msg.data = markerFrontNow(now);
      marker_seen_pub_->publish(marker_seen_msg);
      marker_lost_pub_->publish(marker_lost_msg);
      // marker_distance_pub_->publish(marker_distance_msg);
      marker_front_pub_->publish(marker_front_msg);
      if (marker_debug_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Marker pubs: seen=%s lost=%s front=%s",
          marker_seen_msg.data ? "true" : "false",
          marker_lost_msg.data ? "true" : "false",
          marker_front_msg.data ? "true" : "false");
      }
    }

    // Throttled debug log
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), debug_throttle_ms_,
    //   "p%.0f L %.2f (near %d/%d) | F %.2f (near %d/%d) | R %.2f (near %d/%d) -> %s",
    //   percentile_ * 100.0,
    //   zs.left.p,  zs.left.near_count,  zs.left.valid_count,
    //   zs.front.p, zs.front.near_count, zs.front.valid_count,
    //   zs.right.p, zs.right.near_count, zs.right.valid_count,
    //   zone_now.c_str());

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

  struct ZoneStats
  {
    RoiStats left;
    RoiStats front;
    RoiStats right;
    cv::Rect left_roi;
    cv::Rect front_roi;
    cv::Rect right_roi;
  };

  ZoneStats computeZoneStats(const cv::Mat& depth)
  {
    const int w = depth.cols;
    const int h = depth.rows;

    // Vertical crop
    int y0 = static_cast<int>(std::round(std::clamp(crop_y0_frac_, 0.0, 0.95) * h));
    int y1 = static_cast<int>(std::round(std::clamp(crop_y1_frac_, 0.05, 1.0) * h));
    if (y1 <= y0 + 1) { y0 = 0; y1 = h; }
    int roi_h = std::max(1, y1 - y0);

    // Horizontal split: widen front region
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
    zs.left  = roiStats(depth, left,  percentile_, stride_, left_near_thr);
    zs.front = roiStats(depth, front, percentile_, stride_, front_near_thr);
    zs.right = roiStats(depth, right, percentile_, stride_, right_near_thr);

    return zs;
  }

  RoiStats roiStats(const cv::Mat& depth, const cv::Rect& roi,
                    double percentile, int stride, float near_thresh)
  {
    std::vector<float> vals;
    const int st = std::max(1, stride);
    vals.reserve((roi.width / st + 1) * (roi.height / st + 1));

    const float min_d = static_cast<float>(min_depth_);
    const float max_d = static_cast<float>(max_depth_);

    int near_cnt = 0;
    int valid_cnt = 0;

    for (int y = roi.y; y < roi.y + roi.height; y += st) {
      const float* row = depth.ptr<float>(y);
      for (int x = roi.x; x < roi.x + roi.width; x += st) {
        float d = row[x];
        if (std::isfinite(d) && d > min_d && d < max_d) {
          valid_cnt++;
          if (d < near_thresh) near_cnt++;
          vals.push_back(d);
        }
      }
    }

    if (vals.empty()) return {-1.0f, 0, 0};

    // clamp percentile to [0..1]
    double p = std::clamp(percentile, 0.0, 1.0);
    size_t k = static_cast<size_t>(std::round(p * static_cast<double>(vals.size() - 1)));
    k = std::min(k, vals.size() - 1);

    std::nth_element(vals.begin(), vals.begin() + k, vals.end());
    float pk = vals[k];

    return {pk, near_cnt, valid_cnt};
  }

  bool zoneIsObstacle(const RoiStats& rs, double thresh) const
  {
    if (rs.p <= 0.0f) return false;
    if (rs.valid_count < valid_count_min_) return false; // not enough data
    if (rs.p >= static_cast<float>(thresh)) return false;
    // gate: require multiple near samples to avoid single-pixel artifact
    return rs.near_count >= near_count_k_;
  }

  std::string determineZoneHysteresis(const ZoneStats& zs)
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

    // -------------------------
    // AVOID MODE (not CLEAR): use EXIT threshold, but allow re-targeting
    // -------------------------
    if (last_state_ != "CLEAR") {
      const bool front_close = zoneIsObstacle(zs.front, exit_thresh_);
      const bool left_close  = zoneIsObstacle(zs.left,  left_exit_thresh_);
      const bool right_close = zoneIsObstacle(zs.right, right_exit_thresh_);

      const bool still_close = front_close || left_close || right_close;

      if (still_close) {
        safe_frames_ = 0;

        if (front_close) {
          last_state_ = "CORNER";
          return last_state_;
        }

        // If not front, pick the closer side by percentile depth (smaller p = closer)
        const float lp = (zs.left.p  > 0.0f) ? zs.left.p  : 1e9f;
        const float rp = (zs.right.p > 0.0f) ? zs.right.p : 1e9f;

        if (left_close || right_close) {
          if (left_close && !right_close) last_state_ = "LEFT";
          else if (right_close && !left_close) last_state_ = "RIGHT";
          else last_state_ = (lp <= rp) ? "LEFT" : "RIGHT";  // both close -> closest ROI
        }

        return last_state_;
      }

      // looks safe this frame (no close obstacles under EXIT threshold)
      safe_frames_++;
      if (safe_frames_ >= std::max(1, safe_frames_required_)) {
        last_state_ = "CLEAR";
        safe_frames_ = 0;
        return last_state_;
      }

      // Not enough consecutive safe frames yet -> keep last avoid state
      return last_state_;
    }

    // -------------------------
    // CLEAR MODE: use ENTER threshold to trigger
    // -------------------------
    if (zoneIsObstacle(zs.front, enter_thresh_)) { last_state_ = "CORNER"; safe_frames_ = 0; return last_state_; }
    if (zoneIsObstacle(zs.left,  left_enter_thresh_)) { last_state_ = "LEFT";   safe_frames_ = 0; return last_state_; }
    if (zoneIsObstacle(zs.right, right_enter_thresh_)) { last_state_ = "RIGHT";  safe_frames_ = 0; return last_state_; }

    return "CLEAR";
  }


  void drawROIs(cv::Mat& vis)
  {
    // Recompute ROIs based on current params
    const int w = vis.cols;
    const int h = vis.rows;

    int y0 = static_cast<int>(std::round(std::clamp(crop_y0_frac_, 0.0, 0.95) * h));
    int y1 = static_cast<int>(std::round(std::clamp(crop_y1_frac_, 0.05, 1.0) * h));
    if (y1 <= y0 + 1) { y0 = 0; y1 = h; }
    int roi_h = std::max(1, y1 - y0);

    const double fg = std::max(1.0, front_gain_);
    int front_w = static_cast<int>(std::round((w / 3.0) * fg));
    front_w = std::clamp(front_w, 1, w - 2);

    int side_total = w - front_w;
    int left_w = std::max(1, side_total / 2);
    int right_w = std::max(1, w - front_w - left_w);
    int front_x = left_w;

    cv::rectangle(vis, cv::Rect(0, y0, left_w, roi_h), cv::Scalar(255,255,255), 1);
    cv::rectangle(vis, cv::Rect(front_x, y0, front_w, roi_h), cv::Scalar(255,255,255), 1);
    cv::rectangle(vis, cv::Rect(front_x + front_w, y0, right_w, roi_h), cv::Scalar(255,255,255), 1);
  }

  void rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg)
  { 
    if (!marker_enabled_) return;
    cv::Mat bgr;
    std::string encoding;

    try {
      auto rgb_ptr = cv_bridge::toCvShare(rgb_msg);
      encoding = rgb_ptr->encoding;
      if (encoding == "bgr8") {
        bgr = rgb_ptr->image;
      } else if (encoding == "rgb8") {
        cv::cvtColor(rgb_ptr->image, bgr, cv::COLOR_RGB2BGR);
      } else if (encoding == "bgra8") {
        cv::cvtColor(rgb_ptr->image, bgr, cv::COLOR_BGRA2BGR);
      } else if (encoding == "rgba8") {
        cv::cvtColor(rgb_ptr->image, bgr, cv::COLOR_RGBA2BGR);
      } else if (encoding == "mono8" || encoding == "8UC1") {
        bgr = rgb_ptr->image;
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Unsupported RGB encoding: %s (expected bgr8/rgb8/bgra8/rgba8/mono8)", encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
      return;
    }

    if (bgr.empty()) return;
    if (marker_debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "RGB callback on %s: %dx%d (%s)",
        rgb_topic_.c_str(), bgr.cols, bgr.rows, encoding.c_str());
    }
    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), debug_throttle_ms_,
    //   "RGB frame received: %dx%d (%s)", bgr.cols, bgr.rows, encoding.c_str());
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //   "RGB frame received on %s: %dx%d (%s)",
    //   this->get_fully_qualified_name(), bgr.cols, bgr.rows, encoding.c_str());

    cv::Mat gray;
    if (encoding == "mono8" || encoding == "8UC1") {
      gray = bgr;
    } else {
      cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    const bool detected = detectAruco(gray, corners, ids);
    const int dict_report =
      (last_detected_dict_id_ >= 0) ? last_detected_dict_id_ : aruco_dictionary_id_;
    //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      //"ArUco detections on %s: %zu (dict=%d, inverted=%s)",
      //this->get_fully_qualified_name(),
      //ids.size(),
      //dict_report,
      //last_detected_inverted_ ? "true" : "false");

    if (detected) {
      const auto now = this->now();
      last_marker_time_ = now;
      marker_seen_ = true;
      const bool front_now = markerHasForwardId(ids);
      if (front_now) {
        marker_front_seen_ = true;
        last_marker_front_time_ = now;
      }
      if (marker_debug_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "ArUco detected: count=%zu dict=%d inverted=%s front=%s",
          ids.size(), dict_report, last_detected_inverted_ ? "true" : "false",
          front_now ? "true" : "false");
      }
      if (!ids.empty()) {
        std::ostringstream id_stream;
        for (size_t i = 0; i < ids.size(); ++i) {
          if (i > 0) id_stream << ",";
          id_stream << ids[i];
        }
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //   "Marker detected on %s (ids=[%s])",
        //   this->get_fully_qualified_name(), id_stream.str().c_str());
      }
      // updateMarkerDistance(corners, now, bgr.size());
    } else if (marker_debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No ArUco detected in RGB frame (dict=%d, try_inverted=%s)",
        aruco_dictionary_id_, aruco_try_inverted_ ? "true" : "false");
    }
  }

  bool detectAruco(const cv::Mat& gray,
                   std::vector<std::vector<cv::Point2f>>& corners,
                   std::vector<int>& ids) const
  {
    if (gray.empty()) return false;
    last_detected_dict_id_ = -1;
    last_detected_inverted_ = false;

    auto try_detect = [&](const cv::Mat& img, bool inverted) -> bool {
      std::vector<int> dicts;
      dicts.reserve(1 + aruco_fallback_dicts_.size());
      dicts.push_back(aruco_dictionary_id_);
      for (int64_t dict_id : aruco_fallback_dicts_) {
        const int dict_id_int = static_cast<int>(dict_id);
        if (dict_id_int != aruco_dictionary_id_) {
          dicts.push_back(dict_id_int);
        }
      }

      for (int dict_id : dicts) {
        const auto dict =
          (dict_id == aruco_dictionary_id_) ? aruco_dict_ : cv::aruco::getPredefinedDictionary(dict_id);
        cv::aruco::detectMarkers(img, dict, corners, ids, aruco_params_);
        if (!ids.empty()) {
          last_detected_dict_id_ = dict_id;
          last_detected_inverted_ = inverted;
          return true;
        }
      }
      return false;
    };

    if (try_detect(gray, false)) return true;

    if (aruco_try_inverted_) {
      cv::Mat inverted;
      cv::bitwise_not(gray, inverted);
      if (try_detect(inverted, true)) return true;
    }

    return false;
  }

  bool markerEnabledNow(const rclcpp::Time& now) const
  {
    if (!marker_enabled_ || !marker_seen_) return false;
    const double age_ms = (now - last_marker_time_).seconds() * 1000.0;
    return age_ms >= 0.0 && age_ms < static_cast<double>(marker_hold_ms_);
  }

  bool markerFrontNow(const rclcpp::Time& now) const
  {
    if (!marker_enabled_ || !marker_front_seen_) return false;
    const double age_ms = (now - last_marker_front_time_).seconds() * 1000.0;
    return age_ms >= 0.0 && age_ms < static_cast<double>(marker_front_hold_ms_);
  }

  bool markerHasForwardId(const std::vector<int>& ids) const
  {
    return std::find(ids.begin(), ids.end(), marker_forward_id_) != ids.end();
  }

  // bool markerDistanceValid(const rclcpp::Time& now) const
  // {
  //   if (!marker_distance_valid_) return false;
  //   const double age_ms = (now - last_marker_distance_time_).seconds() * 1000.0;
  //   return age_ms >= 0.0 && age_ms < static_cast<double>(marker_hold_ms_);
  // }

  bool markerLostNow(const rclcpp::Time& now) const
  {
    if (!marker_enabled_) return false;
    const double age_ms = (now - last_marker_lost_time_).seconds() * 1000.0;
    return age_ms >= 0.0 && age_ms < static_cast<double>(marker_lost_hold_ms_);
  }

  // void updateMarkerDistance(const std::vector<std::vector<cv::Point2f>>& corners,
  //                           const rclcpp::Time& now,
  //                           const cv::Size& rgb_size)
  // {
  // }

private:
  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zones_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr marker_seen_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr marker_lost_pub_;
  // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr marker_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr marker_front_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Clock::SharedPtr clock_;

  // Parameters
  double enter_thresh_{0.80};
  double exit_thresh_{1.05};
  double emergency_thresh_{0.45};
  double side_enter_thresh_{0.80};
  double side_exit_thresh_{1.05};
  double left_enter_thresh_{0.80};
  double right_enter_thresh_{0.80};
  double left_exit_thresh_{1.05};
  double right_exit_thresh_{1.05};

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
  int debug_throttle_ms_{500};
  int clear_skip_{1};
  int clear_skip_counter_{0};

  // ArUco detection (publish on marker topics)
  bool marker_enabled_{false};
  bool marker_debug_{false};
  std::string rgb_topic_{"camera/image_raw"};
  std::string marker_label_{"marker"};
  int aruco_dictionary_id_{cv::aruco::DICT_6X6_250};
  bool aruco_corner_refine_{true};
  int marker_window_radius_{2};
  bool aruco_try_inverted_{true};
  std::vector<int64_t> aruco_fallback_dicts_;
  double aruco_min_perimeter_rate_{0.01};
  int aruco_adaptive_win_min_{3};
  int aruco_adaptive_win_max_{53};
  int aruco_adaptive_win_step_{10};
  int aruco_corner_refine_win_{7};
  double aruco_polygonal_accuracy_{0.05};
  int marker_hold_ms_{500};
  int marker_lost_hold_ms_{600};
  int marker_front_hold_ms_{600};
  int marker_forward_id_{0};
  int marker_distance_min_count_{6};
  int marker_depth_max_age_ms_{200};
  bool marker_seen_{false};
  bool marker_front_seen_{false};
  bool last_marker_enabled_{false};
  rclcpp::Time last_marker_time_;
  rclcpp::Time last_marker_lost_time_;
  rclcpp::Time last_marker_front_time_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
  mutable int last_detected_dict_id_{-1};
  mutable bool last_detected_inverted_{false};

  // Legacy marker parameters (unused)
  int marker_min_{80};
  double marker_ratio_{1.4};
  double marker_pixel_ratio_{0.01};
  int marker_stride_{4};

  cv::Mat last_depth_;
  rclcpp::Time last_depth_time_;
  float last_marker_distance_{std::numeric_limits<float>::infinity()};
  rclcpp::Time last_marker_distance_time_;
  bool marker_distance_valid_{false};

  // State
  std::string last_state_{"CLEAR"};
  std::string last_non_clear_zone_{"CORNER"};
  rclcpp::Time last_non_clear_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthZoneDetector>());
  rclcpp::shutdown();
  return 0;
}
