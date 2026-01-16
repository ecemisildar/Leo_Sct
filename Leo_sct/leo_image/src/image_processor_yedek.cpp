// image_processor.cpp
//
// Lightweight depth-only obstacle zone detector (Ignition/Gazebo Sim) for multi-robot runs.
// Publishes std_msgs/String on "detected_zones" with exactly one of:
//   "LEFT", "RIGHT", "CORNER", "CLEAR"
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
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

class DepthZoneDetector : public rclcpp::Node
{
public:
  DepthZoneDetector() : Node("image_processor")
  {
    zones_pub_ = this->create_publisher<std_msgs::msg::String>("detected_zones", 10);

    // --- Parameters (safe defaults) ---
    enter_thresh_     = this->declare_parameter<double>("enter_thresh", 0.80);  // start avoiding
    exit_thresh_      = this->declare_parameter<double>("exit_thresh",  1.00);  // return to clear
    emergency_thresh_ = this->declare_parameter<double>("emergency_thresh", 0.60);

    min_depth_ = this->declare_parameter<double>("min_depth", 0.08);
    max_depth_ = this->declare_parameter<double>("max_depth", 10.0);

    // Sampling / robustness
    stride_ = this->declare_parameter<int>("stride", 2);                // sampling step in px
    percentile_ = this->declare_parameter<double>("percentile", 0.10);  // 0.10 = 10th percentile
    near_count_k_ = this->declare_parameter<int>("near_count_k", 6);    // near pixels required
    valid_count_min_ = this->declare_parameter<int>("valid_count_min", 20);

    // Crop middle band to reduce floor/ceiling artifacts
    crop_y0_frac_ = this->declare_parameter<double>("crop_y0_frac", 0.30); // start at 30% height
    crop_y1_frac_ = this->declare_parameter<double>("crop_y1_frac", 0.80); // end at 80% height

    // ROI geometry: widen the front region a bit (helps corner cases)
    front_gain_ = this->declare_parameter<double>("front_gain", 1.40);

    // Stability: hold last non-clear decision briefly to prevent flicker
    hold_ms_ = this->declare_parameter<int>("hold_ms", 300);

    // Debug controls
    show_debug_ = this->declare_parameter<bool>("show_debug", false);
    debug_throttle_ms_ = this->declare_parameter<int>("debug_throttle_ms", 500);

    // QoS: match your bridge publisher.
    // If your depth publisher is BEST_EFFORT, switch this to .best_effort().
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "depth_camera/depth_image",
      qos,
      std::bind(&DepthZoneDetector::depthCallback, this, std::placeholders::_1)
    );

    last_state_ = "CLEAR";
    last_non_clear_zone_ = "CORNER";
    last_non_clear_time_ = this->now();
    // Stability: require N consecutive safe frames before exiting avoid mode
    safe_frames_required_ = this->declare_parameter<int>("safe_frames_required", 4);

    // RCLCPP_INFO(this->get_logger(),
    //   "DepthZoneDetector: enter=%.2f exit=%.2f emergency=%.2f p=%.2f stride=%d K=%d crop=[%.2f..%.2f] front_gain=%.2f",
    //   enter_thresh_, exit_thresh_, emergency_thresh_, percentile_, stride_, near_count_k_,
    //   crop_y0_frac_, crop_y1_frac_, front_gain_);
  }

private:
    int safe_frames_required_{4};
    int safe_frames_{0};
  struct RoiStats
  {
    float p;        // percentile depth
    int near_count; // number of samples below enter_thresh
    int valid_count;
  };

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
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

    // Compute stats for each zone
    ZoneStats zs = computeZoneStats(depth);

    // Decide zone using gated percentile distances + hysteresis
    std::string zone_now = determineZoneHysteresis(zs);

    // Hold/debounce to avoid CLEAR flicker
    auto now = this->now();
    if (zone_now != "CLEAR") {
      last_non_clear_zone_ = zone_now;
      last_non_clear_time_ = now;
    } else {
      const double age_ms = (now - last_non_clear_time_).seconds() * 1000.0;
      if (age_ms >= 0.0 && age_ms < static_cast<double>(hold_ms_)) {
        zone_now = last_non_clear_zone_;
      }
    }

    // Publish unchanged message
    std_msgs::msg::String out;
    out.data = zone_now;
    zones_pub_->publish(out);

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

    const float near_thr = static_cast<float>(enter_thresh_);
    zs.left  = roiStats(depth, left,  percentile_, stride_, near_thr);
    zs.front = roiStats(depth, front, percentile_, stride_, near_thr);
    zs.right = roiStats(depth, right, percentile_, stride_, near_thr);

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
      return "CORNER";
    }

    // If we are avoiding, require EXIT threshold to return to CLEAR
    if (last_state_ != "CLEAR") {
        const bool still_close =
        zoneIsObstacle(zs.front, exit_thresh_) ||
        zoneIsObstacle(zs.left,  exit_thresh_) ||
        zoneIsObstacle(zs.right, exit_thresh_);

        if (still_close) {
        safe_frames_ = 0;           // not safe yet
        return last_state_;         // keep avoiding
        }

        // looks safe this frame
        safe_frames_++;

        if (safe_frames_ >= std::max(1, safe_frames_required_)) {
        last_state_ = "CLEAR";
        safe_frames_ = 0;
        return "CLEAR";
        }

        // Not enough consecutive safe frames yet → keep previous avoid state
        return last_state_;
    }

    // If clear, use ENTER threshold to trigger
    if (zoneIsObstacle(zs.front, enter_thresh_)) { last_state_ = "CORNER"; safe_frames_ = 0; return "CORNER"; }
    if (zoneIsObstacle(zs.left,  enter_thresh_)) { last_state_ = "LEFT";   safe_frames_ = 0; return "LEFT";   }
    if (zoneIsObstacle(zs.right, enter_thresh_)) { last_state_ = "RIGHT";  safe_frames_ = 0; return "RIGHT";  }


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

private:
  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zones_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  // Parameters
  double enter_thresh_{0.8};
  double exit_thresh_{1.00};
  double emergency_thresh_{0.60};

  double min_depth_{0.08};
  double max_depth_{10.0};

  int stride_{2};
  double percentile_{0.10};
  int near_count_k_{6};
  int valid_count_min_{20};

  double crop_y0_frac_{0.30};
  double crop_y1_frac_{0.80};

  double front_gain_{1.40};

  int hold_ms_{300};

  bool show_debug_{false};
  int debug_throttle_ms_{500};

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
