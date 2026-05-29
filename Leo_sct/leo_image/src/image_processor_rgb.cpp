// image_processor.cpp
//
// Depth obstacle zones.
// Publishes:
// - std_msgs/String on "detected_zones" with tokens: LEFT, RIGHT, CORNER, CLEAR, GREEN, CYAN
// - std_msgs/Float32 on "front_obstacle_distance" with front ROI near distance in meters
// - std_msgs/Bool on "green_detected"
// - std_msgs/Bool on "cyan_detected"
// - std_msgs/String on "detected_color" with one of: GREEN, CYAN, BOTH, NONE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
    clock_ = this->get_clock();
    const auto now = clock_->now();

    zones_pub_ = this->create_publisher<std_msgs::msg::String>("detected_zones", 10);

    front_obstacle_distance_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("front_obstacle_distance", 10);
    green_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("green_detected", 10);
    cyan_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("cyan_detected", 10);
    detected_color_pub_ = this->create_publisher<std_msgs::msg::String>("detected_color", 10);

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
    color_debug_log_ = this->declare_parameter<bool>("color_debug_log", false);
    clear_skip_ = this->declare_parameter<int>("clear_skip", 3);
    safe_frames_required_ = this->declare_parameter<int>("safe_frames_required", 5);

    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", rgb_topic_);
    green_detection_enabled_ = this->declare_parameter<bool>("green_detection_enabled", true);
    cyan_detection_enabled_ = this->declare_parameter<bool>("cyan_detection_enabled", true);
    color_min_area_ratio_ = this->declare_parameter<double>("color_min_area_ratio", 0.01);
    color_saturation_min_ = this->declare_parameter<int>("color_saturation_min", 50);
    color_value_min_ = this->declare_parameter<int>("color_value_min", 60);
    green_min_area_ratio_ = this->declare_parameter<double>("green_min_area_ratio", color_min_area_ratio_);
    cyan_min_area_ratio_ = this->declare_parameter<double>("cyan_min_area_ratio", color_min_area_ratio_);
    green_saturation_min_ = this->declare_parameter<int>("green_saturation_min", color_saturation_min_);
    cyan_saturation_min_ = this->declare_parameter<int>("cyan_saturation_min", color_saturation_min_);
    green_value_min_ = this->declare_parameter<int>("green_value_min", color_value_min_);
    cyan_value_min_ = this->declare_parameter<int>("cyan_value_min", color_value_min_);
    green_hue_min_ = this->declare_parameter<int>("green_hue_min", 40);
    green_hue_max_ = this->declare_parameter<int>("green_hue_max", 85);
    cyan_hue_min_ = this->declare_parameter<int>("cyan_hue_min", 85);
    cyan_hue_max_ = this->declare_parameter<int>("cyan_hue_max", 105);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "depth_camera/depth_image",
      qos,
      std::bind(&DepthZoneDetector::depthCallback, this, std::placeholders::_1));
    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_,
      qos,
      std::bind(&DepthZoneDetector::rgbCallback, this, std::placeholders::_1));

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
        out.data = detectedZoneMessage(last_state_);
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
    ZoneStats zs = computeZoneStats(depth);
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
    current_zone_ = zone_now;
    out.data = detectedZoneMessage(current_zone_);
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
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Unsupported RGB encoding for color detection: %s", encoding.c_str());
        publishColorDetection(false, false);
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
      publishColorDetection(false, false);
      return;
    }

    if (bgr.empty()) {
      publishColorDetection(false, false);
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    const int green_sat_min = std::clamp(green_saturation_min_, 0, 255);
    const int cyan_sat_min = std::clamp(cyan_saturation_min_, 0, 255);
    const int green_val_min = std::clamp(green_value_min_, 0, 255);
    const int cyan_val_min = std::clamp(cyan_value_min_, 0, 255);
    cv::Mat green_mask;
    cv::Mat cyan_mask;

    cv::inRange(
      hsv,
      cv::Scalar(std::clamp(green_hue_min_, 0, 179), green_sat_min, green_val_min),
      cv::Scalar(std::clamp(green_hue_max_, 0, 179), 255, 255),
      green_mask);
    cv::inRange(
      hsv,
      cv::Scalar(std::clamp(cyan_hue_min_, 0, 179), cyan_sat_min, cyan_val_min),
      cv::Scalar(std::clamp(cyan_hue_max_, 0, 179), 255, 255),
      cyan_mask);

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(cyan_mask, cyan_mask, cv::MORPH_OPEN, kernel);

    const double image_area = static_cast<double>(bgr.rows * bgr.cols);
    const int green_area_px = cv::countNonZero(green_mask);
    const int cyan_area_px = cv::countNonZero(cyan_mask);
    const double green_min_area =
      std::max(1.0, std::clamp(green_min_area_ratio_, 0.0, 1.0) * image_area);
    const double cyan_min_area =
      std::max(1.0, std::clamp(cyan_min_area_ratio_, 0.0, 1.0) * image_area);
    const bool green_detected =
      green_detection_enabled_ && static_cast<double>(green_area_px) >= green_min_area;
    const bool cyan_detected =
      cyan_detection_enabled_ && static_cast<double>(cyan_area_px) >= cyan_min_area;

    if (color_debug_log_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "color mask ratios green=%.4f/%0.4f cyan=%.4f/%0.4f detected green=%s cyan=%s",
        static_cast<double>(green_area_px) / std::max(1.0, image_area),
        std::clamp(green_min_area_ratio_, 0.0, 1.0),
        static_cast<double>(cyan_area_px) / std::max(1.0, image_area),
        std::clamp(cyan_min_area_ratio_, 0.0, 1.0),
        green_detected ? "true" : "false",
        cyan_detected ? "true" : "false");
    }

    publishColorDetection(green_detected, cyan_detected);

    if (show_debug_) {
      cv::Mat color_vis = bgr.clone();
      color_vis.setTo(cv::Scalar(0, 255, 0), green_mask);
      color_vis.setTo(cv::Scalar(160, 250, 255), cyan_mask);
      cv::imshow("Color detection (debug)", color_vis);
      cv::waitKey(1);
    }
  }

  void publishColorDetection(bool green_detected, bool cyan_detected)
  {
    green_detected_ = green_detected;
    cyan_detected_ = cyan_detected;

    std_msgs::msg::Bool green_msg;
    green_msg.data = green_detected;
    green_detected_pub_->publish(green_msg);

    std_msgs::msg::Bool cyan_msg;
    cyan_msg.data = cyan_detected;
    cyan_detected_pub_->publish(cyan_msg);

    std_msgs::msg::String color_msg;
    if (green_detected && cyan_detected) {
      color_msg.data = "BOTH";
    } else if (green_detected) {
      color_msg.data = "GREEN";
    } else if (cyan_detected) {
      color_msg.data = "CYAN";
    } else {
      color_msg.data = "NONE";
    }
    detected_color_pub_->publish(color_msg);

    std_msgs::msg::String zones_msg;
    zones_msg.data = detectedZoneMessage(current_zone_);
    zones_pub_->publish(zones_msg);
  }

  std::string detectedZoneMessage(const std::string & obstacle_zone) const
  {
    std::vector<std::string> tokens;
    if (obstacle_zone == "LEFT" || obstacle_zone == "RIGHT" || obstacle_zone == "CORNER") {
      tokens.push_back(obstacle_zone);
    }
    if (green_detected_) {
      tokens.push_back("GREEN");
    }
    if (cyan_detected_) {
      tokens.push_back("CYAN");
    }
    if (tokens.empty()) {
      return "CLEAR";
    }

    std::string out = tokens.front();
    for (size_t i = 1; i < tokens.size(); ++i) {
      out += ",";
      out += tokens[i];
    }
    return out;
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
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_obstacle_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr green_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cyan_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detected_color_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Clock::SharedPtr clock_;

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
  bool color_debug_log_{false};
  int clear_skip_{1};
  int clear_skip_counter_{0};

  int safe_frames_required_{5};
  int safe_frames_{0};

  std::string rgb_topic_{"camera/camera/color/image_raw"};
  bool green_detection_enabled_{true};
  bool cyan_detection_enabled_{true};
  double color_min_area_ratio_{0.01};
  int color_saturation_min_{50};
  int color_value_min_{60};
  double green_min_area_ratio_{0.01};
  double cyan_min_area_ratio_{0.01};
  int green_saturation_min_{50};
  int cyan_saturation_min_{50};
  int green_value_min_{60};
  int cyan_value_min_{60};
  int green_hue_min_{40};
  int green_hue_max_{85};
  int cyan_hue_min_{85};
  int cyan_hue_max_{105};

  bool green_detected_{false};
  bool cyan_detected_{false};
  std::string current_zone_{"CLEAR"};

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
