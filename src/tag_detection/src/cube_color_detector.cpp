#include <memory>
#include <string>
#include <cstring>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// AprilTag (system lib: libapriltag-dev)
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>

class CubeColorDetector : public rclcpp::Node
{
public:
  CubeColorDetector() : Node("cube_color_detector")
  {
    declare_parameter<std::string>("image_topic", "/rgb_camera/image");
    declare_parameter<std::string>("camera_info_topic", "/rgb_camera/camera_info");
    declare_parameter<int>("red_id", 1);
    declare_parameter<int>("blue_id", 10);

    // ROI control
    declare_parameter<int>("roi_half", 55);        // half-size of ROI around tag center
    declare_parameter<int>("exclude_half", 25);    // exclude central square (tag area)
    declare_parameter<bool>("print_once", true);   // latch result per tag
    declare_parameter<bool>("require_confident", true); // if unknown, keep trying
    declare_parameter<bool>("shutdown_after_both", true);

    image_topic_ = get_parameter("image_topic").as_string();
    caminfo_topic_ = get_parameter("camera_info_topic").as_string();
    red_id_ = get_parameter("red_id").as_int();
    blue_id_ = get_parameter("blue_id").as_int();
    roi_half_ = get_parameter("roi_half").as_int();
    exclude_half_ = get_parameter("exclude_half").as_int();
    print_once_ = get_parameter("print_once").as_bool();
    require_confident_ = get_parameter("require_confident").as_bool();
    shutdown_after_both_ = get_parameter("shutdown_after_both").as_bool();

    sub_caminfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      caminfo_topic_, 10,
      std::bind(&CubeColorDetector::camInfoCb, this, std::placeholders::_1));

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&CubeColorDetector::imageCb, this, std::placeholders::_1));

    // AprilTag detector
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);
    td_->quad_decimate = 1.0;
    td_->quad_sigma = 0.0;
    td_->nthreads = 2;
    td_->refine_edges = 1;

    RCLCPP_INFO(get_logger(),
      "cube_color_detector started.\n  image: %s\n  ids: red=%d blue=%d\n  roi_half=%d exclude_half=%d\n  print_once=%s require_confident=%s",
      image_topic_.c_str(), red_id_, blue_id_, roi_half_, exclude_half_,
      print_once_ ? "true" : "false", require_confident_ ? "true" : "false");
  }

  ~CubeColorDetector() override
  {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

private:
  std::string image_topic_;
  std::string caminfo_topic_;
  int red_id_{1};
  int blue_id_{10};

  int roi_half_{55};
  int exclude_half_{25};
  bool print_once_{true};
  bool require_confident_{true};
  bool shutdown_after_both_{false};

  bool caminfo_ready_{false};

  // latch printed results
  bool printed_red_{false};
  bool printed_blue_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;

  apriltag_family_t * tf_{nullptr};
  apriltag_detector_t * td_{nullptr};

  static inline int clampi(int v, int lo, int hi)
  {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr)
  {
    caminfo_ready_ = true;
  }

  // Convert ROS image -> BGR correctly (fixes red/blue swap)
  bool toBGR(const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & out_bgr)
  {
    try {
      // cv_bridge can convert encodings reliably if we request "bgr8"
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      out_bgr = cv_ptr->image;
      return true;
    } catch (...) {
      return false;
    }
  }

  // classify color from ROI excluding central tag area
  // returns: "RED", "BLUE", or "UNKNOWN"
  std::string classifyCubeColor(const cv::Mat & bgr_roi, int exclude_half)
  {
    if (bgr_roi.empty()) return "UNKNOWN";

    // Create mask that excludes center
    cv::Mat mask(bgr_roi.rows, bgr_roi.cols, CV_8UC1, cv::Scalar(255));
    int cx = bgr_roi.cols / 2;
    int cy = bgr_roi.rows / 2;

    int x0 = clampi(cx - exclude_half, 0, bgr_roi.cols - 1);
    int y0 = clampi(cy - exclude_half, 0, bgr_roi.rows - 1);
    int x1 = clampi(cx + exclude_half, 0, bgr_roi.cols - 1);
    int y1 = clampi(cy + exclude_half, 0, bgr_roi.rows - 1);

    cv::Rect center(x0, y0, std::max(1, x1 - x0), std::max(1, y1 - y0));
    mask(center) = 0; // exclude tag region

    // Convert ROI to HSV
    cv::Mat hsv;
    cv::cvtColor(bgr_roi, hsv, cv::COLOR_BGR2HSV);

    // Mean HSV with mask
    cv::Scalar mean_hsv = cv::mean(hsv, mask);
    double H = mean_hsv[0];   // 0..179
    double S = mean_hsv[1];   // 0..255
    double V = mean_hsv[2];   // 0..255

    // If low saturation, likely gray/white/black
    if (S < 50.0 || V < 40.0) {
      return "UNKNOWN";
    }

    // Robust red/blue thresholds
    bool is_red  = (H < 12.0) || (H > 168.0);
    bool is_blue = (H > 90.0 && H < 140.0);

    if (is_red)  return "RED";
    if (is_blue) return "BLUE";
    return "UNKNOWN";
  }

  bool alreadyPrinted(int id) const
  {
    if (id == red_id_) return printed_red_;
    if (id == blue_id_) return printed_blue_;
    return true;
  }

  void markPrinted(int id)
  {
    if (id == red_id_) printed_red_ = true;
    if (id == blue_id_) printed_blue_ = true;
  }

  void maybeShutdown()
  {
    if (shutdown_after_both_ && printed_red_ && printed_blue_) {
      RCLCPP_INFO(get_logger(), "Both colors printed once. Shutting down.");
      rclcpp::shutdown();
    }
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!caminfo_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /rgb_camera/camera_info...");
      return;
    }

    cv::Mat bgr;
    if (!toBGR(msg, bgr)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to convert image to bgr8.");
      return;
    }

    // grayscale for AprilTag
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    image_u8_t * im = image_u8_create(gray.cols, gray.rows);
    for (int y = 0; y < gray.rows; ++y) {
      std::memcpy(&im->buf[y * im->stride], gray.ptr(y), gray.cols);
    }

    zarray_t * detections = apriltag_detector_detect(td_, im);

    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t * det;
      zarray_get(detections, i, &det);
      int id = det->id;

      if (id != red_id_ && id != blue_id_) continue;
      if (print_once_ && alreadyPrinted(id)) continue;

      int u = static_cast<int>(det->c[0]);
      int v = static_cast<int>(det->c[1]);

      int x0 = clampi(u - roi_half_, 0, bgr.cols - 1);
      int y0 = clampi(v - roi_half_, 0, bgr.rows - 1);
      int x1 = clampi(u + roi_half_, 0, bgr.cols - 1);
      int y1 = clampi(v + roi_half_, 0, bgr.rows - 1);

      int w = std::max(1, x1 - x0);
      int h = std::max(1, y1 - y0);

      cv::Rect roi(x0, y0, w, h);
      cv::Mat roi_bgr = bgr(roi);

      std::string color = classifyCubeColor(roi_bgr, exclude_half_);

      // If you want only confident prints, keep trying until RED/BLUE
      if (require_confident_ && color == "UNKNOWN") {
        continue;
      }

      // Print ONCE per tag id
      RCLCPP_INFO(get_logger(), "Tag %d -> cube color: %s (u=%d v=%d)", id, color.c_str(), u, v);

      if (print_once_) {
        markPrinted(id);
        maybeShutdown();
      }
    }

    apriltag_detections_destroy(detections);
    image_u8_destroy(im);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubeColorDetector>());
  rclcpp::shutdown();
  return 0;
}

