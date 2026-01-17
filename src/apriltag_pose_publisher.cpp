#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// System apriltag (libapriltag-dev on Ubuntu)
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/image_u8.h>

class AprilTagPosePublisher : public rclcpp::Node
{
public:
  AprilTagPosePublisher()
  : Node("apriltag_pose_publisher")
  {
    // Parameters
    declare_parameter<std::string>("image_topic", "/rgb_camera/image");
    declare_parameter<std::string>("camera_info_topic", "/rgb_camera/camera_info");
    declare_parameter<std::string>("target_frame", "world");  // world/map/odom
    declare_parameter<std::string>("tag_family", "36h11");
    declare_parameter<double>("tag_size", 0.05);              // meters
    declare_parameter<int>("red_id", 1);
    declare_parameter<int>("blue_id", 10);

    image_topic_   = get_parameter("image_topic").as_string();
    caminfo_topic_ = get_parameter("camera_info_topic").as_string();
    target_frame_  = get_parameter("target_frame").as_string();
    tag_family_    = get_parameter("tag_family").as_string();
    tag_size_      = get_parameter("tag_size").as_double();
    red_id_        = get_parameter("red_id").as_int();
    blue_id_       = get_parameter("blue_id").as_int();

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers
    pub_red_  = create_publisher<geometry_msgs::msg::PoseStamped>("/tag_detection/red_tag_world", 10);
    pub_blue_ = create_publisher<geometry_msgs::msg::PoseStamped>("/tag_detection/blue_tag_world", 10);

    // Subscribers
    sub_caminfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      caminfo_topic_, 10,
      std::bind(&AprilTagPosePublisher::camInfoCb, this, std::placeholders::_1));

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&AprilTagPosePublisher::imageCb, this, std::placeholders::_1));

    // AprilTag detector
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    td_->quad_decimate = 1.0;
    td_->quad_sigma = 0.0;
    td_->nthreads = 2;
    td_->debug = 0;
    td_->refine_edges = 1;

    RCLCPP_INFO(get_logger(),
      "apriltag_pose_publisher started.\n  image: %s\n  camera_info: %s\n  target_frame: %s\n  tag_size: %.3f",
      image_topic_.c_str(), caminfo_topic_.c_str(), target_frame_.c_str(), tag_size_);
  }

  ~AprilTagPosePublisher() override
  {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

private:
  // params
  std::string image_topic_;
  std::string caminfo_topic_;
  std::string target_frame_;
  std::string tag_family_;
  double tag_size_;
  int red_id_;
  int blue_id_;

  // intrinsics
  bool intrinsics_ready_ = false;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  std::string camera_frame_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_red_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_blue_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // AprilTag
  apriltag_family_t * tf_;
  apriltag_detector_t * td_;

  void camInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    camera_frame_ = msg->header.frame_id;
    intrinsics_ready_ = (fx_ > 0.0 && fy_ > 0.0);

    if (!intrinsics_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CameraInfo intrinsics not valid yet.");
    }
  }

  static geometry_msgs::msg::PoseStamped poseFromAprilTagPose(
    const apriltag_pose_t & pose_cam, const std::string & frame_id, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;

    out.pose.position.x = pose_cam.t->data[0];
    out.pose.position.y = pose_cam.t->data[1];
    out.pose.position.z = pose_cam.t->data[2];

    tf2::Matrix3x3 R(
      pose_cam.R->data[0], pose_cam.R->data[1], pose_cam.R->data[2],
      pose_cam.R->data[3], pose_cam.R->data[4], pose_cam.R->data[5],
      pose_cam.R->data[6], pose_cam.R->data[7], pose_cam.R->data[8]
    );
    tf2::Quaternion q;
    R.getRotation(q);
    out.pose.orientation = tf2::toMsg(q);
    return out;
  }

  void publishIdPoseWorld(int id, apriltag_detection_t * det)
  {
    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = tag_size_;
    info.fx = fx_;
    info.fy = fy_;
    info.cx = cx_;
    info.cy = cy_;

    apriltag_pose_t pose_cam;
    estimate_tag_pose(&info, &pose_cam);

    // pose in camera frame
    auto cam_pose = poseFromAprilTagPose(pose_cam, camera_frame_, now());

    // transform to world/map/odom
    geometry_msgs::msg::PoseStamped world_pose;
    try {
      world_pose = tf_buffer_->transform(cam_pose, target_frame_, tf2::durationFromSec(0.1));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF failed %s -> %s : %s", camera_frame_.c_str(), target_frame_.c_str(), e.what());
      return;
    }

    if (id == red_id_) {
      pub_red_->publish(world_pose);
    } else if (id == blue_id_) {
      pub_blue_->publish(world_pose);
    }
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!intrinsics_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /rgb_camera/camera_info...");
      return;
    }
    if (camera_frame_.empty()) {
      camera_frame_ = msg->header.frame_id;
    }

    // Convert to grayscale
    cv::Mat gray;
    try {
      auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      if (cv_ptr->image.channels() == 3) {
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
      } else {
        gray = cv_ptr->image;
      }
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge conversion failed.");
      return;
    }

    // Create apriltag image (avoid deleted ctor issue)
    image_u8_t * im = image_u8_create(gray.cols, gray.rows);
    for (int y = 0; y < gray.rows; ++y) {
      std::memcpy(&im->buf[y * im->stride], gray.ptr(y), gray.cols);
    }

    zarray_t * detections = apriltag_detector_detect(td_, im);

    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t * det;
      zarray_get(detections, i, &det);

      const int id = det->id;
      if (id == red_id_ || id == blue_id_) {
        publishIdPoseWorld(id, det);
      }
    }

    apriltag_detections_destroy(detections);
    image_u8_destroy(im);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagPosePublisher>());
  rclcpp::shutdown();
  return 0;
}

