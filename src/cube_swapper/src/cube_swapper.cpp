#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"

using std::placeholders::_1;

class CubeSwapper : public rclcpp::Node
{
public:
  CubeSwapper() : Node("cube_swapper")
  {
    // ---- Params ----
    declare_parameter<std::string>("arm_group", "ir_arm");
    declare_parameter<std::string>("gripper_group", "ir_gripper");

    declare_parameter<double>("approach_height", 0.15);  // above tag
    declare_parameter<double>("grasp_height", 0.07);     // near top face
    declare_parameter<double>("lift_height", 0.20);      // lift after grasp

    declare_parameter<bool>("use_gripper", true);

    arm_group_ = get_parameter("arm_group").as_string();
    gripper_group_ = get_parameter("gripper_group").as_string();

    approach_h_ = get_parameter("approach_height").as_double();
    grasp_h_ = get_parameter("grasp_height").as_double();
    lift_h_ = get_parameter("lift_height").as_double();

    use_gripper_ = get_parameter("use_gripper").as_bool();

    // Sub to tag poses (published by your tag_detection node)
    sub_red_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tag_detection/red_tag_world", 10, std::bind(&CubeSwapper::redCb, this, _1));
    sub_blue_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tag_detection/blue_tag_world", 10, std::bind(&CubeSwapper::blueCb, this, _1));

    // Timer tick
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&CubeSwapper::tick, this));

    RCLCPP_INFO(get_logger(),
                "cube_swapper started. Waiting for red+blue tag poses... arm_group=%s gripper_group=%s",
                arm_group_.c_str(), gripper_group_.c_str());
  }

private:
  // Params
  std::string arm_group_;
  std::string gripper_group_;
  double approach_h_{0.15};
  double grasp_h_{0.07};
  double lift_h_{0.20};
  bool use_gripper_{true};

  // Tag poses
  std::mutex mtx_;
  std::optional<geometry_msgs::msg::PoseStamped> red_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> blue_pose_;
  bool executed_{false};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_red_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_blue_;
  rclcpp::TimerBase::SharedPtr timer_;

  void redCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    red_pose_ = *msg;
  }

  void blueCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    blue_pose_ = *msg;
  }

  geometry_msgs::msg::Pose makePoseAbove(const geometry_msgs::msg::PoseStamped & tag, double dz) const
  {
    geometry_msgs::msg::Pose p = tag.pose;
    p.position.z += dz;

    // Placeholder orientation (often you will tune this)
    // If your arm refuses to plan, we’ll set this from the current EE orientation instead.
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 1.0;

    return p;
  }

  bool goNamed(moveit::planning_interface::MoveGroupInterface & mg, const std::string & name)
  {
    mg.setNamedTarget(name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed to named target '%s' (group=%s)", name.c_str(), mg.getName().c_str());
      return false;
    }
    if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed to named target '%s' (group=%s)", name.c_str(), mg.getName().c_str());
      return false;
    }
    return true;
  }

  bool goPose(moveit::planning_interface::MoveGroupInterface & mg,
              const geometry_msgs::msg::Pose & pose,
              const std::string & frame_id)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.header.stamp = now();
    ps.pose = pose;

    mg.setPoseTarget(ps);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed to pose target (group=%s, frame=%s)", mg.getName().c_str(), frame_id.c_str());
      return false;
    }

    if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed to pose target (group=%s)", mg.getName().c_str());
      return false;
    }

    return true;
  }

  void tick()
  {
    if (executed_) return;

    std::optional<geometry_msgs::msg::PoseStamped> red, blue;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      red = red_pose_;
      blue = blue_pose_;
    }

    if (!red || !blue) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for both tag poses...");
      return;
    }

    executed_ = true;

    // MoveIt interfaces (must be created after node is spinning)
    moveit::planning_interface::MoveGroupInterface arm(shared_from_this(), arm_group_);
    moveit::planning_interface::MoveGroupInterface gripper(shared_from_this(), gripper_group_);

    arm.setPlanningTime(5.0);
    arm.setMaxVelocityScalingFactor(0.3);
    arm.setMaxAccelerationScalingFactor(0.3);

    gripper.setPlanningTime(2.0);

    RCLCPP_INFO(get_logger(), "Got tags. Starting pick(red)->place(blue).");

    // HOME
    if (!goNamed(arm, "home")) return;

    // Open gripper before approach
    if (use_gripper_) {
      if (!goNamed(gripper, "open")) return;
    }

    // ---- PICK RED ----
    const std::string frame_red = red->header.frame_id;
    RCLCPP_INFO(get_logger(), "Picking RED tag pose in frame '%s'", frame_red.c_str());

    auto pre_grasp = makePoseAbove(*red, approach_h_);
    if (!goPose(arm, pre_grasp, frame_red)) return;

    auto grasp = makePoseAbove(*red, grasp_h_);
    if (!goPose(arm, grasp, frame_red)) return;

    if (use_gripper_) {
      if (!goNamed(gripper, "close")) return;
    }

    auto lift = makePoseAbove(*red, lift_h_);
    if (!goPose(arm, lift, frame_red)) return;

    // ---- PLACE BLUE ----
    const std::string frame_blue = blue->header.frame_id;
    RCLCPP_INFO(get_logger(), "Placing at BLUE tag pose in frame '%s'", frame_blue.c_str());

    auto pre_place = makePoseAbove(*blue, approach_h_);
    if (!goPose(arm, pre_place, frame_blue)) return;

    auto place = makePoseAbove(*blue, grasp_h_);
    if (!goPose(arm, place, frame_blue)) return;

    if (use_gripper_) {
      if (!goNamed(gripper, "open")) return;
    }

    // Retreat up
    if (!goPose(arm, pre_place, frame_blue)) return;

    // HOME again
    (void)goNamed(arm, "home");

    RCLCPP_INFO(get_logger(), "Done pick(red)->place(blue). For full swap, run the inverse too.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CubeSwapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

