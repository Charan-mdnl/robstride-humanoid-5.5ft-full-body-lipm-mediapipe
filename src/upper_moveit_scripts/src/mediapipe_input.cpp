#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;

static const std::string LEFT_GROUP = "left";
static const std::string RIGHT_GROUP = "right";

class MediaPipeMoveItNode : public rclcpp::Node
{
public:
  MediaPipeMoveItNode()
  : Node("mediapipe_node"),
    busy_(false),
    left_seeded_(false),
    right_seeded_(false)
  {
    this->declare_parameter("velocity_scaling", 0.5);
    this->declare_parameter("acceleration_scaling", 0.5);

    velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();
  }

  void init()
  {
    // IMPORTANT: shared_from_this() must not be called from the constructor.
    left_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), LEFT_GROUP);
    right_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), RIGHT_GROUP);

    left_arm_->setMaxVelocityScalingFactor(velocity_scaling_);
    left_arm_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    right_arm_->setMaxVelocityScalingFactor(velocity_scaling_);
    right_arm_->setMaxAccelerationScalingFactor(acceleration_scaling_);

    RCLCPP_INFO(this->get_logger(), "Left planning frame  : %s",
                left_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "Right planning frame : %s",
                right_arm_->getPlanningFrame().c_str());

    left_joint_names_ = left_arm_->getJointNames();
    right_joint_names_ = right_arm_->getJointNames();

    RCLCPP_INFO(this->get_logger(), "=== left joint names ===");
    for (const auto &name : left_joint_names_) {
      RCLCPP_INFO(this->get_logger(), "  %s", name.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "=== right joint names ===");
    for (const auto &name : right_joint_names_) {
      RCLCPP_INFO(this->get_logger(), "  %s", name.c_str());
    }

    left_target_cache_.assign(left_joint_names_.size(), 0.0);
    right_target_cache_.assign(right_joint_names_.size(), 0.0);

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/arm_poses",
      10,
      std::bind(&MediaPipeMoveItNode::arm_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /arm_poses");
    RCLCPP_INFO(this->get_logger(), "This node does not fetch /joint_states in its own logic.");
  }

private:
  bool set_joint_by_name(
    const std::vector<std::string> &names,
    const std::string &joint_name,
    double value,
    std::vector<double> &target)
  {
    if (target.size() != names.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Target size (%zu) does not match joint name count (%zu)",
                   target.size(), names.size());
      return false;
    }

    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i] == joint_name) {
        target[i] = value;
        return true;
      }
    }
    return false;
  }

  bool plan_and_execute(
    moveit::planning_interface::MoveGroupInterface &mg,
    const std::vector<double> &joint_target,
    const std::string &label)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Do not call setStartStateToCurrentState() here, since that would require
    // current robot state from /joint_states.
    mg.setJointValueTarget(joint_target);

    bool success =
      (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Planning failed", label.c_str());
      return false;
    }

    auto exec_result = mg.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Execution failed", label.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "[%s] Execution complete", label.c_str());
    return true;
  }

  void arm_pose_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (busy_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MoveIt busy, skipping incoming /arm_poses message");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = true;

    try {
      const auto data = json::parse(msg->data);

      const double left_shoulder  = data.at("left").at("shoulder").get<double>();
      const double left_elbow     = data.at("left").at("elbow").get<double>();
      const double right_shoulder = data.at("right").at("shoulder").get<double>();
      const double right_elbow    = data.at("right").at("elbow").get<double>();

      RCLCPP_INFO(
        this->get_logger(),
        "Received /arm_poses | L[shoulder=%.3f elbow=%.3f] R[shoulder=%.3f elbow=%.3f]",
        left_shoulder, left_elbow, right_shoulder, right_elbow);

      if (!left_seeded_) {
        left_target_cache_.assign(left_joint_names_.size(), 0.0);
        left_seeded_ = true;
      }
      if (!right_seeded_) {
        right_target_cache_.assign(right_joint_names_.size(), 0.0);
        right_seeded_ = true;
      }

      std::vector<double> left_target = left_target_cache_;
      std::vector<double> right_target = right_target_cache_;

      const bool left_ok1 = set_joint_by_name(left_joint_names_, "shoulder_pitch_l", left_shoulder, left_target);
      const bool left_ok2 = set_joint_by_name(left_joint_names_, "elbow_pitch_l", left_elbow, left_target);
      const bool right_ok1 = set_joint_by_name(right_joint_names_, "shoulder_pitch_r", right_shoulder, right_target);
      const bool right_ok2 = set_joint_by_name(right_joint_names_, "elbow_pitch_r", right_elbow, right_target);

      if (!(left_ok1 && left_ok2 && right_ok1 && right_ok2)) {
        RCLCPP_ERROR(this->get_logger(),
          "Could not find one or more expected joints. Check MoveIt joint names.");
        busy_ = false;
        return;
      }

      const bool left_success = plan_and_execute(*left_arm_, left_target, "LEFT");
      const bool right_success = plan_and_execute(*right_arm_, right_target, "RIGHT");

      if (left_success) {
        left_target_cache_ = left_target;
      }
      if (right_success) {
        right_target_cache_ = right_target;
      }
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse /arm_poses JSON: %s", e.what());
    }

    busy_ = false;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_;

  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  std::vector<double> left_target_cache_;
  std::vector<double> right_target_cache_;

  double velocity_scaling_;
  double acceleration_scaling_;

  std::mutex mutex_;
  std::atomic<bool> busy_;
  bool left_seeded_;
  bool right_seeded_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MediaPipeMoveItNode>();
  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
