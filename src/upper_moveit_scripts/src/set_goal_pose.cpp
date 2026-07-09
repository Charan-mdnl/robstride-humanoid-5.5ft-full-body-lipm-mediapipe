#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "set_goal_pose_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto logger = node->get_logger();

  const auto get_string_param = [&](const std::string & name, const std::string & fallback) {
    std::string value = fallback;
    (void)node->get_parameter(name, value);
    return value;
  };

  const auto get_double_param = [&](const std::string & name, double fallback) {
    double value = fallback;
    (void)node->get_parameter(name, value);
    return value;
  };

  const auto get_bool_param = [&](const std::string & name, bool fallback) {
    bool value = fallback;
    (void)node->get_parameter(name, value);
    return value;
  };

  const std::string group = get_string_param("group", "right_arm");
  std::string ee_link = get_string_param("ee_link", "");
  const std::string frame_id = get_string_param("frame_id", "");
  const double velocity_scaling = get_double_param("velocity_scaling", 0.2);
  const double acceleration_scaling = get_double_param("acceleration_scaling", 0.2);
  const bool execute_motion = get_bool_param("execute", true);
  const double x_goal = get_double_param("x", std::numeric_limits<double>::quiet_NaN());
  const double y_goal = get_double_param("y", std::numeric_limits<double>::quiet_NaN());
  const double z_goal = get_double_param("z", std::numeric_limits<double>::quiet_NaN());
  const bool has_absolute_position_request =
    !std::isnan(x_goal) || !std::isnan(y_goal) || !std::isnan(z_goal);
  const bool requested_absolute_mode = get_bool_param("use_absolute_position", false);
  const bool use_absolute_position =
    requested_absolute_mode || has_absolute_position_request;
  const bool use_orientation = get_bool_param("use_orientation", false);
  const double dx = get_double_param("dx", 0.0);
  const double dy = get_double_param("dy", 0.0);
  const double dz = get_double_param("dz", 0.0);

  if (ee_link.empty()) {
    if (group == "right_arm" || group == "right") {
      ee_link = "forearm_r";
    } else if (group == "left_arm" || group == "left") {
      ee_link = "forearm_l";
    }
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, group);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);

  const std::string planning_frame = move_group.getPlanningFrame();
  const std::string resolved_frame_id =
    (frame_id.empty() || frame_id == "world") ? planning_frame : frame_id;

  move_group.setPoseReferenceFrame(resolved_frame_id);
  move_group.setStartStateToCurrentState();
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.05);

  const auto current_pose = move_group.getCurrentPose(ee_link);
  auto target_pose = current_pose.pose;

  const auto maybe_set = [&](const std::string & name, double & field) {
    const double value = get_double_param(name, std::numeric_limits<double>::quiet_NaN());
    if (!std::isnan(value)) {
      field = value;
    }
  };

  if (use_absolute_position) {
    if (!std::isnan(x_goal)) target_pose.position.x = x_goal;
    if (!std::isnan(y_goal)) target_pose.position.y = y_goal;
    if (!std::isnan(z_goal)) target_pose.position.z = z_goal;
    RCLCPP_INFO(logger, "Using absolute position goal.");
  } else {
    target_pose.position.x += dx;
    target_pose.position.y += dy;
    target_pose.position.z += dz;
    RCLCPP_INFO(logger, "Using relative position offset.");
  }

  maybe_set("qx", target_pose.orientation.x);
  maybe_set("qy", target_pose.orientation.y);
  maybe_set("qz", target_pose.orientation.z);
  maybe_set("qw", target_pose.orientation.w);

  const bool has_orientation_goal =
    use_orientation ||
    !std::isnan(get_double_param("qx", std::numeric_limits<double>::quiet_NaN())) ||
    !std::isnan(get_double_param("qy", std::numeric_limits<double>::quiet_NaN())) ||
    !std::isnan(get_double_param("qz", std::numeric_limits<double>::quiet_NaN())) ||
    !std::isnan(get_double_param("qw", std::numeric_limits<double>::quiet_NaN()));

  RCLCPP_INFO(logger, "Planning group   : %s", group.c_str());
  RCLCPP_INFO(logger, "Planning frame   : %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "End-effector link: %s", ee_link.c_str());
  RCLCPP_INFO(logger, "Reference frame  : %s", resolved_frame_id.c_str());
  RCLCPP_INFO(logger, "Current pose xyz : [%.3f, %.3f, %.3f]",
    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  RCLCPP_INFO(logger, "Target pose xyz  : [%.3f, %.3f, %.3f]",
    target_pose.position.x, target_pose.position.y, target_pose.position.z);

  bool target_ok = false;
  if (has_orientation_goal) {
    target_ok = move_group.setApproximateJointValueTarget(target_pose, ee_link);
    RCLCPP_INFO(logger, "Using approximate IK for full pose goal.");
  } else {
    target_ok = move_group.setApproximateJointValueTarget(target_pose, ee_link);
    RCLCPP_INFO(logger, "Using approximate IK for position goal.");
  }

  if (!target_ok) {
    RCLCPP_ERROR(logger, "Could not compute a valid IK target for the requested pose.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool success =
    (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success) {
    RCLCPP_ERROR(logger, "Pose-goal planning failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(logger, "Pose-goal planning succeeded.");

  if (execute_motion) {
    const auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Trajectory execution failed.");
      rclcpp::shutdown();
      spinner.join();
      return 2;
    }
    RCLCPP_INFO(logger, "Trajectory execution complete.");
  } else {
    RCLCPP_INFO(logger, "Execution disabled; plan only.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
