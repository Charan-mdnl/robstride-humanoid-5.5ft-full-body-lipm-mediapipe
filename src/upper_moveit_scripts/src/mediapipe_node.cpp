/**
 * @file mediapipe_node.cpp
 * @brief MoveIt2 C++ node controlling both "left_arm" and "right" planning
 * groups.
 *
 * Sets up MoveGroupInterface for each arm, plans to named states and
 * custom joint-space targets, then executes the trajectories.
 */

#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

static const std::string LEFT_GROUP = "left";
static const std::string RIGHT_GROUP = "right";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "mediapipe_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Spin in a background thread so MoveGroupInterface action calls work
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto logger = node->get_logger();

  // ═══════════════════════════════════════════════════════════════════════════
  //  Create MoveGroupInterfaces for both arms
  // ═══════════════════════════════════════════════════════════════════════════
  moveit::planning_interface::MoveGroupInterface left_arm(node, LEFT_GROUP);
  moveit::planning_interface::MoveGroupInterface right_arm(node, RIGHT_GROUP);

  RCLCPP_INFO(logger, "Left  arm planning frame : %s",
              left_arm.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Right arm planning frame : %s",
              right_arm.getPlanningFrame().c_str());

  // ── Print current joint values ─────────────────────────────────────────────
  auto print_joints =
      [&logger](const std::string &label,
                moveit::planning_interface::MoveGroupInterface &mg) {
        auto names = mg.getJointNames();
        auto values = mg.getCurrentJointValues();
        RCLCPP_INFO(logger, "=== %s current joints ===", label.c_str());
        for (size_t i = 0; i < names.size(); ++i) {
          RCLCPP_INFO(logger, "  %s : %.4f", names[i].c_str(), values[i]);
        }
      };

  print_joints("left_arm", left_arm);
  print_joints("right", right_arm);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;

  // ═══════════════════════════════════════════════════════════════════════════
  //  LEFT ARM — move to Home_left
  // ═══════════════════════════════════════════════════════════════════════════
  RCLCPP_INFO(logger, "--- [LEFT] Moving to named target: Home_left ---");
  left_arm.setNamedTarget("Home_left");
  success = (left_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, "[LEFT] Plan SUCCEEDED — executing");
    left_arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "[LEFT] Plan to Home_left FAILED");
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  RIGHT ARM — move to HomeRight
  // ═══════════════════════════════════════════════════════════════════════════
  RCLCPP_INFO(logger, "--- [RIGHT] Moving to named target: HomeRight ---");
  right_arm.setNamedTarget("HomeRight");
  success = (right_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, "[RIGHT] Plan SUCCEEDED — executing");
    right_arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "[RIGHT] Plan to HomeRight FAILED");
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  LEFT ARM — custom joint-space goal
  //  Joint order: shoulder_pitch_l, shoulder_roll_l, elbow_yaw_l, elbow_pitch_l
  // ═══════════════════════════════════════════════════════════════════════════
  RCLCPP_INFO(logger, "--- [LEFT] Moving to custom joint target ---");
  left_arm.setJointValueTarget(std::vector<double>{0.3, 0.5, 0.0, -0.5});
  left_arm.setMaxVelocityScalingFactor(0.5);
  left_arm.setMaxAccelerationScalingFactor(0.5);

  success = (left_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, "[LEFT] Custom joint plan SUCCEEDED — executing");
    left_arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "[LEFT] Custom joint plan FAILED");
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  RIGHT ARM — custom joint-space goal
  //  Joint order: shoulder_pitch_r, shoulder_roll_r, elbow_yaw_r, elbow_pitch_r
  // ═══════════════════════════════════════════════════════════════════════════
  RCLCPP_INFO(logger, "--- [RIGHT] Moving to custom joint target ---");
  right_arm.setJointValueTarget(std::vector<double>{0.3, -0.5, 0.0, -0.5});
  right_arm.setMaxVelocityScalingFactor(0.5);
  right_arm.setMaxAccelerationScalingFactor(0.5);

  success = (right_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, "[RIGHT] Custom joint plan SUCCEEDED — executing");
    right_arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "[RIGHT] Custom joint plan FAILED");
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Return both arms to home poses
  // ═══════════════════════════════════════════════════════════════════════════
  RCLCPP_INFO(logger, "--- Returning both arms to Home ---");

  left_arm.setNamedTarget("Home_left");
  success = (left_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, "[LEFT] Return plan SUCCEEDED — executing");
    left_arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "[LEFT] Return plan FAILED");
  }

  right_arm.setNamedTarget("HomeRight");
  success = (right_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, "[RIGHT] Return plan SUCCEEDED — executing");
    right_arm.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "[RIGHT] Return plan FAILED");
  }

  RCLCPP_INFO(logger, "Mediapipe node demo complete.");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
