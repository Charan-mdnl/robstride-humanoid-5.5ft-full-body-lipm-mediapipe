#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

class GestureHandlerNode : public rclcpp::Node {
public:
  GestureHandlerNode(const rclcpp::NodeOptions &options)
      : Node("gesture_handler", options) {
    
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "/gesture_cmd", 10,
        std::bind(&GestureHandlerNode::gestureCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gesture Handler initialized.");
  }

  void startMoveItThread() {
    worker_thread_ = std::thread(&GestureHandlerNode::workerLoop, this);
  }

  ~GestureHandlerNode() {
    running_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void gestureCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_gesture_ = msg->data;
  }

  void workerLoop() {
    // Instantiate inside the thread when executor is spinning
    moveit::planning_interface::MoveGroupInterface right_arm(shared_from_this(), "right_arm");
    moveit::planning_interface::MoveGroupInterface left_arm(shared_from_this(), "left_arm");

    right_arm.setPlanningTime(5.0);
    left_arm.setPlanningTime(5.0);
    right_arm.setMaxVelocityScalingFactor(0.5);
    left_arm.setMaxVelocityScalingFactor(0.5);
    right_arm.setMaxAccelerationScalingFactor(0.5);
    left_arm.setMaxAccelerationScalingFactor(0.5);

    std::string current_gesture = "DEFAULT";

    while (running_ && rclcpp::ok()) {
      std::string target_gesture;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        target_gesture = latest_gesture_;
      }

      if (target_gesture != current_gesture) {
        RCLCPP_INFO(this->get_logger(), "Processing gesture: %s", target_gesture.c_str());

        std::string right_goal = "standing";
        std::string left_goal = "standing";
        bool valid_gesture = true;

        if (target_gesture == "HI_BOTH") {
          right_goal = "right_arm_bye";
          left_goal = "left_arm_bye";
        } else if (target_gesture == "HI_L") {
          left_goal = "left_arm_bye";
        } else if (target_gesture == "HI_R") {
          right_goal = "right_arm_bye";
        } else if (target_gesture == "NAMASTE") {
            right_goal = "right_namaste";
            left_goal = "left_namaste";
        } else if (target_gesture == "HANDSHAKE_BOTH") {
          right_goal = "right_arm_handshake";
          left_goal = "left_arm_handshake";
        } else if (target_gesture == "HANDSHAKE_L") {
          left_goal = "left_arm_handshake";
        } else if (target_gesture == "HANDSHAKE_R") {
          right_goal = "right_arm_handshake";
        } else if (target_gesture == "DEFAULT") {
          // already set to standing
        } else {
          valid_gesture = false;
        }

        if (valid_gesture) {
          auto planAndExecute = [this](moveit::planning_interface::MoveGroupInterface& group, const std::string& target_state) {
            group.setNamedTarget(target_state);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
              group.execute(plan);
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to plan for %s", target_state.c_str());
            }
          };

          RCLCPP_INFO(this->get_logger(), "Executing gesture: %s", target_gesture.c_str());
          planAndExecute(right_arm, right_goal);
          planAndExecute(left_arm, left_goal);
          
          RCLCPP_INFO(this->get_logger(), "Finished executing gesture: %s", target_gesture.c_str());
        }

        current_gesture = target_gesture;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::thread worker_thread_;
  std::mutex mutex_;
  std::string latest_gesture_{"DEFAULT"};
  std::atomic<bool> running_{true};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GestureHandlerNode>(
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // The node must spin in an executor so MoveGroupInterface can receive action feedback.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  node->startMoveItThread();
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
