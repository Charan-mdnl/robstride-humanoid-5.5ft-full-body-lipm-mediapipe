cmake_minimum_required(VERSION 3.8)
project(upper_moveit_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
add_executable(mediapipe_exe src/mediapipe_node.cpp)
target_include_directories(mediapipe_exe PUBLIC include)
ament_target_dependencies(mediapipe_exe ament_cmake rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_msgs)
    add_executable(mediapipe_inp_exe src/mediapipe_input.cpp)
target_include_directories(mediapipe_inp_exe PUBLIC include)
ament_target_dependencies(mediapipe_inp_exe ament_cmake rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_msgs
    nlohmann_json)

add_executable(set_goal_pose_exe src/set_goal_pose.cpp)
target_include_directories(set_goal_pose_exe PUBLIC include)
ament_target_dependencies(set_goal_pose_exe ament_cmake rclcpp
    geometry_msgs
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS 
	mediapipe_exe
	mediapipe_inp_exe
	set_goal_pose_exe
    DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
