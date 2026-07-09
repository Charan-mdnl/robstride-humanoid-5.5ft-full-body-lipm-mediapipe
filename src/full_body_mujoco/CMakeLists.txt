cmake_minimum_required(VERSION 3.8)
project(full_body_mujoco)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Only nr_ik_test is built here (no pinocchio / mujoco / glfw dependency).
add_executable(nr_ik_test scripts/nr_ik_test.cpp)
ament_target_dependencies(nr_ik_test
  rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs urdf trajectory_msgs visualization_msgs)
target_link_libraries(nr_ik_test Eigen3::Eigen)
install(TARGETS nr_ik_test DESTINATION lib/${PROJECT_NAME})

ament_package()
