cmake_minimum_required(VERSION 3.8)
project(full_body_hardware_leg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# ── Fake hardware plugin shared library ───────────────────────────────────────
add_library(fake_system SHARED
  src/fake_system.cpp
)
target_include_directories(fake_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fake_system
  hardware_interface
  pluginlib
  rclcpp
)
# Each plugin target exports its own description file so they are independently
# discoverable by pluginlib even when only one of the two libraries is built.
pluginlib_export_plugin_description_file(hardware_interface full_body_hardware_plugins.xml)

# ── RobStride real hardware plugin ────────────────────────────────────────────
# NOTE: robstride_system.cpp will not compile until you complete the TODOs:
#   1. Copy can_interfaces.h + protocol_completed.h into include/full_body_hardware/
#   2. Copy can_interfaces.cpp into src/
#   3. Uncomment the #include lines in robstride_system.hpp and robstride_system.cpp
#   4. Uncomment the CanInterface member and all CAN call sites
# Until then, keep EXCLUDE_FROM_ALL so the default build stays green.
add_library(robstride_system SHARED
  src/robstride_system.cpp
  src/can_interfaces.cpp          # copy from RobStride_Control/cpp/src/
)
target_include_directories(robstride_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robstride_system
  hardware_interface
  pluginlib
  rclcpp
)
pluginlib_export_plugin_description_file(hardware_interface robstride_system_plugins.xml)

install(TARGETS fake_system robstride_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# ── Install ROS resources ─────────────────────────────────────────────────────
install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  full_body_hardware_plugins.xml
  robstride_system_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootSensor.msg"
  "msg/FootSensorArray.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
