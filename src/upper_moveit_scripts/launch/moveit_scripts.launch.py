import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("upper_body", package_name="angad_moveit_upper_arm").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="mediapipe_node",
        package="upper_moveit_scripts",
        executable="mediapipe_exe",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )