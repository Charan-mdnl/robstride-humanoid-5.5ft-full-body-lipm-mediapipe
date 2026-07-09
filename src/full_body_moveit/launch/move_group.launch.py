"""
move_group.launch.py
====================
Launches the MoveIt 2 move_group node for the Angad full-body humanoid.

Prerequisites (must already be running):
  ros2 launch full_body_mujoco full_body_mujoco.launch.py

This node connects to the running robot_state_publisher and the
ros2_control JointTrajectoryControllers spawned by full_body_mujoco.

Usage:
  source ~/clean_ws/install/setup.bash
  ros2 launch full_body_moveit move_group.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_mujoco = get_package_share_directory('full_body_mujoco')

    moveit_config = (
        MoveItConfigsBuilder('angad_full_body', package_name='full_body_moveit')
        .robot_description(
            file_path=os.path.join(pkg_mujoco, 'urdf', 'full_body.xacro')
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit'),
                'config', 'full_body.srdf'
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit'),
                'config', 'kinematics.yaml'
            )
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit'),
                'config', 'joint_limits.yaml'
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit'),
                'config', 'moveit_controllers.yaml'
            )
        )
        .planning_scene_monitor(
            publish_robot_description=False,      # RSP already running
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=['ompl'],
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'trajectory_execution.allowed_execution_duration_scaling': 2.0},
            {'trajectory_execution.allowed_goal_duration_margin': 0.5},
            {'trajectory_execution.execution_duration_monitoring': False},
            {'move_group.capabilities': ''},
        ],
    )

    return LaunchDescription([move_group_node])
