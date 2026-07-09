"""
move_group.launch.py
====================
Launches the MoveIt 2 move_group node for the Angad full-body humanoid.

Prerequisites (must already be running):
  ros2 launch full_body_hardware full_body_hardware.launch.py

This node connects to the running robot_state_publisher and the
ros2_control JointTrajectoryControllers spawned by full_body_hardware.

Usage:
  source ~/clean_ws/install/setup.bash
  ros2 launch full_body_moveit_hardware move_group.launch.py
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _to_param_file(params_dict):
    """Write a parameters dict to a temp YAML file.

    This avoids the !!python/tuple serialisation bug that occurs when
    ROS 2 launch normalises inline dicts containing lists.
    """
    wrapped = {'/**': {'ros__parameters': params_dict}}
    fd, path = tempfile.mkstemp(prefix='launch_moveit_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.dump(wrapped, f, default_flow_style=False)
    return path


def _create_move_group(context: LaunchContext):
    pkg_hardware = get_package_share_directory('full_body_hardware')
    use_real_hardware = LaunchConfiguration('use_real_hardware').perform(context)

    moveit_config = (
        MoveItConfigsBuilder('angad_full_body', package_name='full_body_moveit_hardware')
        .robot_description(
            file_path=os.path.join(pkg_hardware, 'urdf', 'full_body.xacro'),
            mappings={'use_real_hardware': use_real_hardware},
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit_hardware'),
                'config', 'full_body.srdf'
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit_hardware'),
                'config', 'kinematics.yaml'
            )
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit_hardware'),
                'config', 'joint_limits.yaml'
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory('full_body_moveit_hardware'),
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

    moveit_params_file = _to_param_file(moveit_config.to_dict())

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_params_file,
            {'use_sim_time': False},
            {'trajectory_execution.allowed_execution_duration_scaling': 2.0},
            {'trajectory_execution.allowed_goal_duration_margin': 0.5},
            {'trajectory_execution.execution_duration_monitoring': False},
            {'start_state_max_bounds_error': 0.05},
        ],
    )

    return [move_group_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_real_hardware',
            default_value='false',
            description='Use real RobStrideSystem hardware instead of a full fake system',
        ),
        OpaqueFunction(function=_create_move_group),
    ])

