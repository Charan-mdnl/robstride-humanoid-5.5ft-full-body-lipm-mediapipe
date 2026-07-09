"""
full_body_hardware.launch.py
============================
ROS 2 launch file for the Angad full-body humanoid using the fake
ros2_control hardware plugin (full_body_hardware/FakeSystem).

Pipeline:
  1. Process full_body.xacro → robot_state_publisher (TF / RViz)
  2. Start ros2_control_node with the controllers yaml
  3. Spawner nodes activate all controllers after the node starts
  4. (Optional) Start RViz2

Setup:
  source ~/clean_ws/install/setup.bash
  colcon build --packages-select full_body_hardware
  ros2 launch full_body_hardware full_body_hardware.launch.py

Options:
  rviz:=true/false    (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def create_nodes(context: LaunchContext):
    # ── Paths ──────────────────────────────────────────────────────────────────
    pkg_share = get_package_share_directory('full_body_hardware')

    xacro_file       = os.path.join(pkg_share, 'urdf',   'full_body.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'full_body_controllers.yaml')

    # ── Launch argument values ─────────────────────────────────────────────────
    rviz_enabled = LaunchConfiguration('rviz').perform(context)
    use_real_hardware = LaunchConfiguration('use_real_hardware').perform(context)

    # ── Process URDF xacro → robot_description (TF + ros2_control) ───────────
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={'use_real_hardware': use_real_hardware}
    ).toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_xml}

    # ── 1. Robot State Publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': False},
        ],
    )

    # ── 2. ros2_control_node (fake hardware) ───────────────────────────────────
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            controllers_yaml,
            {'use_sim_time': False},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    # ── 3. Controller spawners ─────────────────────────────────────────────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    lower_body_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='lower_body_controller_spawner',
        arguments=['lower_body_controller', '-c', '/controller_manager'],
        output='screen',
    )

    torso_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='torso_controller_spawner',
        arguments=['torso_controller', '-c', '/controller_manager'],
        output='screen',
    )

    left_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='left_arm_controller_spawner',
        arguments=['left_arm_controller', '-c', '/controller_manager'],
        output='screen',
    )

    right_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='right_arm_controller_spawner',
        arguments=['right_arm_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # ── 4. RViz2 (optional) ────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')
    rviz2 = Node(
        condition=IfCondition(rviz_enabled),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    # ── 5. Standing pose publisher — commands initial pose to all controllers ────
    hold_standing_pose = Node(
        package='full_body_hardware',
        executable='foot_force_array_publisher.py',
        name='foot_force_array_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── Event chain: spawn controllers after controller_manager starts ─────────
    # ── Sequential chain: each spawner waits for the previous one to exit ──
    start_jsb = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                LogInfo(msg='[full_body_hardware] controller_manager started. Spawning controllers...'),
                joint_state_broadcaster,
            ],
        )
    )

    start_lower = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[lower_body_controller],
        )
    )

    start_torso = RegisterEventHandler(
        OnProcessExit(
            target_action=lower_body_controller,
            on_exit=[torso_controller],
        )
    )

    start_left = RegisterEventHandler(
        OnProcessExit(
            target_action=torso_controller,
            on_exit=[left_arm_controller],
        )
    )

    start_right = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_controller,
            on_exit=[right_arm_controller],
        )
    )

    return [
        robot_state_publisher,
        controller_manager_node,
        start_jsb,
        start_lower,
        start_torso,
        start_left,
        start_right,
        rviz2,  # starts alongside RSP/controller_manager — only needs /robot_description + /tf
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 for visualisation',
        ),
        DeclareLaunchArgument(
            'use_real_hardware',
            default_value='true',
            description='Use real RobStrideSystem hardware instead of a full fake system',
        ),
        OpaqueFunction(function=create_nodes),
    ])
