"""
mujoco.launch.py
================
ROS 2 launch file for the upper_body robot in MuJoCo using the XP_robot
MuJoCo XML model (upper_body_xp.xml).

Key design points:
- NO xacro2mjcf conversion — uses the pre-crafted XP_robot MuJoCo XML directly
- Joint names are URDF names (shoulder_pitch_r etc.) throughout the entire stack,
  so MoveIt works without any changes
- Single unified controllers yaml contains both type declarations and joint params
- Spawner nodes load, configure, and activate each controller

Pipeline:
  1. Process upper_body.xacro (mujoco:=true) → robot_state_publisher (TF/RViz)
  2. Start mujoco_ros2_control with upper_body_xp.xml + unified controllers yaml
  3. Spawner nodes activate joint_state_broadcaster, left_controller, right_controller
  4. (Optional) Start RViz2

Usage:
  source ~/mujoco_ws/install/setup.bash
  source ~/clean_ws/install/setup.bash
  ros2 launch upper_body_mujoco mujoco.launch.py

  # Options
  ros2 launch upper_body_mujoco mujoco.launch.py rviz:=false
  ros2 launch upper_body_mujoco mujoco.launch.py realtime:=0.5
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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def create_nodes(context: LaunchContext):
    # ── Paths ──────────────────────────────────────────────────────────────────
    upper_body_share        = get_package_share_directory('upper_body')
    upper_body_mujoco_share = get_package_share_directory('upper_body_mujoco')

    xacro_file       = os.path.join(upper_body_share, 'urdf', 'upper_body.xacro')
    # Single yaml: controller types + joint/interface params at top-level per controller
    controllers_yaml = os.path.join(upper_body_mujoco_share, 'config',
                                    'upper_body_mujoco_controllers.yaml')
    mujoco_xml       = os.path.join(upper_body_mujoco_share, 'config',
                                    'upper_body_xp.xml')

    # ── Launch argument values ──────────────────────────────────────────────────
    rviz_enabled = LaunchConfiguration('rviz').perform(context)
    realtime     = LaunchConfiguration('realtime').perform(context)
    sim_freq     = LaunchConfiguration('sim_freq').perform(context)

    # ── Process URDF (mujoco:=true activates MujocoSystem ros2_control block) ──
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            'use_fake_hardware': 'false',
            'mujoco': 'true',
        }
    ).toprettyxml(indent='  ')

    robot_description = {'robot_description': robot_description_xml}

    # ── 1. Robot State Publisher (TF tree + RViz) ───────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ],
    )

    # ── 2. mujoco_ros2_control ─────────────────────────────────────────────────
    # The unified yaml declares controller types under controller_manager and
    # provides joint/interface params at the top level (e.g. left_controller.ros__parameters).
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controllers_yaml,
            {'simulation_frequency': float(sim_freq)},
            {'realtime_factor':      float(realtime)},
            {'robot_model_path':     mujoco_xml},
            {'show_gui':             True},
            {'use_sim_time':         True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    # ── 3. Controller spawners ──────────────────────────────────────────────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    left_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='left_controller_spawner',
        arguments=['left_controller', '-c', '/controller_manager'],
        output='screen',
    )

    right_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='right_controller_spawner',
        arguments=['right_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # ── 4. RViz2 (optional) ────────────────────────────────────────────────────
    rviz2 = Node(
        condition=IfCondition(rviz_enabled),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': True}],
    )

    # ── Event chain: wait for mujoco to start before spawning controllers ──────
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco_node,
            on_start=[
                LogInfo(msg='[upper_body_mujoco] MuJoCo started. Spawning controllers...'),
                joint_state_broadcaster,
                left_controller,
                right_controller,
                rviz2,
            ],
        )
    )

    return [
        robot_state_publisher,
        mujoco_node,
        load_controllers,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 alongside MuJoCo.',
        ),
        DeclareLaunchArgument(
            'realtime',
            default_value='1.0',
            description='MuJoCo real-time factor (1.0 = wall speed).',
        ),
        DeclareLaunchArgument(
            'sim_freq',
            default_value='500.0',
            description='MuJoCo simulation frequency in Hz.',
        ),
        OpaqueFunction(function=create_nodes),
    ])
