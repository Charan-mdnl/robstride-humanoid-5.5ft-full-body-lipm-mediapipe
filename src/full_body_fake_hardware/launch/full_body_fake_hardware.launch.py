"""
full_body_fake_hardware.launch.py
==================================
Launches the full Angad humanoid with 100% mock ros2_control hardware.
No real motors or CAN bus required.

Pipeline:
  1. Process full_body.xacro  ->  robot_state_publisher (TF / RViz)
  2. ros2_control_node with mock_components/GenericSystem for all 21 joints
  3. Controller spawners: joint_state_broadcaster + 4 JointTrajectoryControllers
  4. RViz2 (optional, default: true)

Usage:
  source ~/clean_ws/install/setup.bash
  ros2 launch full_body_fake_hardware full_body_fake_hardware.launch.py
  ros2 launch full_body_fake_hardware full_body_fake_hardware.launch.py rviz:=false
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context: LaunchContext):
    pkg = get_package_share_directory('full_body_fake_hardware')

    xacro_file       = os.path.join(pkg, 'urdf',   'full_body.xacro')
    controllers_yaml = os.path.join(pkg, 'config', 'full_body_controllers.yaml')
    rviz_config      = os.path.join(pkg, 'config', 'display.rviz')

    rviz_enabled = LaunchConfiguration('rviz').perform(context)

    robot_description_xml = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_xml}

    # 1. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    # 2. ros2_control_node (mock hardware)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            controllers_yaml,
            {'use_sim_time': False},
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
    )

    # 3. Controller spawners
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
    lower_body = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lower_body_controller', '-c', '/controller_manager'],
        output='screen',
    )
    torso = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['torso_controller', '-c', '/controller_manager'],
        output='screen',
    )
    left_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '-c', '/controller_manager'],
        output='screen',
    )
    right_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '-c', '/controller_manager'],
        output='screen',
    )
    right_hand = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_hand_controller', '-c', '/controller_manager'],
        output='screen',
    )
    left_hand = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_hand_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # 4. RViz2 — use ExecuteProcess with a filtered env so snap's libpthread
    #    doesn't shadow the system glibc when launched from VSCode snap
    snap_drop = {'SNAP', 'SNAP_LIBRARY_PATH', 'SNAP_DATA', 'SNAP_COMMON',
                 'SNAP_USER_DATA', 'SNAP_USER_COMMON', 'SNAP_INSTANCE_NAME'}
    clean_env = {k: v for k, v in os.environ.items() if k not in snap_drop}

    from launch_ros.substitutions import FindPackageShare
    rviz2 = ExecuteProcess(
        condition=IfCondition(rviz_enabled),
        cmd=['rviz2', '-d', rviz_config],
        output='log',
        env=clean_env,
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                LogInfo(msg='[fake_hardware] controller_manager up — spawning controllers'),
                jsb,
                lower_body,
                torso,
                left_arm,
                right_arm,
                right_hand,
                left_hand,
            ],
        )
    )

    return [rsp, controller_manager, load_controllers, rviz2]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2',
        ),
        OpaqueFunction(function=create_nodes),
    ])
