"""
full_body_mujoco_viewer.launch.py
==================================
ROS 2 launch file for the Angad full-body humanoid with MuJoCo C++ physics viewer
and ros2_control using fake_hardware (mock_components/GenericSystem).

Pipeline:
  1. Process full_body.xacro with fake_hardware → robot_state_publisher (TF)
  2. Start ros2_control_node with mock_components/GenericSystem
  3. Spawner nodes activate all controllers
  4. Start MuJoCo C++ physics viewer (subscribes to /joint_states)

Setup:
  source ~/clean_ws/install/setup.bash
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.local/lib/python3.10/site-packages/mujoco
  export MUJOCO_PLUGIN_PATH=~/.local/lib/python3.10/site-packages/mujoco/plugin
  ros2 launch full_body_mujoco full_body_mujoco_viewer.launch.py

Options:
  use_sim_time:=true/false    (default: false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def create_nodes(context: LaunchContext):
    # ── Paths ──────────────────────────────────────────────────────────────────
    pkg_share_mujoco = get_package_share_directory('full_body_mujoco')
    pkg_share_fake   = get_package_share_directory('full_body_fake_hardware')

    xacro_file       = os.path.join(pkg_share_fake, 'urdf',   'full_body.xacro')
    controllers_yaml = os.path.join(pkg_share_fake, 'config', 'full_body_controllers.yaml')

    # ── Launch argument values ─────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # ── Process URDF xacro with fake_hardware → robot_description ─────────────
    robot_description_xml = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_xml}

    # ── 1. Robot State Publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time == 'true'},
        ],
    )

    # ── 2. ros2_control_node with fake_hardware ────────────────────────────────
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            controllers_yaml,
            {'use_sim_time': use_sim_time == 'true'},
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

    right_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='right_hand_controller_spawner',
        arguments=['right_hand_controller', '-c', '/controller_manager'],
        output='screen',
    )

    left_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='left_hand_controller_spawner',
        arguments=['left_hand_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # ── Event chain: spawn controllers sequentially after controller_manager starts ────────
    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                LogInfo(msg='[full_body_mujoco_viewer] controller_manager started. Spawning joint_state_broadcaster...'),
                joint_state_broadcaster,
            ],
        )
    )

    load_lower_body = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg='[full_body_mujoco_viewer] Spawning lower_body_controller...'),
                lower_body_controller,
            ],
        )
    )

    load_torso = RegisterEventHandler(
        OnProcessExit(
            target_action=lower_body_controller,
            on_exit=[
                LogInfo(msg='[full_body_mujoco_viewer] Spawning torso_controller...'),
                torso_controller,
            ],
        )
    )

    load_left_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=torso_controller,
            on_exit=[
                LogInfo(msg='[full_body_mujoco_viewer] Spawning left_arm_controller...'),
                left_arm_controller,
            ],
        )
    )

    load_right_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_controller,
            on_exit=[
                LogInfo(msg='[full_body_mujoco_viewer] Spawning right_arm_controller...'),
                right_arm_controller,
            ],
        )
    )

    load_right_hand = RegisterEventHandler(
        OnProcessExit(
            target_action=right_arm_controller,
            on_exit=[
                LogInfo(msg='[full_body_mujoco_viewer] Spawning right_hand_controller...'),
                right_hand_controller,
            ],
        )
    )

    load_left_hand = RegisterEventHandler(
        OnProcessExit(
            target_action=right_hand_controller,
            on_exit=[
                LogInfo(msg='[full_body_mujoco_viewer] Spawning left_hand_controller...'),
                left_hand_controller,
            ],
        )
    )

    # ── 4. MuJoCo C++ Physics Viewer (subscribes to /joint_states) ────────────
    mujoco_viewer = Node(
        package='full_body_mujoco',
        executable='ik_mujoco_viewer',
        name='ik_mujoco_viewer',
        output='screen',
    )

    return [
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_lower_body,
        load_torso,
        load_left_arm,
        load_right_arm,
        load_right_hand,
        load_left_hand,
        mujoco_viewer,
    ]


def generate_launch_description():
    # Set environment variables for MuJoCo
    mujoco_lib_path = os.path.expanduser('~/.local/lib/python3.10/site-packages/mujoco')
    mujoco_plugin_path = os.path.join(mujoco_lib_path, 'plugin')
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=mujoco_lib_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')
        ),
        SetEnvironmentVariable(
            name='MUJOCO_PLUGIN_PATH',
            value=mujoco_plugin_path
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (false for fake_hardware)',
        ),
        OpaqueFunction(function=create_nodes),
    ])
