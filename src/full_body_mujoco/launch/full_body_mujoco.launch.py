"""
full_body_mujoco.launch.py
==========================
ROS 2 launch file for the Angad full-body humanoid with MuJoCo physics viewer
and ros2_control using fake_hardware (mock_components/GenericSystem).

Pipeline:
  1. Process full_body.xacro → robot_state_publisher (TF)
  2. Start ros2_control_node with fake_hardware
  3. Spawner nodes activate all controllers
  4. Start MuJoCo C++ physics viewer (subscribes to /joint_states)

Setup:
  source ~/clean_ws/install/setup.bash
  colcon build --packages-select full_body_mujoco
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.local/lib/python3.10/site-packages/mujoco
  export MUJOCO_PLUGIN_PATH=~/.local/lib/python3.10/site-packages/mujoco/plugin
  ros2 launch full_body_mujoco full_body_mujoco.launch.py

Options:
  use_sim_time:=true/false    (default: false for fake hardware)
"""

import os
import tempfile

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
    pkg_share   = get_package_share_directory('full_body_mujoco')
    xp_mesh_dir = '/home/me/clean_ws/src/XP_robot_with_hand_mjcf/meshes'   # absolute, canonical

    xacro_file       = os.path.join(pkg_share, 'urdf',   'full_body.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'full_body_controllers.yaml')
    plugins_yaml     = os.path.join(pkg_share, 'config', 'mujoco_plugins.yaml')
    mujoco_xml_templ = os.path.join(pkg_share, 'config', 'angad_full_body.xml')

    # ── Launch argument values ─────────────────────────────────────────────────
    rviz_enabled = LaunchConfiguration('rviz').perform(context)
    realtime     = LaunchConfiguration('realtime').perform(context)
    sim_freq     = LaunchConfiguration('sim_freq').perform(context)

    # ── Inject mesh path into MuJoCo XML ──────────────────────────────────────
    # angad_full_body.xml contains the literal string MESHDIR_PLACEHOLDER where
    # the meshdir attribute value should go.  We replace it at launch time so
    # the installed file remains path-independent.
    with open(mujoco_xml_templ, 'r') as f:
        xml_content = f.read()

    xml_patched = xml_content.replace('MESHDIR_PLACEHOLDER', xp_mesh_dir)

    # Write to a temp file (deleted when OS cleans up /tmp)
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='angad_full_body_',
        suffix='.xml',
        delete=False,
    )
    tmp.write(xml_patched)
    tmp.flush()
    tmp.close()
    mujoco_xml = tmp.name

    # ── Process URDF xacro → robot_description (TF + ros2_control) ───────────
    robot_description_xml = xacro.process_file(
        xacro_file, mappings={'headless': 'true'}).toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_xml}

    # ── 1. Robot State Publisher ───────────────────────────────────────────────
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
    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controllers_yaml,
            plugins_yaml,
            {'simulation_frequency': float(sim_freq)},
            {'realtime_factor':      float(realtime)},
            {'robot_model_path':     mujoco_xml},
            {'use_sim_time':         True},
            {'initial_keyframe_key': 'standing'},  # load standing pose at t=0
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

    # ── 5. Standing pose publisher — commands initial pose to all controllers ────
    hold_standing_pose = Node(
        package='full_body_mujoco',
        executable='foot_force_array_publisher.py',
        name='foot_force_array_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Event chain: spawn controllers after MuJoCo node starts ───────────────
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco_node,
            on_start=[
                LogInfo(msg='[full_body_mujoco] MuJoCo started. Spawning controllers...'),
                joint_state_broadcaster,
                lower_body_controller,
                torso_controller,
                left_arm_controller,
                right_arm_controller,
                right_hand_controller,
                left_hand_controller,
                #hold_standing_pose,
            ],
        )
    )

    # ── Viewer: ik_sphere_viz.py (correct axes, shows IK markers) ────────────
    viewer = Node(
        package='full_body_mujoco',
        executable='ik_sphere_viz.py',
        name='ik_sphere_viz',
        output='screen',
    )

    return [
        robot_state_publisher,
        mujoco_node,
        load_controllers,
        viewer,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 for visualisation',
        ),
        DeclareLaunchArgument(
            'realtime',
            default_value='1.0',
            description='MuJoCo real-time factor (0.0 = max speed)',
        ),
        DeclareLaunchArgument(
            'sim_freq',
            default_value='500.0',
            description='Simulation frequency in Hz (must match MuJoCo timestep)',
        ),
        OpaqueFunction(function=create_nodes),
    ])
