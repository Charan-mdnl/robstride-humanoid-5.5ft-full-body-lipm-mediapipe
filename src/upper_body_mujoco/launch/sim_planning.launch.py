import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def create_nodes(context: LaunchContext):
    # --- Paths ---
    upper_body_share        = get_package_share_directory('upper_body')
    upper_body_mujoco_share = get_package_share_directory('upper_body_mujoco')
    moveit_config_share     = get_package_share_directory('angad_moveit_upper_arm')

    xacro_file       = os.path.join(upper_body_share, 'urdf', 'upper_body.xacro')
    controllers_yaml = os.path.join(upper_body_mujoco_share, 'config',
                                    'upper_body_mujoco_controllers.yaml')
    mujoco_xml       = os.path.join(upper_body_mujoco_share, 'config',
                                    'upper_body_xp.xml')
    rviz_config      = os.path.join(moveit_config_share, 'config', 'moveit.rviz')

    # --- Launch argument values ---
    rviz_enabled = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    mj_gui       = LaunchConfiguration('mj_gui').perform(context).lower() == 'true'
    sim_freq     = LaunchConfiguration('sim_freq').perform(context)
    realtime     = LaunchConfiguration('realtime').perform(context)

    # --- Process URDF ---
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={'mujoco': 'true', 'use_fake_hardware': 'false'}
    ).toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_xml}

    # --- MoveIt Config ---
    moveit_config = (
        MoveItConfigsBuilder("upper_body", package_name="angad_moveit_upper_arm")
        .robot_description(mappings={'mujoco': 'true'})
        .to_moveit_configs()
    )

    # --- 1. Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # --- 2. MuJoCo Node (Using original working configuration) ---
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
            {'show_gui':             mj_gui},
            {'use_sim_time':         True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    # --- 3. MoveGroup Node ---
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            # Tell MoveIt exactly which controller manager to talk to
            {'controller_manager_name': '/controller_manager'},
        ],
    )

    # --- 4. RViz2 ---
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
        arguments=['-d', rviz_config],
    )

    # --- 5. Controller Spawners ---
    def create_spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '-c', '/controller_manager'],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco_node,
            on_start=[
                LogInfo(msg='MuJoCo started. Spawning controllers and MoveIt...'),
                create_spawner('joint_state_broadcaster'),
                create_spawner('left_controller'),
                create_spawner('right_controller'),
                # Delay move_group by 3s so /clock is publishing before MoveIt init
                TimerAction(
                    period=3.0,
                    actions=[
                        move_group_node,
                        rviz2_node if rviz_enabled else LogInfo(msg='RViz disabled.'),
                    ]
                ),
            ],
        )
    )

    return [
        robot_state_publisher,
        mujoco_node,
        load_controllers,
    ]

def generate_launch_description():
    from launch import LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2.'),
        DeclareLaunchArgument('mj_gui', default_value='true', description='Launch MuJoCo GUI.'),
        DeclareLaunchArgument('sim_freq', default_value='500.0', description='Sim frequency.'),
        DeclareLaunchArgument('realtime', default_value='1.0', description='Realtime factor.'),
        OpaqueFunction(function=create_nodes),
    ])
