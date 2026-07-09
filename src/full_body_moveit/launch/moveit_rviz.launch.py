"""
moveit_rviz.launch.py
=====================
Launches RViz2 with the MoveIt Motion Planning plugin pre-configured
for the Angad full-body humanoid.

Start the simulation first, then this:
  ros2 launch full_body_mujoco  full_body_mujoco.launch.py rviz:=false
  ros2 launch full_body_moveit  move_group.launch.py
  ros2 launch full_body_moveit  moveit_rviz.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_mujoco  = get_package_share_directory('full_body_mujoco')
    pkg_moveit  = get_package_share_directory('full_body_moveit')

    clean_ld_library_path = ':'.join(
        p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':')
        if p and '/snap/' not in p
    )
    clean_env = {
        'LD_LIBRARY_PATH': clean_ld_library_path,
        'GTK_PATH': '',
        'GTK_EXE_PREFIX': '',
        'GIO_MODULE_DIR': '',
        'GTK_IM_MODULE_FILE': '',
        'GIO_LAUNCHED_DESKTOP_FILE': '',
        'GIO_LAUNCHED_DESKTOP_FILE_PID': '',
        'GDK_PIXBUF_MODULEDIR': '',
        'GDK_PIXBUF_MODULE_FILE': '',
        'QT_PLUGIN_PATH': '',
        'QML2_IMPORT_PATH': '',
        'SNAP': '',
        'SNAP_NAME': '',
        'SNAP_REVISION': '',
        'SNAP_ARCH': '',
        'SNAP_LIBRARY_PATH': '',
        'LD_PRELOAD': '',
    }

    moveit_config = (
        MoveItConfigsBuilder('angad_full_body', package_name='full_body_moveit')
        .robot_description(
            file_path=os.path.join(pkg_mujoco, 'urdf', 'full_body.xacro')
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_moveit, 'config', 'full_body.srdf')
        )
        .robot_description_kinematics(
            file_path=os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
        )
        .joint_limits(
            file_path=os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')
        )
        .trajectory_execution(
            file_path=os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml')
        )
        .planning_pipelines(
            pipelines=['ompl']
        )
        .to_moveit_configs()
    )

    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        additional_env=clean_env,
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([rviz_node])
