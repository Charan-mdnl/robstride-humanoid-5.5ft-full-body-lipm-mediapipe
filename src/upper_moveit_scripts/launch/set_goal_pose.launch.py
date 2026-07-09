import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_mujoco = get_package_share_directory("full_body_mujoco")
    pkg_moveit = get_package_share_directory("full_body_moveit")

    moveit_config = (
        MoveItConfigsBuilder("angad_full_body", package_name="full_body_moveit")
        .robot_description(
            file_path=os.path.join(pkg_mujoco, "urdf", "full_body.xacro")
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_moveit, "config", "full_body.srdf")
        )
        .robot_description_kinematics(
            file_path=os.path.join(pkg_moveit, "config", "kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(pkg_moveit, "config", "joint_limits.yaml")
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument("group", default_value="right_arm"),
        DeclareLaunchArgument("ee_link", default_value="forearm_r"),
        DeclareLaunchArgument("frame_id", default_value="pelvis"),
        DeclareLaunchArgument("use_absolute_position", default_value="false"),
        DeclareLaunchArgument("x", default_value=".nan"),
        DeclareLaunchArgument("y", default_value=".nan"),
        DeclareLaunchArgument("z", default_value=".nan"),
        DeclareLaunchArgument("dx", default_value="0.0"),
        DeclareLaunchArgument("dy", default_value="0.0"),
        DeclareLaunchArgument("dz", default_value="0.0"),
        DeclareLaunchArgument("use_orientation", default_value="false"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.2"),
        DeclareLaunchArgument("acceleration_scaling", default_value="0.2"),
        DeclareLaunchArgument("execute", default_value="true"),
        Node(
            name="set_goal_pose_test",
            package="upper_moveit_scripts",
            executable="set_goal_pose_exe",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": True},
                {
                    "group": LaunchConfiguration("group"),
                    "ee_link": LaunchConfiguration("ee_link"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "use_absolute_position": LaunchConfiguration("use_absolute_position"),
                    "x": LaunchConfiguration("x"),
                    "y": LaunchConfiguration("y"),
                    "z": LaunchConfiguration("z"),
                    "dx": LaunchConfiguration("dx"),
                    "dy": LaunchConfiguration("dy"),
                    "dz": LaunchConfiguration("dz"),
                    "use_orientation": LaunchConfiguration("use_orientation"),
                    "velocity_scaling": LaunchConfiguration("velocity_scaling"),
                    "acceleration_scaling": LaunchConfiguration("acceleration_scaling"),
                    "execute": LaunchConfiguration("execute"),
                },
            ],
        ),
    ])
