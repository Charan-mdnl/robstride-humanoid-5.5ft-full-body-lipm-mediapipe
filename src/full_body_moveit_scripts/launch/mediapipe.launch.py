from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='full_body_moveit_scripts',
            executable='mediapipe_exe',
            name='gesture_handler',
            output='screen',
        ),
    ])
