from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("upper_body", package_name="angad_moveit_upper_arm").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
