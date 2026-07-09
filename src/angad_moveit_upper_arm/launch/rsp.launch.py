from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("upper_body", package_name="angad_moveit_upper_arm").robot_description(mappings={'use_fake_hardware': 'true'}).to_moveit_configs()
    return generate_rsp_launch(moveit_config)
