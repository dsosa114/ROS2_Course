from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mark_one_arm", package_name="mark_one_arm_moveit_setup").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
