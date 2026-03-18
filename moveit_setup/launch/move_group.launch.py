from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import SetParameter

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mk1", package_name="moveit_setup").to_moveit_configs()
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        generate_move_group_launch(moveit_config)
])
