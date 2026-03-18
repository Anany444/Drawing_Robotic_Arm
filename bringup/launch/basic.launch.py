import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource


robot_description_arg = DeclareLaunchArgument(
    name="robot_urdf",
    default_value=os.path.join(
        get_package_share_directory("annin_ar4_description"), "urdf", "ar.urdf.xacro"
    ),
    description="Absolute path to robot urdf file"
)

robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("robot_urdf")
        ]),
        value_type=str
)

controllers_file= os.path.join(
    get_package_share_directory("bringup"), "config", "ros2_controllers.yaml"
)

def generate_launch_description():
    
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description, "use_sim_time": True}]
    )
    
    ros2_control_node = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        parameters = [controllers_file, {"use_sim_time": True}]
    )
    
    joint_state_broadcaster = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )
    
    arm_controller = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["arm_controller", "--controller-manager", "/controller_manager"]
    )
    
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("moveit_setup"), "launch", "move_group.launch.py")
        )
    )
    
    rviz = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", os.path.join(get_package_share_directory("bringup"), "config", "rviz_config.rviz")],
        parameters = [{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        robot_description_arg,
        robot_state_publisher,
        #ros2_control_node,
        joint_state_broadcaster,
        arm_controller,
        move_group_launch,
        rviz
    ])
    