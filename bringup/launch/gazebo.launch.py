import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory("description")

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )
   
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 2 -r'"])
                }.items()
             )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "mk1",
            "-x", "0.0",  
            "-y", "0.0",  
            "-z", "0.0",  
            "-R", "0.0", 
            "-P", "0.0",
            "-Y", "0.0", # Yaw (in radians, e.g., 1.57 for 90 degrees)
        ],
    )


    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            #"/pen_contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts",
            #"/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            #"/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            #"/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            #"/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            #"/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            #"/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            #"/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            #"/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            #"/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            #"/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
    )

    return LaunchDescription([
        world_name_arg,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])