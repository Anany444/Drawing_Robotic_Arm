import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory 

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( 
            os.path.join(
                get_package_share_directory("bringup"),
                "launch", "gazebo.launch.py"
            ) 
        )
    )
    
    basic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( 
            os.path.join( 
                get_package_share_directory("bringup"), 
                "launch", "basic.launch.py" 
            ) 
        )
    )
    
    return LaunchDescription([
        gazebo_launch,
        basic_launch
    ])