import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ekf_launch = os.path.join(
        get_package_share_directory('ekf'), 'launch', 'burger_waffle.launch.py')
    
    
    teleop_launch = os.path.join(
        get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')
    
    
    control_turtlebots = Node(
        package='control_turtlebots',  
        executable='control_turtlebots',  
        name='control_turtlebots',  
        output='screen', 
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch)
        ),
        control_turtlebots,

    ])