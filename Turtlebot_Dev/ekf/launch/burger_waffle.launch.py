import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the launch files
    burger_ekf = os.path.join(
        get_package_share_directory('ekf'), 'launch', 'burger_ekf.launch.py')
    
    waffle_ekf = os.path.join(
        get_package_share_directory('ekf'), 'launch', 'waffle_ekf.launch.py')
    
    return LaunchDescription([
        # Launch first_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(burger_ekf),
            launch_arguments={}.items(),
        ),

        # Delay before launching the robot_state_publisher
        TimerAction(
            period=5.0,  # Adjust the delay time as needed
            actions=[
                # Launch second_launch.py
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(waffle_ekf),
                    launch_arguments={}.items(),
                ),
            ],
        ),
       

    ])
