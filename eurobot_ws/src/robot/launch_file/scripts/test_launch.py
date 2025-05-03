import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    return launch.LaunchDescription([
    
        # Nodes de prova: Talker i Listener de ROS2
        launch_ros.actions.Node(
            package='demo_nodes_cpp',  # Paquet estàndard de ROS2 amb nodes de demostració
            executable='talker',       # Executable del talker
            name='talker',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='demo_nodes_cpp',  # Paquet estàndard de ROS2 amb nodes de demostració
            executable='listener',     # Executable del listener
            name='listener',
            output='screen'
        ),
    ])
