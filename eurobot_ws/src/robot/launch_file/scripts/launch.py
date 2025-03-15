import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

## ***************
## LAUNCH FILE
## ***************

"""
Processes the received LIDAR scan data and determines if there is an obstacle.

Args:
        * package    --> where is the node
        * executable --> node executable name
        * name       --> node name
        * outupt     --> always screen
"""

def generate_launch_description():

    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),  
        'launch',
        'rplidar_a1_launch.py'  
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='controller',  
            executable='controller_node.py',       
            name='controller',          
            output='screen'
        ),
        launch_ros.actions.Node(
            package='controller',  
            executable='driver_01_node.py',       
            name='driver01',          
            output='screen'
        ),
        launch_ros.actions.Node(
            package='basic_computation',  
            executable='basic_routine.py',       
            name='basic_routine',          
            output='screen'
        ),
        launch_ros.actions.Node(
            package='lidar',  
            executable='lidar_subscriber_node',       
            name='lidar_subscriber',          
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        )
    ])
