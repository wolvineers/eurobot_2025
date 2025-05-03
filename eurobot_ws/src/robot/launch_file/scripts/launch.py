import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),

        TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package='controller',
                    executable='driver_01_node.py',
                    name='driver01',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package='controller',
                    executable='driver_02_node.py',
                    name='driver02',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='controller',
                    executable='controller_node.py',
                    name='controller',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                launch_ros.actions.Node(
                    package='lidar',
                    executable='lidar_subscriber_node',
                    name='lidar_subscriber',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package='emergency_button',
                    executable='emergency_button',
                    name='emergency_button',
                    output='screen'
                )
            ]
        ),

    ])
