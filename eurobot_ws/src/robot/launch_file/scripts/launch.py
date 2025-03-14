import launch
import launch_ros.actions


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
        )
    ])

