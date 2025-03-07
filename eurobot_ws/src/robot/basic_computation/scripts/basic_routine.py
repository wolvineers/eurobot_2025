#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Vector3


## *************************
## BASIC ROUTINE CLASS
## *************************

class BasicRoutineNode(Node):
    
    def __init__(self):
        """
        Initializes all the attributes, publishers and timers of the node.
        """

        ## --- TO DO ---
        ##  - Read the list of movements from a .txt file

        super().__init__('basic_routine')

        # Attributes
        self.timer_period_ = 1

        ## movement_list_ element = (
        ##   linear_velocity    --> from 0.0 to 100.0
        ##   angular_velocity   --> from 0.0 to 1000.0 (where 1000.0 = 100.0 in linear velocity)
        ##   movement_time
        ## )
        self.movements_list_ = [
            (30.0, 0.0, 3.0),
            (0.0, 100.0, 4.0)
        ]

        # Publishers
        self.robot_mov_pub_ = self.create_publisher(Vector3, '/movement', 10)

        # Timers
        self.movement_tim_ = self.create_timer(self.timer_period_, self.add_movement)


    def add_movement(self):
        """
        At each elapsed time, publishes the first movement in the array and removes it after publishing.
        """
        
        if self.movements_list_:
            movement_msg = Vector3()

            movement_msg.x = self.movements_list_[0][0]
            movement_msg.y = self.movements_list_[0][1]
            movement_msg.z = self.movements_list_[0][2]

            self.get_logger().info("Movement publisher (x, y, z): (" + str(movement_msg.x) + ", " + str(movement_msg.y) + ", " + str(movement_msg.z) + ")")
            
            self.robot_mov_pub_.publish(movement_msg)

            self.movements_list_.pop(0)
           

## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    basic_routine_node = BasicRoutineNode()

    rclpy.spin(basic_routine_node)

    basic_routine_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()