#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
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
        self.end_order_    = True
        self.encoder_left_ = 0

        ''' --- LIST ELEMENTS ---
        * 
        * Tuple key = m:
        *   - linear_velocity    --> from 0.0 to 1.0
        *   - angular_velocity   --> from 0.0 to 1.0 (where 1000.0 = 100.0 in linear velocity)
        *   - movement_time
        *
        * Tuple key = a:
        *   - action number
        '''
        self.movements_list_ = [
            ('m', (-0.3, 0.0, 25.0)),
            ('m', (0.3, 0.0, 25.0)),
            # ('m', (0.0, 0.1, 45.0)),
            # ('m', (0.2, 0.0, 30.0)),
            # ('m', (10.0, 0.0, 50.0)),
            # ('a', 2),
            # ('a', 3),
            # ('a', 2),
            # ('a', 3)
        ]

        # Publishers
        self.robot_mov_pub_     = self.create_publisher(Vector3, '/movement', 10)
        self.torete_act_pub_    = self.create_publisher(Int32, '/t_action', 10)
        self.insomnious_act_pub = self.create_publisher(Int32, '/i_action', 10)

        # Subscribers
        self.end_order_sub_    = self.create_subscription(Bool, '/controller/end_order', self.end_order_callback, 10)
        self.encoder_left_sub_ = self.create_subscription(Float32, '/controller/encoder_left', self.encoder_left_callback, 10)

        # Timers
        self.movement_tim_ = self.create_timer(self.timer_period_, self.add_movement)


    def end_order_callback(self, end_order):
        """
        Gets true if the order sended before is finished, false otherwise.

        Args:
            end_order (Bool): The message received by the subscriber, containing the state of the last order.
        """
        
        self.end_order_ = end_order.data
        #self.get_logger().info("End order callback: " + str(end_order.data))


    def encoder_left_callback(self, encoder_left):
        """
        Gets the value of encoder_left and save it to encoder_left_.

        Args:
            encoder_left (Float32): The message received by the subscriber, containing encoder data.
        """

        self.encoder_left_ = encoder_left.data
        self.get_logger().info("Encoder left value: " + str(self.encoder_left_))


    def add_movement(self):
        """
        At each elapsed time, if the last order is finished, publishes the first order in the array and removes it after publishing.
        """

        if self.end_order_ and self.movements_list_ and self.encoder_left_ == 0:
            
            if self.movements_list_[0][0] == 'm':
                movement_msg = Vector3()

                movement_msg.x = self.movements_list_[0][1][0]
                movement_msg.y = self.movements_list_[0][1][1]
                movement_msg.z = self.movements_list_[0][1][2]

                self.get_logger().info("Movement publisher (x, y, z): (" + str(movement_msg.x) + ", " + str(movement_msg.y) + ", " + str(movement_msg.z) + ")")
                
                self.robot_mov_pub_.publish(movement_msg)

            elif self.movements_list_[0][0] == 'a':
                action_msg = Int32()

                action_msg.data = self.movements_list_[0][1]

                self.get_logger().info("Action publisher: " + str(action_msg.data))

                self.torete_act_pub_.publish(action_msg)

            self.movements_list_.pop(0)
            self.end_order_ = False
           

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