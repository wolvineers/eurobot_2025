#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from utils.scripts.serial_communication import open_serial_port, send_message, read_message


## ***********************
## SECOND DRIVER CLASS
## ***********************

class SecondDriverNode(Node):
    
    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('second_driver')


        # Attributes
        self.port        = '/dev/ttyUSB0'
        self.baudrate    = 115200
        self.serial_port = open_serial_port(self.port, self.baudrate)

        self.timer_period_ = 0.1

        # Subscribers
        self.action_commands_sub_  = self.create_subscription(JointTrajectory, '/controller/action_commands', self.actions_commands_callback, 10)


    def actions_commands_callback(self, action_commands):
        """
        Gets the servos positions and sends all to the ESP32.

        Args:
            action_commands (JointTrajectory): The message received by the subscriber, containing the servos positions.
        """

        ''' --- SERVOS POSITIONS MESSAGE ---
        *   
        *
        *
        '''

        for i in range(len(action_commands.joint_names)):
            # Get servo number and position
            num_servo = action_commands.joint_names[i]
            pos_servo = action_commands.points[0].positions[i]

            # Send message to the ESP32
            send_message(self.serial_port, f"{num_servo},{pos_servo}")


## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    second_driver_node = SecondDriverNode()

    rclpy.spin(second_driver_node)

    second_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()