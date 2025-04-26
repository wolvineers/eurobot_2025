#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from msgs.msg import JointActionPoint

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
        self.action_commands_sub_  = self.create_subscription(JointActionPoint, '/controller/action_commands', self.actions_commands_callback, 10)

        # Publishers
        self.end_motor_action_pub_  = self.create_publisher(Bool, '/controller/motor_action', 10)
        self.end_action_pub_ = self.create_publisher(Bool, "controller/end_action", 10)

        # Timers
        self.motor_action_tim_ = self.create_timer(self.timer_period_, self.end_action_timer)


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

        # Servos message
        positions_msg = ""

        for i, servo in enumerate(action_commands.servos_names):
            positions_msg += f"{servo},"
            positions_msg += f"{action_commands.position[i]},"

        positions_msg += f"AP,{action_commands.activate}"


        motors_msg = ""

        for i, motor in enumerate(action_commands.motors_names):
            motors_msg += f"{motor},"
            motors_msg += f"{action_commands.velocity[i]},"

        motors_msg = motors_msg[:-1]

        send_message(self.serial_port, positions_msg)


    def end_action_timer(self):
        """
        Reads the message sended by Esp32 containing the encoders value and publish this data.
        """
        
        # self.get_logger().info("Volta")
        end_action_msg = read_message(self.serial_port)
        
        if end_action_msg != None:

            # Separates the message into the three parts marked by commas and saves the values
            message_parts = end_action_msg.split(",")
            end_action    = int(message_parts[1])

            # Prepare and publish the message

            if end_action == 1:
                end_msg = Bool()
                end_msg.data = True

                self.end_action_pub_.publish(end_msg)


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