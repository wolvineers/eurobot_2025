#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
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
        self.action_commands_sub_ = self.create_subscription(JointActionPoint, '/controller/action_commands', self.actions_commands_callback, 10)

        # Publishers
        self.end_action_pub_ = self.create_publisher(Bool, "/controller/end_action_02", 10)
        self.imu_pub_        = self.create_publisher(Float32, "/controller/imu", 10)

        # Timers
        self.read_msgs_tim_ = self.create_timer(self.timer_period_, self.read_msgs_timer)



    def actions_commands_callback(self, action_commands):
        """
        Gets the servos positions and sends all to the ESP32.

        Args:
            action_commands (JointTrajectory): The message received by the subscriber, containing the servos positions.
        """

        # Servos message
        positions_msg = ""

        for i, servo in enumerate(action_commands.servos_names):
            positions_msg += f"{servo},"
            positions_msg += f"{action_commands.position[i]},"

        positions_msg += f"AP,{action_commands.activate}"

        self.get_logger().info(positions_msg)

        send_message(self.serial_port, positions_msg)


    def read_msgs_timer(self):
        """
        Reads the message sended by Esp32 containing the imu data or the end action value and publish this data.
        """
        
        # self.get_logger().info("Volta")
        received_msg = read_message(self.serial_port)
        
        if received_msg != None:

            # Split the message into parts separated by commas and remove the last part (checksum)
            msg_parts = received_msg.split(",")[:-1]
            self.get_logger().info('Taula missatge: ' + str(msg_parts))

            # Process command pairs
            for i_msg_parts in range(0, len(msg_parts), 2):
                msg_element = msg_parts[i_msg_parts]
                msg_value   = float(msg_parts[i_msg_parts + 1])

                if msg_element == "EA":
                    end_msg = Bool()
                    end_msg.data = True

                    self.end_action_pub_.publish(end_msg)
                else:
                    self.get_logger().info('IMU: ' + str(msg_value))
                    imu_msg = Float32()
                    imu_msg.data = msg_value

                    self.imu_pub_.publish(imu_msg)



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