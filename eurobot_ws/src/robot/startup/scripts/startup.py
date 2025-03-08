#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool

from utils.scripts.serial_communication import open_serial_port, read_message

## ***********************
## STARTUP CLASS
## ***********************

class StartUp(Node):

    def __init__(self):
        """
        Initializes all the attributes and publishers of the node.
        """

        super().__init__('startup')

        # Attributes
        self.port        = '/dev/ttyUSB0'
        self.baudrate    = 115200
        self.serial_port = open_serial_port(self.port, self.baudrate)

        # Publishers
        self.startup_pub = self.create_publisher(Bool, '/startup', 10)

        # Timers
        self.timer = self.create_timer(0.5, self.callback_startup)


    def callback_startup(self):
        """
        Gets de value of the limit switch and publish another message to start all in the robot.

        Args:
            startup_msg (Bool): The message recived by serial port, containing the limit switch data
        """

        startup_msg = read_message(self.serial_port)

        msg = Bool()
        msg.data = startup_msg
        self.startup_pub.publish(msg)


## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)
    
    startup_node = StartUp()

    rclpy.spin(startup_node)

    startup_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()