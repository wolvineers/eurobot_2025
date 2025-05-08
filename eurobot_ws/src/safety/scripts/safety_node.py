#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utils.scripts.serial_communication import open_serial_port, send_message
import RPi.GPIO as GPIO

BUTTON_PIN = 40
port01 = '/dev/ttyUSB0'
port02 = '/dev/ttyACM0'
baudrate = 115200
serial_port01 = open_serial_port(port01, baudrate)
serial_port02 = open_serial_port(port02, baudrate)  

class EmergencyButtonNode(Node):
    def __init__(self):
        super().__init__('emergency_button_node')
        self.publisher_ = self.create_publisher(String, 'emergency_button', 10)
        self.timer = self.create_timer(1.0, self.check_button_state)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.get_logger().info('Emergency Button Node started.')

    def check_button_state(self):
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            send_message(serial_port01, "Emergency,1")
            send_message(serial_port02, "Emergency,1")
            self.get_logger().info('Emergency button pressed! Sending STOP command.')
            msg = String()
            msg.data = 'Emergency button pressed!'
            self.publisher_.publish(msg)
        
        elif GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            send_message(serial_port01, "Emergency,0")
            send_message(serial_port02, "Emergency,0")
            self.get_logger().info('Emergency button released! Sending CONTINUE command.')
            msg = String()
            msg.data = 'Emergency button released!'
            self.publisher_.publish(msg)


    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyButtonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()