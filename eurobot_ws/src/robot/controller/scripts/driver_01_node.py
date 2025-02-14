#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from utils.msg import MotorPower
from std_msgs.msg import Float32


## *************************
## FIRST DRIVER CLASS
## *************************

class FirstDriverNode(Node):
    
    def __init__(self):
        """
        Initializes all the attributes and the subscribers of the node.
        """

        super().__init__('first_driver')

        # Attributes
        self.motors_pow_ = [0] * 4
        self.timer_period_ = 0.03

        # Subscribers
        self.motors_pow_sub_ = self.create_subscription(MotorPower, '/controller/motors_pow', self.move_callback, 10)

        # Publishers
        self.encoder_left_pub_ = self.create_publisher(Float32, '/controller/encoder_left', 10)
        self.encoder_right_pub_ = self.create_publisher(Float32, '/controller/encoder_right', 10)

        # Timers
        self.encoders_tim_ = self.create_timer(self.timer_period_, self.encoders_callback)


    def set_motor_vel():
        """
        Assigns the corresponding power to each motor taken from the motors_pow_ array.
        """

        # TO DO: Implement the code to assign the power to each motor taken from the motors_pow_ array
        # motors_pow_ is an array where each position corresponds to the motor number in the esp32
        ...

    def move_callback(self, motor_pow):
        """
        Stores the motor power data in motors_pow_ and assigns each motor power.

        Args:
            motor_pow (MotorPower): The message received by the subscriber, containing motor power data.
        """

        if len(motor_pow.motor_power) == 4:
            self.motors_pow_ = motor_pow.motor_power
            self.get_logger().info('Motor powers: %s' % str(self.motors_pow_.tolist()))
        else:
            self.get_logger().warn('Invalid data. Expected 4 values, got %d.' % len(motor_pow.motor_power))


        # set_motor_vel()

    def encoders_callback(self):
        """
        Gets the encoder values and publish to its topic
        """

        # Get the motors distance with the port serie communication
        # ** The values must be in centimeters
        encoder_left = ...      # TO DO: read the encoder left value with the functions implemented in file utils/src/serial_communication.py
        encoder_right = ...     # TO DO: read the encoder right value with the functions implemented in file utils/src/serial_communication.py

        # Publish the robot distance
        encoder_left_msg = Float32()
        encoder_right_msg = Float32()

        encoder_left_msg.data = encoder_left
        encoder_right_msg.data = encoder_right

        self.encoder_left_pub_.publish(encoder_left_msg)
        self.encoder_right_pub_.publish(encoder_right_msg)

        self.get_logger().info(f'Encoder left: {encoder_left_msg.data:.2f}  |  Encoder right: {encoder_right_msg.data:.2f}')



## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    first_driver_node = FirstDriverNode()

    rclpy.spin(first_driver_node)

    first_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()