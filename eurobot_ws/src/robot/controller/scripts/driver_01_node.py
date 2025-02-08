#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from utils.msg import MotorPower


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

        # Subscribers
        self.motors_pow_sub_ = self.create_subscription(MotorPower, '/controller/motors_pow', self.move_callback, 10)


    def set_motor_vel():
        """
        Assigns the corresponding power to each motor taken from the motors_pow_ array.
        """
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