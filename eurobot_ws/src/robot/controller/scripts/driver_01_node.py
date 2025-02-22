#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from utils.msg import MotorPower
from std_msgs.msg import Float32
from serial_communication import open_serial_port, send_message, read_message


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
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.serial_port = open_serial_port(self.port, self.baudrate)

        self.motors_pow_ = [0] * 4
        self.timer_period_ = 0.03

        # Subscribers
        self.motors_pow_sub_ = self.create_subscription(MotorPower, '/controller/motors_pow', self.move_callback, 10)

        # Publishers
        self.encoder_left_pub_ = self.create_publisher(Float32, '/controller/encoder_left', 10)
        self.encoder_right_pub_ = self.create_publisher(Float32, '/controller/encoder_right', 10)

        # Timers
        #self.encoders_tim_ = self.create_timer(self.timer_period_, self.encoders_callback)


    def set_motor_vel(self):
        """
        Assigns the corresponding power to each motor taken from the motors_pow_ array.
        """
        motor1, motor2, motor3, motor4 = self.motors_pow_
        send_message(self.serial_port, f"ML,{(motor1)}")
        send_message(self.serial_port, f"MR,{(motor2)}")
        send_message(self.serial_port, f"M3,{(motor3)}")
        send_message(self.serial_port, f"M4,{(motor4)}")




    def move_callback(self, motor_pow):
        """
        Stores the motor power data in motors_pow_ and assigns each motor power.

        Args:
            motor_pow (MotorPower): The message received by the subscriber, containing motor power data.
        """

        if len(motor_pow.motor_power) == 4:
            self.get_logger().info('Received motor powers: %s' % str(motor_pow.motor_power.tolist()))
            self.motors_pow_ = motor_pow.motor_power
            self.get_logger().info('Motor powers: %s' % str(self.motors_pow_.tolist()))
        else:
            self.get_logger().warn('Invalid data. Expected 4 values, got %d.' % len(motor_pow.motor_power))


        self.set_motor_vel()

    # def encoders_callback(self):
        
    #     encoder_left = 0.0
    # #     #encoder_right = 0.0
    #     self.get_logger().info('Hola')
    #     encoder_left = float(read_message(self.serial_port))
    #     self.get_logger().info('Hola')

    # #     #elif message == "e_R":
    # #     #    encoder_right = float(read_message())

    # #     encoder_left_msg = Float32()
    # #     #encoder_right_msg = Float32()

    # #     encoder_left_msg.data = encoder_left
    # #     #encoder_right_msg.data = encoder_right

    # #     self.encoder_left_pub_.publish(encoder_left_msg)
    # #     #self.encoder_right_pub_.publish(encoder_right_msg)

    # #     self.get_logger().info(f'Encoder left: {encoder_left_msg.data[0]:.2f}')




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