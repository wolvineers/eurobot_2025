#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from msgs.msg import JointActionPoint

from utils.scripts.serial_communication import open_serial_port, send_message, read_message


## ***********************
## FIRST DRIVER CLASS
## ***********************

class FirstDriverNode(Node):
    
    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('first_driver')


        # Attributes
        self.port        = '/dev/ttyUSB1'
        self.baudrate    = 115200
        self.serial_port = open_serial_port(self.port, self.baudrate)

        self.timer_period_ = 0.05

        # Subscribers
        self.motors_pow_sub_      = self.create_subscription(Twist, '/controller/motors_pow', self.move_callback, 10)
        self.action_commands_sub_ = self.create_subscription(JointActionPoint, '/controller/action_commands', self.actions_commands_callback, 10)

        # Publishers
        self.encoder_left_pub_  = self.create_publisher(Float32, '/controller/encoder_left', 10)
        self.encoder_right_pub_ = self.create_publisher(Float32, '/controller/encoder_right', 10)
        self.end_action_pub_ = self.create_publisher(Bool, "controller/end_action", 10)

        # Timers
        # self.encoders_tim_ = self.create_timer(self.timer_period_, self.encoders_timer)
        self.message_tim_ = self.create_timer(self.timer_period_, self.message_timer_)


    def move_callback(self, motor_pow):
        """
        Gets the values of motor_pow and send them to the Esp32.

        Args:
            motor_pow (Twist): The message received by the subscriber, containing motor power data.
                * Twist message params:
                *   linear.y --> velocity left motor
                *   linear.z --> velocity right motor
        """

        motors_pow_01 = motor_pow.linear.y
        motors_pow_02 = motor_pow.linear.z

        send_message(self.serial_port, f"ML,{(motors_pow_01)}")
        send_message(self.serial_port, f"MR,{(motors_pow_02)}")

    
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

        # Motors message
        motors_msg = ""

        for i, motor in enumerate(action_commands.motors_names):
            motors_msg += f"{motor},"
            motors_msg += f"{action_commands.velocity[i]},"

        motors_msg = motors_msg[:-1]

        send_message(self.serial_port, motors_msg)


    def message_timer(self):
        """
        Reads the message sended by Esp32 containing the encoders value and publish this data.
        """
        
        received_msg = read_message(self.serial_port)
        
        if received_msg != None:
            # Separates the message into the three parts marked by commas and saves the values
            msg_parts   = received_msg.split(",")
            msg_element = msg_parts[0]
            msg_value   = float(msg_parts[1])

            if msg_element == "EA":
                # Publish the end action message
                end_msg = Bool()
                end_msg.data = True

                self.end_action_pub_.publish(end_msg)
            else:
                # Prepare and publish the encoders message
                self.get_logger().info('Encoder: ' + str(msg_value))
                encoder_msg      = Float32()
                encoder_msg.data = msg_value

                if msg_element == "EL":
                    self.encoder_left_pub_.publish(encoder_msg)
                elif msg_element == "ER":
                    if encoder_msg.data != 0:
                        encoder_msg.data *= -1
                    self.encoder_right_pub_.publish(encoder_msg)


    # def encoders_timer(self):
    #     """
    #     Reads the message sended by Esp32 containing the encoders value and publish this data.
    #     """
        
    #     encoder_msg = read_message(self.serial_port)
        
    #     if encoder_msg != None:
    #         # Separates the message into the three parts marked by commas and saves the values
    #         encoder_parts  = encoder_msg.split(",")
    #         encoder        = encoder_parts[0]
    #         encoder_value  = float(encoder_parts[1])

    #         # Prepare and publish the message
    #         self.get_logger().info('Encoder: ' + str(encoder_value))
    #         encoder_msg      = Float32()
    #         encoder_msg.data = encoder_value

    #         if encoder == "EL":
    #             self.encoder_left_pub_.publish(encoder_msg)
    #         elif encoder == "ER":
    #             if encoder_msg.data != 0:
    #                 encoder_msg.data *= -1
    #             self.encoder_right_pub_.publish(encoder_msg)


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