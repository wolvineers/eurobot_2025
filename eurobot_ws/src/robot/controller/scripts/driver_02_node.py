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
        self.port        = '/dev/ttyUSB2'
        self.baudrate    = 115200
        self.serial_port = open_serial_port(self.port, self.baudrate)

        self.timer_period_ = 0.1

        # Subscribers
        self.action_commands_sub_  = self.create_subscription(JointTrajectory, '/controller/action_commands', self.actions_commands_callback, 10)

        # Publishers
        self.end_motor_action_pub_  = self.create_publisher(Bool, '/controller/motor_action', 10)

        # Timers
        self.motor_action_tim_ = self.create_timer(self.timer_period_, self.motor_timer)


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
            num_actuator = action_commands.joint_names[i]

            if action_commands.points[0].positions:
                val_actuator = action_commands.points[0].positions[i]
            else:
                val_actuator = action_commands.points[0].velocities[i]

            # Send message to the ESP32
            send_message(self.serial_port, f"{num_actuator},{val_actuator}")

    def motor_timer(self):
        """
        Reads the message sended by Esp32 containing the encoders value and publish this data.
        """
        
        # self.get_logger().info("Volta")
        motor_msg = read_message(self.serial_port)
        
        end_msg = Bool()
        end_msg.data = True
        self.end_motor_action_pub_.publish(end_msg)

        # self.get_logger().info(str(motor_msg))
        
        # if motor_msg != None:
        #     self.get_logger().info("Missatge no nul")
        #     # Separates the message into the three parts marked by commas and saves the values
        #     message_parts   = motor_msg.split(",")
        #     actuator        = message_parts[0]
        #     actuator_value  = int(message_parts[1])

        #     # Prepare and publish the message
        #     end_msg = Bool()

        #     if actuator == "EA" and actuator_value == 0:
        #         end_msg.data = False
        #     elif actuator == "EA" and actuator_value == 1:
        #         end_msg.data = True

        #     self.end_motor_action_pub_.publish(end_msg)


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