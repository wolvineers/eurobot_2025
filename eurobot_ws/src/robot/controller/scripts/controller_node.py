#!/usr/bin/env python3

import rclpy, time

from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Twist
from differential_drive import DifferentialWheel


## *************************
## CONTROLLER CLASS
## *************************

class ControllerNode(Node):
    
    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('controller')

        # Attributes
        self.encoder_left_  = 0
        self.encoder_right_ = 0

        self.vel_left     = 0.0
        self.vel_right    = 0.0
        self.mov_time     = 0
        self.elapsed_time = 0.0
        
        self.timer_period_ = 0.1

        self.robot_movements = []

        # Publishers
        self.velocities_pub_ = self.create_publisher(Twist, '/controller/motors_pow', 10)

        # Subscribers
        self.movement_sub_      = self.create_subscription(Vector3, '/movement', self.movement_callback, 10)
        self.encoder_left_sub_  = self.create_subscription(Float32, '/controller/encoder_left', self.encoder_left_callback, 10)
        self.encoder_right_sub_ = self.create_subscription(Float32, '/controller/encoder_right', self.encoder_right_callback, 10)

        # Timers
        self.controller_tim_ = self.create_timer(self.timer_period_, self.control_velocities)

        # Differential drive object
        self.robot = DifferentialWheel(l=0.25, radius=0.0475)

    
    def encoder_left_callback(self, encoder_left):
        """
        Gets the value of encoder_left and save it to encoder_left_.

        Args:
            encoder_left (Float32): The message received by the subscriber, containing encoder data.
        """

        self.encoder_left_ = encoder_left.data
        self.get_logger().info("Encoder left value: " + str(self.encoder_left_))
    

    def encoder_right_callback(self, encoder_right):
        """
        Gets the value of encoder_right and save it to encoder_right_.

        Args:
            encoder_right (Float32): The message received by the subscriber, containing encoder data.
        """

        self.encoder_right_ = encoder_right.data
        self.get_logger().info("Encoder right value: " + str(self.encoder_right_))
    

    def movement_callback(self, movement):
        """
        Gets the values of movement and and stores them int the robot_movement queue.

        Args:
            movement (Vector3): The message received by the subscriber, containing movement data.
                * Vector3 message params:
                *   x --> linear velocity
                *   y --> angular velocity
                *   z --> movement time
        """

        self.robot_movements.append((movement.x, movement.y, movement.z))
        self.get_logger().info("New movement added")


    def control_velocities(self):
        """
        At each elapsed time publishes the power of each motor for the current movement to the respective topic.
        """

        ## --- TO DO ---
        ##  - Change the robot_movement queue to an action server
        ##  - Add the lidar detection in the first condition
        ##  - Remove the delay to avoid shaking and control it in the basic_routine.py

        if self.robot_movements:
            # Check if there are pending movements to be processed

            if self.mov_time == 0:
                # If the last movement has finished, fetch the values for the new movement
                time.sleep(1)  # Delay to avoid robot shaking

                self.mov_time = self.robot_movements[0][2]
                self.vel_left, self.vel_right = self.robot.get_motor_velocities(self.robot_movements[0][0], self.robot_movements[0][1])

            # Update the robot's state
            dt = 0.1
            self.robot.update_state(self.vel_left, self.vel_right, dt)
            self.elapsed_time += dt

            if self.elapsed_time >= self.mov_time:
                # If the time has finished set powers to 0.0
                self.vel_left = 0.0
                self.vel_right = 0.0
                self.mov_time = 0
                self.elapsed_time = 0.0

                # Remove the completed movement from the queue of pending movements
                self.robot_movements.pop(0)

        # Publish the message with each motor power
        motor_vel_msg = Twist()

        motor_vel_msg.linear.y = self.vel_left
        motor_vel_msg.linear.z = self.vel_right

        self.get_logger().info("Velocity left: " + str(self.vel_left) + " | Velocity right: " + str(self.vel_right))
        
        self.velocities_pub_.publish(motor_vel_msg)      


## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()