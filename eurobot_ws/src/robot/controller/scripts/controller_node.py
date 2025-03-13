#!/usr/bin/env python3

import rclpy, time

from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Vector3, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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

        self.num_action_   = 0
        self.state_action_ = 0

        # Publishers
        self.velocities_pub_ = self.create_publisher(Twist, '/controller/motors_pow', 10)
        self.end_order_pub_  = self.create_publisher(Bool, '/controller/end_order', 10)
        self.action_pub_     = self.create_publisher(JointTrajectory, '/controller/action_commands', 10)


        # Subscribers
        self.encoder_left_sub_  = self.create_subscription(Float32, '/controller/encoder_left', self.encoder_left_callback, 10)
        self.encoder_right_sub_ = self.create_subscription(Float32, '/controller/encoder_right', self.encoder_right_callback, 10)
        self.movement_sub_      = self.create_subscription(Vector3, '/movement', self.movement_callback, 10)
        self.t_action_sub_      = self.create_subscription(Int32, '/t_action', self.t_action_callback, 10)
        self.i_action_sub_      = self.create_subscription(Int32, '/i_action', self.i_action_callback, 10)

        # Timers
        self.controller_tim_ = self.create_timer(self.timer_period_, self.control_velocities)
        self.action_tim_     = self.create_timer(1.0, self.control_actuators)

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

    
    def t_action_callback(self, num_action):
        """
        Gets the Torete action number, stores them in the num_action_ variable and initializes state_action_.

        Args:
            num_action (Int32): The message received by the subscriber, containing the action number.
        """

        ''' --- TORETE ACTIONS LIST ---
        *   
        * Action 01: Init actuators
        * Action 02: Pick up material
        * Action 03: Build tribune
        *
        '''

        self.num_action_   = num_action.data
        self.state_action_ = 0

    
    def i_action_callback(self, num_action):
        """
        Gets the Insomnious action number and sends all the orders to execute it to the ESP32.

        Args:
            num_action (Int32): The message received by the subscriber, containing the action number.
        """
        
        ''' --- INSOMNIOUS ACTIONS LIST ---
        *   
        * Action 01: Init actuators
        *
        '''

        ...


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

        #self.get_logger().info("Velocity left: " + str(self.vel_left) + " | Velocity right: " + str(self.vel_right))
        
        self.velocities_pub_.publish(motor_vel_msg)   


    def control_actuators(self):
        """
        Every second gets the Torete action number and publishes the servos positions depending on the action_state_ or the end of the action.
        """

        ''' --- SERVOS POSITIONS MESSAGE ---
        *   
        * 
        *
        '''

        action_msg = JointTrajectory()
        point      = JointTrajectoryPoint()

        if self.num_action_ == 1:
            ...

        elif self.num_action_ == 2: # Pick up material

            if self.state_action_ == 0: # Raise platforms
                point.positions = [140.0, 45.0, 140.0]

                action_msg.joint_names = ["S04", "S05", "S06"]
                action_msg.points.append(point)

                self.action_pub_.publish(action_msg)

                self.state_action_ += 1
                self.get_logger().info("Raising platforms")
            
            elif self.state_action_ == 1: # Take columns
                point.positions = [30.0, 30.0]

                action_msg.joint_names = ["S07", "S08"]
                action_msg.points.append(point)

                self.action_pub_.publish(action_msg)

                self.state_action_ += 1
                self.get_logger().info("Taking columns")

            else:   # End action
                self.num_action_ = 0

                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)
                self.get_logger().info("Action 2 (pick up material) ended")

        elif self.num_action_ == 3: # Build tribune
            
            if self.state_action_ == 0: # Leave columns
                point.positions = [150.0, 125.0]

                action_msg.joint_names = ["S07", "S08"]
                action_msg.points.append(point)

                self.action_pub_.publish(action_msg)

                self.state_action_ += 1
                self.get_logger().info("Leaving columns")
            
            elif self.state_action_ == 1: # Leave platform
                point.positions = [20.0, 160.0, 20.0]

                action_msg.joint_names = ["S04", "S05", "S06"]
                action_msg.points.append(point)

                self.action_pub_.publish(action_msg)

                self.state_action_ += 1
                self.get_logger().info("Leaving platforms")

            else:   # End action
                self.num_action_ = 0

                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)
                self.get_logger().info("Action 3 (build tribune) ended")


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