#!/usr/bin/env python3

import rclpy, time

from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Vector3, Twist
from differential_drive import DifferentialWheel
from msgs.msg import JointActionPoint
from robot.controller.scripts.insomnius_actions import handle_action

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
        
        self.timer_period_ = 0.05

        self.num_action_t_ = 0
        self.num_order_i = 0
        self.state_action_ = 0
        self.end_action_   = False

        self.dist_accel_    = 7.0
        self.dist_desaccel_ = 10.0
        self.init_vel_      = 5.0
        self.final_vel_     = 15.0
        self.linear_vel_    = 0.0
        self.angular_vel_   = 0.0
        self.distance_      = 0.0
        self.direction_     = 1

        # Publishers
        self.velocities_pub_ = self.create_publisher(Twist, '/controller/motors_pow', 10)
        self.end_order_pub_  = self.create_publisher(Bool, '/controller/end_order', 10)
        self.action_pub_     = self.create_publisher(JointActionPoint, '/controller/action_commands', 10)

        # Subscribers
        self.encoder_left_sub_  = self.create_subscription(Float32, '/controller/encoder_left', self.encoder_left_callback, 10)
        self.encoder_right_sub_ = self.create_subscription(Float32, '/controller/encoder_right', self.encoder_right_callback, 10)
        self.movement_sub_      = self.create_subscription(Vector3, '/movement', self.movement_callback, 10)
        self.t_action_sub_      = self.create_subscription(Int32, '/t_action', self.t_action_callback, 10)
        self.i_action_sub_      = self.create_subscription(Int32, '/i_action', self.i_action_callback, 10)
        self.opponent_detected  = self.create_subscription(Bool, '/lidar', self.opponent_detected_callback, 10)
        self.end_action_sub_    = self.create_subscription(Bool, '/controller/end_action', self.end_action_callback, 10)

        # Timers
        self.controller_tim_ = self.create_timer(self.timer_period_, self.control_velocities)
        self.action_tim_t_   = self.create_timer(1.0, self.control_actuators_t)
        self.action_tim_i_   = self.create_timer(1.0, self.control_actuators_i)

        # Differential drive object
        self.robot = DifferentialWheel(l=0.25, radius=0.0475)

    
    def encoder_left_callback(self, encoder_left):
        """
        Gets the value of encoder_left and save it to encoder_left_.

        Args:
            encoder_left (Float32): The message received by the subscriber, containing encoder data.
        """

        self.encoder_left_ = encoder_left.data
        # self.get_logger().info("Encoder left value: " + str(self.encoder_left_))
    

    def encoder_right_callback(self, encoder_right):
        """
        Gets the value of encoder_right and save it to encoder_right_.

        Args:
            encoder_right (Float32): The message received by the subscriber, containing encoder data.
        """

        self.encoder_right_ = encoder_right.data
        # self.get_logger().info("Encoder right value: " + str(self.encoder_right_))
    

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

        # self.robot_movements.append((movement.x, movement.y, movement.z))
        # self.get_logger().info("New movement added")

        self.linear_vel_  = abs(movement.x)
        self.angular_vel_ = abs(movement.y)
        self.distance_    = movement.z

        if movement.x < 0 or movement.y < 0:
            self.direction_ = -1

        #self.vel_left, self.vel_right = self.robot.get_motor_velocities(self.linear_vel, self.angular_vel)

        self.get_logger().info("Movement added")

    
    def t_action_callback(self, num_action):
        """
        Gets the Torete action number, stores them in the num_action_t_ variable and initializes state_action_.

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

        self.num_action_t_ = num_action.data
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

        self.get_logger().info("Order received")

        self.num_order_i = num_action.data
        self.state_action_ = 1
        

    def opponent_detected_callback(self, opponent):
        """
        Gets the bool data of the leader if an opponent is too close.

        Args:
            opponent (Bool): The message recived by the subscriber, containing a Bool data.

            *
            * True : Continue forward
            * False: STOP
            *
        """

        self.opponent_detected = opponent.data

    
    def end_action_callback(self, end_action):
        """
        When the action before has finished so it gets true, add one to the state action.

        Args:
            end_order (Bool): The message received by the subscriber, containing the state of the last action.
        """

        self.end_action_ = not end_action.data
        self.state_action_ += 1

        self.get_logger().info("Action ended. New action: " + str(self.state_action_))


    def control_velocities(self):
        """
        At each elapsed time publishes the power of each motor for the current movement to the respective topic.
        """

        ## --- TO DO ---
        ##  - Change the robot_movement queue to an action server
        ##  - Add the lidar detection in the first condition
        ##  - Remove the delay to avoid shaking and control it in the basic_routine.py

        vel_left = 0
        vel_right = 0

        if self.distance_ != 0 and self.opponent_detected:

            distance_moved    = (abs(self.encoder_left_) + abs(self.encoder_right_)) / 2
            self.get_logger().info("Distance moved: " + str(distance_moved))

            straight_distance = self.distance_ - self.dist_accel_ - self.dist_desaccel_

            # Acceleration
            if distance_moved < self.dist_accel_:
                # self.get_logger().info("Dins d'acceleracio | linear vel, init vel, dist_accel: " + str(self.linear_vel_) + ", " + str(self.init_vel_) + ", " + str(self.dist_accel_))
                # self.get_logger().info("Res operacio: " + str(self.linear_vel_ - self.init_vel_))

                n_accel   = self.init_vel_
                m_accel_v = (self.linear_vel_ - self.init_vel_) / self.dist_accel_
                m_accel_w = (self.angular_vel_ - self.init_vel_) / self.dist_accel_

                v = ((m_accel_v * distance_moved) + n_accel) * self.direction_
                w = ((m_accel_w * distance_moved) + n_accel) * self.direction_

                vel_left, vel_right = self.robot.get_motor_velocities(v, w)

                # self.get_logger().info("Dins d'acceleracio | m, vl, vr: " + str(m_accel_v) + ", " + str(vel_left) + ", " + str(vel_right))

            # Straight
            elif distance_moved < straight_distance:
                # self.get_logger().info("Dins de straight")
                vel_left, vel_right = self.robot.get_motor_velocities(self.linear_vel_ * self.direction_, self.angular_vel_ * self.direction_)

            # Desacceleration
            elif distance_moved < self.distance_:
                # self.get_logger().info("Dins de desaccel | m, linear vel, init vel, dist_desaccel: " + str(self.linear_vel_) + ", " + str(self.init_vel_) + ", " + str(self.dist_accel_))
                # self.get_logger().info("Res operacio: " + str(self.linear_vel_ - self.init_vel_))

                m_desaccel_v = (self.final_vel_ - self.linear_vel_) / (self.distance_ - straight_distance)
                n_desaccel_v = self.final_vel_ - m_desaccel_v * self.distance_
                m_desaccel_w = (self.final_vel_ - self.angular_vel_) / (self.distance_ - straight_distance)
                n_desaccel_w = self.final_vel_ - m_desaccel_w * self.distance_

                if self.linear_vel_ != 0:
                    v = (m_desaccel_v * distance_moved + n_desaccel_v) * self.direction_
                else:
                    v = 0

                if self.angular_vel_ != 0:
                    w = (m_desaccel_w * distance_moved + n_desaccel_w) * self.direction_
                else:
                    w = 0

                # self.get_logger().info("Dins d'acceleracio | m, vl, vr: " + str(m_desaccel_v) + ", " + str(v))

                vel_left, vel_right = self.robot.get_motor_velocities(v, w)

            elif distance_moved >= self.distance_:
                self.get_logger().info("Parant")
                # If the time has finished set powers to 0.0
                vel_left  = 0.0
                vel_right = 0.0
                self.distance_ = 0
                self.direction_ = 1

                # Publish end of movement
                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)

        # Publish the message with each motor power
        motor_vel_msg = Twist()

        motor_vel_msg.linear.y = float(vel_left)
        motor_vel_msg.linear.z = float(vel_right)

        #self.get_logger().info("Velocity left: " + str(vel_left) + " | Velocity right: " + str(vel_right))
        
        self.velocities_pub_.publish(motor_vel_msg)   


    def control_actuators_t(self):
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

        if self.num_action_t_ == 1:
            ...

        elif self.num_action_t_ == 2: # Pick up material

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
                self.num_action_t_ = 0

                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)
                self.get_logger().info("Action 2 (pick up material) ended")

        elif self.num_action_t_ == 3: # Build tribune
            
            if self.state_action_ == 0: # Leave columns
                point.positions = [170.0, 155.0]

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
                self.num_action_t_ = 0

                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)
                self.get_logger().info("Action 3 (build tribune) ended")

    
    def control_actuators_i(self):
        """
        Every second gets the Insomnius action number and publishes the action to do.
        """

        ''' --- ACTUATORS MESSAGE ---
        * Joint Action  
        *   - header (header) --> description of the action
        *   - names (string []) --> list of the names of the action
        *   - points (JointActionPoint []) --> list the points of the action
        *
        * Joint Action Point
        *   - position (int32 [])  --> servos values (each element of the array corresponds to each servo)
        *   - velocity (float32 []) --> motors values (the same as the servos)
        *   - activate (bool) --> air pump

        * PROPOSTA  
        *   - servos_names (string [])
        *   - motors_names (string [])
        *
        *   - position (int32 [])
        *   - velocity (float32 [])
        *   - activate (bool)
        '''

        self.get_logger().info("Dins control actuators --> Num order: " + str(self.num_order_i) + " | End action:" + str(self.end_action_))

        if self.num_order_i > 0 and not self.end_action_:
            self.end_action_ = True

            action_msg = JointActionPoint()
            action_msg = handle_action(self.num_order_i, self.state_action_)

            self.get_logger().info("Servos: " + str(action_msg.servos_names))
            self.get_logger().info("Motors: " + str(action_msg.motors_names))

            if action_msg.servos_names or action_msg.motors_names:
                # Missatge ple per publicar
                self.get_logger().info("Publishing message")
                self.action_pub_.publish(action_msg)

            else:
                # Missatge buit i per tant ordre acabada
                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)

                self.num_order_i = 0
                self.end_action_ = False
                
                self.get_logger().info("Order " + str(self.num_order_i) + " ended")



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