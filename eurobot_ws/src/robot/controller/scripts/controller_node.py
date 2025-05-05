#!/usr/bin/env python3

import rclpy, math

from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Vector3, Twist
from differential_drive import DifferentialWheel
from msgs.msg import JointActionPoint
from robot.controller.scripts.insomnius_actions import handle_action

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


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

        self.num_action_t_  = 0
        self.num_order_i    = 0
        self.state_action_  = 0
        self.end_action_01_ = True
        self.end_action_02_ = True

        self.dist_accel_    = 7.0
        self.dist_desaccel_ = 10.0
        self.init_vel_      = 0.2
        self.final_vel_     = 0.1
        self.linear_vel_    = 0.0
        self.angular_vel_   = 0.0
        self.distance_      = 0.0
        self.time_mov_      = 0.0
        self.start_time_    = 0.0
        self.angle_goal_    = 0.0
        self.direction_     = 1
        self.const_corretion_ = 0.02

        self.trajectory_x_ = []
        self.trajectory_y_ = []

        self.imu_ = 0.0


        # Publishers
        self.velocities_pub_ = self.create_publisher(Twist, '/controller/motors_pow', 10)
        self.end_order_pub_  = self.create_publisher(Bool, '/controller/end_order', 10)
        self.action_pub_     = self.create_publisher(JointActionPoint, '/controller/action_commands', 10)


        # Subscribers
        self.encoder_left_sub_  = self.create_subscription(Float32, '/controller/encoder_left', self.encoder_left_callback, 10)
        self.encoder_right_sub_ = self.create_subscription(Float32, '/controller/encoder_right', self.encoder_right_callback, 10)
        self.movement_sub_      = self.create_subscription(Vector3, '/movement', self.movement_callback, 10)
        self.movement_tim_sub_  = self.create_subscription(Vector3, '/movement_tim', self.movement_tim_callback, 10)
        self.t_action_sub_      = self.create_subscription(Int32, '/t_action', self.t_action_callback, 10)
        self.i_action_sub_      = self.create_subscription(Int32, '/i_action', self.i_action_callback, 10)
        self.opponent_detected  = self.create_subscription(Bool, '/lidar', self.opponent_detected_callback, 10)
        self.end_action_01_sub_ = self.create_subscription(Bool, '/controller/end_action_01', self.end_action_01_callback, 10)
        self.end_action_02_sub_ = self.create_subscription(Bool, '/controller/end_action_02', self.end_action_02_callback, 10)
        self.imu_sub_           = self.create_subscription(Float32, "/controller/imu", self.imu_callback, 10)

        self.plot_traj_sub_ = self.create_subscription(Bool, '/plot_traj', self.plot_trajectory, 10)


        # Timers
        self.controller_striaght_tim_ = self.create_timer(self.timer_period_, self.control_velocities_straight)
        self.controller_turn_tim_     = self.create_timer(self.timer_period_, self.control_velocities_turn)
        self.controller_time_tim_     = self.create_timer(self.timer_period_, self.control_velocities_tim)

        self.action_tim_t_   = self.create_timer(1.0, self.control_actuators_t)
        self.action_tim_i_   = self.create_timer(1.0, self.control_actuators_i)

        # Differential drive object
        self.robot = DifferentialWheel(l=0.23, radius=0.0475)

    
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
        Gets the values of movement and stores them int the robot_movement queue.

        Args:
            movement (Vector3): The message received by the subscriber, containing movement data.
                * Vector3 message params:
                *   x --> linear velocity
                *   y --> angular velocity
                *   z --> movement distance
        """

        self.linear_vel_  = abs(movement.x)
        self.angular_vel_ = abs(movement.y)

        if self.angular_vel_ == 0.0:
            self.distance_ = movement.z
            self.angle_goal_ = 0.0
        elif self.linear_vel_ == 0.0:
            self.angle_goal_ = movement.z
            self.distance_ = 0.0

        self.time_mov_ = 0.0

        if movement.x < 0 or movement.y < 0:
            self.direction_ = -1


    def movement_tim_callback(self, movement_tim):
        """
        Gets the values of movement timer and stores them int the robot_movement queue.

        Args:
            movement (Vector3): The message received by the subscriber, containing movement data.
                * Vector3 message params:
                *   x --> linear velocity
                *   y --> angular velocity (0)
                *   z --> movement time
        """

        self.get_logger().info("Moviment rebut")

        self.linear_vel_ = abs(movement_tim.x)
        self.time_mov_   = movement_tim.z
        self.start_time_ = self.get_clock().now()

        if movement_tim.x < 0:
            self.direction_ = -1

    
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

        self.get_logger().info("Order received")

        self.num_order_i = num_action.data
        self.state_action_ = 0


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

    
    def imu_callback(self, imu):
        """
        Gets the float data of the imu.

        Args:
            imu (Float): The message recived by the subscriber, containing a Float data.
        """
        
        self.imu_ = imu.data

    
    def end_action_01_callback(self, end_action):
        """
        When the previous first drive action has finished so it gets true, add one to the state action.

        Args:
            end_order (Bool): The message received by the subscriber, containing the state of the last action.
        """

        self.end_action_01_ = end_action.data

        self.get_logger().info("Action ended. New action: " + str(self.state_action_))


    def end_action_02_callback(self, end_action):
        """
        When the previous second drive action has finished so it gets true, add one to the state action.

        Args:
            end_order (Bool): The message received by the subscriber, containing the state of the last action.
        """

        self.end_action_02_ = end_action.data

        self.get_logger().info("Action ended. New action: " + str(self.state_action_))


    def control_velocities_straight(self):
        """
        At each elapsed time publishes the power of each motor for the current straight movement to the respective topic.
        """
        
        vel_left = 0
        vel_right = 0

        if self.distance_ != 0 and self.opponent_detected and self.angular_vel_ == 0.0 and self.time_mov_ == 0.0:
            
            distance_moved = (abs(self.encoder_left_) + abs(self.encoder_right_)) / 2
            
            straight_distance = self.distance_ - self.dist_accel_ - self.dist_desaccel_

            # Acceleration
            if distance_moved < self.dist_accel_:
                n_accel_v = self.init_vel_
                m_accel_v = (self.linear_vel_ - self.init_vel_) / self.dist_accel_

                v = ((m_accel_v * distance_moved) + n_accel_v) * self.direction_
                w = 0.0

                vel_left, vel_right = self.robot.get_motor_velocities(v, w)

            # Straight
            elif distance_moved < straight_distance:
                vel_left, vel_right = self.robot.get_motor_velocities(self.linear_vel_ * self.direction_, 0.0)

            # Desacceleration
            elif distance_moved < self.distance_:
                m_desaccel_v = (self.final_vel_ - self.linear_vel_) / (self.distance_ - straight_distance)
                n_desaccel_v = self.final_vel_ - m_desaccel_v * self.distance_

                v = (m_desaccel_v * distance_moved + n_desaccel_v) * self.direction_

                vel_left, vel_right = self.robot.get_motor_velocities(v, 0.0)

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

            # Adjust difference between encoders
            if distance_moved < self.distance_:
                encoder_left_abs  = abs(self.encoder_left_)
                encoder_right_abs = abs(self.encoder_right_)

                correction_factor = int(abs(encoder_left_abs - encoder_right_abs) / 2) * self.const_corretion_ 

                if abs(self.encoder_left_) < abs(self.encoder_right_):
                    vel_left += (correction_factor * self.direction_)
                elif abs(self.encoder_left_) > abs(self.encoder_right_):
                    vel_right += (correction_factor * self.direction_)  
        
        if self.angular_vel_ == 0.0 and self.time_mov_ == 0.0:
            # Publish the message with each motor power
            motor_vel_msg = Twist()

            motor_vel_msg.linear.y = float(vel_left)
            motor_vel_msg.linear.z = float(vel_right)
        
            self.velocities_pub_.publish(motor_vel_msg)
        
            # Assume that vel_left and vel_right are in m/s -> Convert to rad/s
            wl = vel_left / self.robot.radius
            wr = vel_right / self.robot.radius

            # Update robot state using the computed wheel speeds and time increment
            self.robot.set_theta(math.radians(self.imu_))
            self.robot.update_state(wl, wr, self.timer_period_)

            x, y, _ = self.robot.get_state()
            self.trajectory_x_.append(x)
            self.trajectory_y_.append(y)


    def control_velocities_tim(self):
        """
        At each elapsed time publishes the power of each motor for the current straight movement to the respective topic.
        """
        
        vel_left = 0
        vel_right = 0

        if self.time_mov_ != 0 and self.opponent_detected and self.distance_ == 0.0 and self.angular_vel_ == 0.0:
            
            # Convert time in seconds
            now = self.get_clock().now()
            elapsed_time = (now - self.start_time_).nanoseconds * 1e-9

            if elapsed_time < self.time_mov_:
                vel_left, vel_right = self.robot.get_motor_velocities(self.linear_vel_ * self.direction_, 0.0)

            else:
                self.get_logger().info("Parant")
                # If the time has finished set powers to 0.0
                vel_left         = 0.0
                vel_right        = 0.0
                self.time_mov_   = 0.0
                self.start_time_ = 0.0
                self.direction_  = 1

                # Publish end of movement
                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)

            # Adjust difference between encoders
            if elapsed_time < self.time_mov_:
                encoder_left_abs  = abs(self.encoder_left_)
                encoder_right_abs = abs(self.encoder_right_)

                correction_factor = int(abs(encoder_left_abs - encoder_right_abs) / 2) * self.const_corretion_ 

                if abs(self.encoder_left_) < abs(self.encoder_right_):
                    vel_left += (correction_factor * self.direction_)
                elif abs(self.encoder_left_) > abs(self.encoder_right_):
                    vel_right += (correction_factor * self.direction_)  
        

        if self.distance_ == 0.0 and self.angular_vel_ == 0.0:
            # Publish the message with each motor power
            motor_vel_msg = Twist()

            motor_vel_msg.linear.y = float(vel_left)
            motor_vel_msg.linear.z = float(vel_right)
        
            self.velocities_pub_.publish(motor_vel_msg)
        
            # Assume that vel_left and vel_right are in m/s -> Convert to rad/s
            wl = vel_left / self.robot.radius
            wr = vel_right / self.robot.radius

            # Update robot state using the computed wheel speeds and time increment
            self.robot.set_theta(math.radians(self.imu_))
            self.robot.update_state(wl, wr, self.timer_period_)

            x, y, _ = self.robot.get_state()
            self.trajectory_x_.append(x)
            self.trajectory_y_.append(y)


    def control_velocities_turn(self):
        """
        At each elapsed time publishes the power of each motor for the current turn movement to the respective topic.
        """

        vel_left = 0
        vel_right = 0

        if self.angle_goal_ != 0 and self.opponent_detected and self.linear_vel_ == 0.0 and self.time_mov_ == 0.0:

            self.direction_ = 1 if self.imu_ < self.angle_goal_ else -1

            if self.imu_ < self.angle_goal_ - 2 or self.imu_ > self.angle_goal_ + 2:
                vel_left, vel_right = self.robot.get_motor_velocities(0.0, self.angular_vel_ * self.direction_)

            else:
                self.get_logger().info("Parant")
                # If the time has finished set powers to 0.0
                vel_left  = 0.0
                vel_right = 0.0
                self.angle_goal_ = 0
                self.direction_ = 1

                # Publish end of movement
                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action) 
        
        if self.linear_vel_ == 0.0 and self.time_mov_ == 0.0:
            # Publish the message with each motor power
            motor_vel_msg = Twist()

            motor_vel_msg.linear.y = float(vel_left) * 10.0
            motor_vel_msg.linear.z = float(vel_right) * 10.0
            
            self.velocities_pub_.publish(motor_vel_msg)

            # Assume that vel_left and vel_right are in m/s -> Convert to rad/s
            wl = vel_left / self.robot.radius
            wr = vel_right / self.robot.radius

            # Update robot state using the computed wheel speeds and time increment
            self.robot.set_theta(math.radians(self.imu_))
            self.robot.update_state(wl, wr, self.timer_period_)

            x, y, _ = self.robot.get_state()
            self.trajectory_x_.append(x)
            self.trajectory_y_.append(y)
  
    
    def plot_trajectory(self, msg):
        """
        Generates the robot trajectory when the subscriber receives a message.

        Args:
            msg (Bool): The message received by the subscriber.
        """

        self.get_logger().info("Plotting trajectory... " + str(msg.data))

        plt.plot(self.trajectory_x_, self.trajectory_y_)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Trajectòria ROS 2")
        plt.grid(True)
        plt.axis("equal")
          
        plt.savefig("/wolvi/src/robot/controller/scripts/robot_trajectory.png", dpi=300)
        plt.close()


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
        Every second gets the Insomnius order number and publishes the action to do.
        """

        ''' --- ACTUATORS MESSAGE ---
        *
        * Joint Action Point
        *   - servos_names (string [])
        *   - motors_names (string [])
        *
        *   - position (int32 [])   --> servos values (each element of the array corresponds to each servo name)
        *   - velocity (float32 []) --> motors values (each element of the array corresponds to each motor name)
        *   - activate (bool)       --> air pump
        *   
        '''

        if self.num_order_i > 0 and self.end_action_01_ and self.end_action_02_:
            self.end_action_01_ = False
            self.end_action_02_ = False
            self.state_action_ += 1

            action_msg = JointActionPoint()
            action_msg = handle_action(self.num_order_i, self.state_action_)

            self.get_logger().info("Servos: " + str(action_msg.servos_names))
            self.get_logger().info("Motors: " + str(action_msg.motors_names))

            if action_msg.servos_names or action_msg.motors_names:
                # No empty message
                self.get_logger().info("Publishing message")
                self.action_pub_.publish(action_msg)
                
                # Check if there is any motor to move
                if not action_msg.motors_names:
                    self.end_action_01_ = True

            else:
                # Empty message so end of the order
                end_action = Bool()
                end_action.data = True
                self.end_order_pub_.publish(end_action)

                self.num_order_i = 0
                self.end_action_01_ = True
                self.end_action_02_ = True
                
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