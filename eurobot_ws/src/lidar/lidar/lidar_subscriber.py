import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from utils.scripts.serial_communication import send_message, open_serial_port

## ***************
## LIDAR CLASS
## ***************

class LidarSubscriberNode(Node):
    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__("lidar_subscriber")

        self.port        = '/dev/ttyUSB0'
        self.baudrate    = 115200
        self.serial_port = open_serial_port(self.port, self.baudrate)

        # Subscriber
        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.callback_lidar, 10)

        # Publisher
        self.lidar_pub_ = self.create_publisher(Bool, "/lidar", 10)
        
        # Attributes (min_stop_distance = distance for close objects - max_stop_distance = distance to avoid detection of the support)
        self.min_stop_distance = 50
        self.max_stop_distance = 20

        # Initial message
        self.get_logger().info("Waiting for lidar data")


    def callback_lidar(self, scan):
        """
        Processes the received LIDAR scan data and determines if there is an obstacle.

        Args:
            scan (LaserScan): The message received by the subscriber, containing LIDAR angle and distance data.
                * scan.ranges --> list of distances measured by the LIDAR
                * scan.angle_min --> minimum scanning angle
                * scan.angle_max --> maximum scanning angle
                * scan.angle_increment --> angle increment between each measurement
        """
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        angles = angles[:len(scan.ranges)]  

        obstacle_detected = False 

        for angle, distance in zip(angles, scan.ranges):
            if not np.isfinite(distance): 
                continue

            distance_cm = distance * 100
            angle_deg = np.degrees(angle) + 180

            if distance_cm < self.max_stop_distance:
                continue

            elif distance_cm < self.min_stop_distance:
                obstacle_detected = True
                self.get_logger().error(f"Obstacle detected at {angle_deg:.2f}° - Distance: {distance_cm:.2f} cm")
                break
        
        """
        Publishes a Boolean message indicating the presence of an obstacle.
        
        True  = No danger (path is clear)
        False = Obstacle detected too close
        """

        
        msg = Bool()
        msg.data = not obstacle_detected  
        self.lidar_pub_.publish(msg)
        send_message(self.serial_port, "{msg.data}")



## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    node = LidarSubscriberNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
