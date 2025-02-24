import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

min_stop_distance = 40
max_stop_distance = 20

class LidarSubscriberNode(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.callback_lidar, 10)
        self.lidar_pub_ = self.create_publisher(Bool, "/lidar", 10)
        self.get_logger().info("Waiting for lidar data")

    def callback_lidar(self, scan):
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        angles = angles[:len(scan.ranges)]  

        obstacle_detected = False 

        for angle, distance in zip(angles, scan.ranges):
            if not np.isfinite(distance): 
                continue

            distance_cm = distance * 100
            angle_deg = np.degrees(angle) + 180

            if distance_cm < max_stop_distance:
                continue

            elif distance_cm < min_stop_distance:
                obstacle_detected = True
                self.get_logger().error(f"Obstacle detected at {angle_deg:.2f}° - Distance: {distance_cm:.2f} cm")
                break

        msg = Bool()
        msg.data = not obstacle_detected  
        self.lidar_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
