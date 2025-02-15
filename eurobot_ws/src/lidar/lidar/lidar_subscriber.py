import rclpy
import numpy as np
import time

from rclpy.node import Node
from sensor_msgs.msg import LaserScan

min_stop_distance = 60
max_stop_distance = 30

class LidarSubscriberNode(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.callback_lidar, 10)
        self.get_logger().info("Waiting for lidar data")

    def callback_lidar(self, scan):
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        angles = angles[:len(scan.ranges)]


        for angle, distance in zip(angles, scan.ranges):
            distance = distance * 100
            angle = np.degrees(angle) + 180
            if distance < max_stop_distance:
                continue
                
            elif distance < min_stop_distance and distance > max_stop_distance:
                self.get_logger().error("False")
                return False
                

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()