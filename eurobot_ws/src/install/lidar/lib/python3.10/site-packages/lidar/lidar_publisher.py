
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rplidar import RPLidar

# Distancias de advertencia
CAUTION_DIST = 500
STOP_DIST = 300

# Puerto del LiDAR
PORT = "/dev/ttyUSB0"

class LidarNode(Node):
    def __init__(self):
        super().__init__("lidar_node")

        # Crear el publicador
        self.publisher_ = self.create_publisher(Bool, "lidar", 10)

        # Crear el temporizador para publicar cada 0.5 segundos
        self.timer_ = self.create_timer(0.5, self.publish_message)

        # Inicializar el LiDAR
        try:
            self.lidar = RPLidar(PORT)
            self.get_logger().info("Lidar started successfully!")
        except Exception as e:
            self.get_logger().error(f"Error connecting to LiDAR: {e}")
            self.lidar = None

    def publish_message(self):
        self.get_logger().info("Dins!")
        msg = Bool()
        msg.data = self.read_lidar()
        self.publisher_.publish(msg)

    def read_lidar(self):
        self.get_logger().info("Reading!")
        try:
            variable = True
            self.get_logger().info("prova!")
            for scan in self.lidar.iter_scans():
                #variable = False
                self.get_logger().info("1r for!")
            #     for scan_quality, angle, distance in scan: 
            #         self.get_logger().info("2n for!")
            #         if distance < STOP_DIST:
            #             return False
            #         else:
            #             return True
            return True
        except Exception as e:
            self.get_logger().error(f"Error {e}")
            return True




    def destroy(self):
        if self.lidar:
            self.lidar.stop()
            self.lidar.disconnect()
            self.get_logger().info("Lidar disconnected")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()