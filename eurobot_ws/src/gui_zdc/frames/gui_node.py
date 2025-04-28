import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.action_publisher = self.create_publisher(Int32, '/i_action', 10)
        self.start_publisher = self.create_publisher(Bool, '/start', 10)
        self.points_subscriber = self.create_subscription(Int32, '/points', self.points_callback, 10)
        self.points = 0  # Store the received points

    def initialize_robot(self, state):
        msg = Int32()
        msg.data = state  # Send state (0 or 1)
        self.action_publisher.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')
    
    def points_callback(self, msg):
        self.points = msg.data
        self.get_logger().info(f'Received points: {self.points}')
    
    def get_points(self):
        return self.points
    
    def start_robot(self, state):
        msg = Bool()
        msg.data = state
        self.start_publisher.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')


gui_node = None


def init_gui_node():
    global gui_node
    rclpy.init(args=None)  # Initialize ROS
    gui_node = GUINode()  # Create the ROS node instance
    gui_node.get_logger().info("ROS node initialized successfully!")
    rclpy.spin(gui_node)  # Start spinning to listen for messages
    gui_node.destroy_node()
    rclpy.shutdown()

def get_node():
    return gui_node

def test():
    print("Hello")
