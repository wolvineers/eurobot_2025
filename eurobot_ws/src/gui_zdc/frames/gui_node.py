import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Int8MultiArray

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')

        # Publishers
        self.action_publisher = self.create_publisher(Int32, '/i_action', 10)
        self.start_publisher = self.create_publisher(Bool, '/start', 10)

        # Subscribers
        self.points_subscriber = self.create_subscription(Int32, '/points', self.points_callback, 10)
        self.sensor_subscriber = self.create_subscription(Int8MultiArray, '/sensor_status', self.sensor_callback, 10)
        self.button_subscriber = self.create_subscription(Int8MultiArray, '/button_status', self.button_callback, 10)

        # Internal state
        self.points = 0
        self.sensor_states = [0] * 5  # Default to 5 sensors
        self.button_states = [0] * 2  # Default to 2 buttons

    # --- Callbacks ---
    def points_callback(self, msg):
        self.points = msg.data
        self.get_logger().info(f'Received points: {self.points}')

    def sensor_callback(self, msg):
        self.sensor_states = msg.data[:5]
        self.get_logger().info(f'Sensor states updated: {self.sensor_states}')

    def button_callback(self, msg):
        self.button_states = msg.data[:2]
        self.get_logger().info(f'Button states updated: {self.button_states}')

    # --- Accessors ---
    def get_points(self):
        return self.points

    def get_sensor_states(self):
        return {
            f"Sensors{i+1}": self.sensor_states[i] if i < len(self.sensor_states) else 0
            for i in range(5)
        }

    def get_button_states(self):
        return {
            f"Button{i+1}": self.button_states[i] if i < len(self.button_states) else 0
            for i in range(2)
        }

    # --- Publishers ---
    def initialize_robot(self, state):
        msg = Int32()
        msg.data = state
        self.action_publisher.publish(msg)
        self.get_logger().info(f'Published /i_action: {msg.data}')
    
    def start_robot(self, state):
        msg = Bool()
        msg.data = state
        self.start_publisher.publish(msg)
        self.get_logger().info(f'Published /start: {msg.data}')


# --- Singleton pattern ---
gui_node = None

def init_gui_node():
    global gui_node
    rclpy.init(args=None)
    gui_node = GUINode()
    gui_node.get_logger().info("ROS node initialized successfully!")
    rclpy.spin(gui_node)
    gui_node.destroy_node()
    rclpy.shutdown()

def get_node():
    return gui_node

def test():
    print("Hello")
