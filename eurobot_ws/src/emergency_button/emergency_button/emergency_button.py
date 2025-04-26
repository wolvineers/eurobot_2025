import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

BUTTON_PIN = 40  

class EmergencyButtonNode(Node):
    def __init__(self):
        super().__init__('emergency_button_node')
        self.publisher_ = self.create_publisher(String, 'emergency_button', 10)
        self.timer = self.create_timer(0.05, self.check_button_state)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.button_prev_state = GPIO.HIGH  

        self.get_logger().info('Emergency Button Node started.')

    def check_button_state(self):
        button_state = GPIO.input(BUTTON_PIN)
        if button_state == GPIO.LOW and self.button_prev_state == GPIO.HIGH:
            self.get_logger().info('Button pressed!')
            msg = String()
            msg.data = 'Emergency button pressed!'
            self.publisher_.publish(msg)

        self.button_prev_state = button_state

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyButtonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()