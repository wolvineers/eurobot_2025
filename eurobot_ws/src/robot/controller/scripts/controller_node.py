import rclpy
from rclpy.node import Node
from utils.msg import MotorPower


## *************************
## CONTROLLER CLASS
## *************************

class ControllerNode(Node):
    
    def __init__(self):

        # Attributes
        ...

        # Publishers
        ...

        # Subscribers
        ...


    def move_straight():
        ...

    def move_turn():
        ...

    def pose_callback():
        ...



## *************************
## MAIN CODE
## *************************

def main(args=None):
    ...

if __name__ == "__main__":
    main()