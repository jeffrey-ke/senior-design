import rclpy
from rclpy.node import Node
from HardwareBridge import HardwareBridge

class HBNode(Node):

    bridge_ = HardwareBridge()

    def __init__():
        super().__init__("HBNode")
        

        # timer to spin bridge


        # timer to read from bridge and publish

        # subscribers that feed the bridge

        
        # Publishers
        