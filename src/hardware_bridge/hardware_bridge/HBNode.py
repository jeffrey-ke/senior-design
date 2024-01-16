import rclpy
from rclpy.node import Node
from HardwareBridge import HardwareBridge

BRIDGE_SPIN_FREQUENCY = 10 # Hz


class HBNode(Node):

    bridge_ = HardwareBridge()

    def __init__(self):
        super().__init__("HBNode")
        

        # timer to spin bridge
        self.bridge_timer_ = self.create_timer(1/BRIDGE_SPIN_FREQUENCY, lambda _: self.bridge_.Spin())

        # timer to read from bridge and publish
        
        # subscribers that feed the bridge

        
        # Publishers
        