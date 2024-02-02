import rclpy
from rclpy.node import Node
import time
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Int16

MAX_DURATION_SECONDS = 600

class StateMachineNode(Node):
    def __init__(self):

        ###################
        # Publishers ######
        ###################
        self.wp_pub_ = self.create_publisher(GeoPoint, "/waypoint", 10)
        self.kill_pub_ = self.create_publisher(Int16, "/KILL", 10)