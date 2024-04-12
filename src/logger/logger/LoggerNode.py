import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from .Logger import Logger

class LoggerNode(Node):
    def __init__(self):
        super().__init__("Logger node")
        self.waypoint_state_log_sub_ = self.create_subscription(String, 
                                                                "/waypoint_state_log", 
                                                                self.Log, 
                                                                10)
        self.logger_ = Logger()
        
    def Log(self, msg: String):
        self.logger_.GiveLine(msg.data)

def main(args=None):
    rclpy.init()
    rclpy.spin(LoggerNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()