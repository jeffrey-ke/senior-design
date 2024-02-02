import rclpy
from rclpy.node import Node
from time import time
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Int16

MAX_DURATION_SECONDS = 600
SPIN_FREQUENCY = 100

lat = 0.0
lon = 0.0

class StateMachineNode(Node):
    def __init__(self):

        ###################
        # Publishers ######
        ###################
        self.wp_pub_ = self.create_publisher(GeoPoint, "/waypoint", 10)
        self.kill_pub_ = self.create_publisher(Int16, "/KILL", 10)

        ###################
        # Timers
        ##################
        self.spin_timer_ = self.create_timer(1/SPIN_FREQUENCY, self.Spin)

        self.start_time_ = time()
        self.cur_time_ = self.start_time_

    def Spin(self):
        self.wp_pub_.publish(GeoPoint())
        self.UpdateTime()
        if (self.ShouldDie()):
            self.kill_pub_.publish(Int16(data=1))
        else:
            self.kill_pub_.publish(Int16(data=0))

    def UpdateTime(self):
        self.cur_time_ = time()
        

    def ShouldDie(self):
        return self.cur_time_ - self.start_time_ > MAX_DURATION_SECONDS
            
        
def main(args=None):
    rclpy.init()
    rclpy.spin(StateMachineNode())
    rclpy.shutdown()

if __name__ == '__main __':
    main()