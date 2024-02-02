import rclpy

from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from .NavigationNode import NavigationNode
from profiler_msgs.msg import Pwm

# 37.429518,-121.982590 are the of a point approximately 10 meters out from the dock 

SPIN_FREQUENCY = 10 # Hz


class VelocityCommanderNode(Node):
    def __init__(self):

        ##################
        # Subscribers ###
        #################
        self.create_subscription(GeoPoint, "/gps", self.UpdateLatLon, 10)  
        self.create_subscription(GeoPoint, "/waypoint", self.UpdateWaypoint, 10)

        

        #################
        # Publishers ####
        #################
        self.pwm_pub_ = self.create_publisher(Pwm, "/pwm", 10)


        #################
        # Timers ########
        #################
        self.spin_timer_ = self.create_timer(1/SPIN_FREQUENCY, self.Spin)


        ################
        # Workers ######
        ################
        self.navigator_ = NavigationNode()


    def Spin(self):
        pwm = self.navigator_.waypointToPwm(self.lat_, self.lon_,
                                            self.wp_lat_, self.wp_lon_,
                                            self.heading_)
        self.pwm_pub_.publish(Pwm(forward_l_pwm=pwm[0],
                                  forward_r_pwm=pwm[1],
                                  down_l_pwm=pwm[2],
                                  down_r_pwm=pwm[3]))

    def UpdateWaypoint(self, msg):
        self.wp_lat_, self.wp_lon_ = msg.latitude, msg.longitude

    def UpdateLatLonHeading(self, msg):
        self.lat_, self.lon_, self.heading_ = msg.latitude, msg.longitude, msg.altitude

def main(args=None):
    rclpy.init()
    rclpy.spin(VelocityCommanderNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()