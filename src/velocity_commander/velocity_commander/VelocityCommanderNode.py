import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from .NavigationNode import NavigationNode
from profiler_msgs.msg import Pwm
from profiler_msgs.action import Waypoint
from profiler_msgs.action import Profile

# 37.429518,-121.982590 are the of a point approximately 10 meters out from the dock 

SPIN_FREQUENCY = 10 # Hz

currentCoords = []

class VelocityCommanderNode(Node):
    def __init__(self):

        ##################
        # Subscribers ###
        #################
        #self.gps_sub_ = self.create_subscription(GeoPoint, "/gps", self.gps_callback, 10)  
        #self.gps_sub_

        #################
        # Publishers ####
        #################
        self.pwm_pub_ = self.create_publisher(Pwm, "/pwm", 10)

        ##################
        # Actions ########
        ##################
        self._action_server = ActionServer(self,Waypoint,'waypoint', self.waypoint_callback)
        self._action_server = ActionServer(self,Profile,'profile', self.profile_callback)

        #################
        # Timers ########
        #################
        self.spin_timer_ = self.create_timer(1/SPIN_FREQUENCY, self.Spin)


        ################
        # Workers ######
        ################
        self.navigator_ = NavigationNode()

    def gps_callback(self,msg):
        currentCoords[0] = GeoPoint[0]
        currentCoords[1] = GeoPoint[1]

    def waypoint_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        coords = goal_handle.request.waypoint_coords 

        feedback_msg = Waypoint.Feedback()
        feedback_msg.distance_to_waypoint = self.navigator_.getDistanceToWaypoint(currentCoords[0], currentCoords[1], coords[0], coords[1])
        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        result = Waypoint.Result()
        result.arrived_at_waypoint = True
        return result
    
    def profile_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        depth = goal_handle.request.desired_depth


        goal_handle.succeed()
        result = Profile.Result()
        result.ending_depth = 0.0
        return result

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