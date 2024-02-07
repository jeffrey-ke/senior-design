import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from .NavigationNode import NavigationNode
from profiler_msgs.msg import Pwm
from profiler_msgs.action import Waypoint
from profiler_msgs.action import Profile

# 37.429518,-121.982590 are the of a point approximately 10 meters out from the dock 

class VelocityCommanderNode(Node):
    lat_ = 0.0
    wp_lat_ = 0.0
    lon_ = 0.0
    wp_lon_ = 0.0
    heading_ = 0.0
    depth = 0.0

    def __init__(self):
        super().__init__('VelocityCommanderNode') #Important!
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

        ################
        # Workers ######
        ################
        self.navigator_ = NavigationNode()

    def gps_callback(self,msg):
        self.lat_ = msg.latitude
        self.lon_ = msg.longitude

    def waypoint_callback(self, goal_handle):
        self.get_logger().debug('traveling to waypoint...')
        coords = goal_handle.request.waypoint_coords 
        wp_lat_ = coords.latitude
        wp_lon_ = coords.longitude
        pwm = [1,1,1,1] #explicit type definition
        while(pwm[0]!=0 and pwm[1]!=0): #while not at waypoint 
            #update gnss
            pwm = self.navigator_.waypointToPwm(self.lat_, self.lon_,
                                                self.wp_lat_, self.wp_lon_,
                                                self.heading_)
            self.pwm_pub_.publish(Pwm(forward_l_pwm=pwm[0],
                                    forward_r_pwm=pwm[1],
                                    down_l_pwm=pwm[2],
                                    down_r_pwm=pwm[3]))
            feedback_msg = Waypoint.Feedback()
            feedback_msg.distance_to_waypoint = self.navigator_.getDistanceToWaypoint(self.lat_, self.lon_, self.wp_lat_, self.wp_lon_)
            goal_handle.publish_feedback(feedback_msg)

            #testing
            self.lat_ += (self.wp_lat_-self.lat_)/5
            self.lon_ += (self.wp_lon_-self.lon_)/5

        goal_handle.succeed()
        result = Waypoint.Result()
        result.arrived_at_waypoint = True
        return result
    
    def profile_callback(self, goal_handle):
        self.get_logger().debug('profiling water column...')
        desiredDepth = goal_handle.request.desired_depth
        self.htovFlip() #flip orientation
        pwm = [1,1,1,1] #explicit type definition
        while(self.depth>=desiredDepth): #or close to sea floor
            #update depth
            pwm = self.navigator_.descendToPwm(self.depth, desiredDepth, self.heading_)

            self.pwm_pub_.publish(Pwm(forward_l_pwm=pwm[0],
                                    forward_r_pwm=pwm[1],
                                    down_l_pwm=pwm[2],
                                    down_r_pwm=pwm[3]))
            self.depth += 0.5  #testing               

        while(self.depth<1.0):
            #update depth
            #get and log sensor data
            pwm = self.navigator_.ascendToPwm(self.depth, desiredDepth, self.heading_)
            self.pwm_pub_.publish(Pwm(forward_l_pwm=pwm[0],
                                    forward_r_pwm=pwm[1],
                                    down_l_pwm=pwm[2],
                                    down_r_pwm=pwm[3]))
            self.depth -= 0.5   #testing

        self.vtohFlip() #flip orientation back
        goal_handle.succeed()
        result = Profile.Result()
        result.ending_depth = self.depth
        return result
        

    def htovFlip(self): #script to flip profiler horizontal to vertical
        self.get_logger().debug('gamer flip')


    def vtohFlip(self): #script to flip profiler vertical to horizontal
        self.get_logger().debug('gamer flip back')

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