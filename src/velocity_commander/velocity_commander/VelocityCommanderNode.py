import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Quaternion
from .NavigationNode import NavigationNode
from profiler_msgs.action import Waypoint
from profiler_msgs.action import Profile
from profiler_msgs.srv import GetDepth
from profiler_msgs.srv import GetGnss
from profiler_msgs.srv import GetOrientation
from profiler_msgs.srv import SendPwm

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

        #################
        # Services ####
        #################
        self.depth_srv_ = self.create_client(GetDepth, 'get_depth')
        self.gnss_srv_ = self.create_client(GetGnss, 'get_gnss')
        self.orientation_srv_ = self.create_client(GetOrientation, 'get_orientation')
        self.pwm_srv_ = self.create_client(SendPwm, 'send_pwm')

        ##################
        # Actions ########
        ##################
        self._action_server = ActionServer(self,Waypoint,'waypoint', self.waypoint_callback)
        self._action_server = ActionServer(self,Profile,'profile', self.profile_callback)

        ################
        # Workers ######
        ################
        self.navigator_ = NavigationNode()

    def get_depth(self):
        req = GetDepth.Request()
        self.future = self.depth_srv_.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().depth

    def get_gnss(self):
        req = GetGnss.Request()
        self.future = self.gnss_srv_.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().gnss

    def CreatePoint(self, geo):
        lat, long, heading = geo.latitude, geo.longitude, geo.altitude
        dec_lat, dec_long = self.LatLongDegreesToDecimal(lat, long)
        return GeoPoint(latitude=dec_lat, longitude=dec_long, altitude=heading)
    
    def LatLongDegreesToDecimal(self, lat, long):
        degs_lat = lat // 100 # extracts DD
        minutes_lat = lat % 100 # extracts MM.MMMM

        degs_long = long // 100
        minutes_long = long % 100

        return (degs_lat + minutes_lat / 60, degs_long + minutes_long / 60)

    def get_orientation(self):
        req = GetOrientation.Request()
        self.future = self.orientation_srv_.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        quat =  self.future.result().orientation
        return QuaternionToEuler(quat.q_w,quat.q_x,quat.q_y,quat.q_z)

    def QuaternionToEuler(self, w, x, y, z):
        from transforms3d._gohlketransforms import euler_from_quaternion, eu
        return euler_from_quaternion([w, x, y, z])

    def send_pwm(self,pwm):
        req = SendPwm.Request(forward_l_pwm=pwm[0],
                                    forward_r_pwm=pwm[1],
                                    down_l_pwm=pwm[2],
                                    down_r_pwm=pwm[3])
        self.future = self.pwm_srv_.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().success
    

    def waypoint_callback(self, goal_handle):
        self.get_logger().info('traveling to waypoint...')
        coords = goal_handle.request.waypoint_coords 
        self.wp_lat_ = coords.latitude
        self.wp_lon_ = coords.longitude

        geo = self.get_gnss()
        self.get_logger().info('got some gamer coords')
        self.lat_ = geo.latitude
        self.lon_ = geo.longitude
        self.heading_ = geo.altitude

        while(not self.navigator_.atWaypoint(self.lat_, self.lon_, self.wp_lat_, self.wp_lon_)): #while not at waypoint 
            pwm = self.navigator_.waypointToPwm(self.lat_, self.lon_,
                                                self.wp_lat_, self.wp_lon_,
                                                self.heading_)
            from math import pi
            self.get_logger().info("Bearing {}".format(str(self.navigator_.bearing_)))
            self.get_logger().info("Heading {}".format(str(self.heading_ * 180/pi + 180 + 12.5)))
            self.get_logger().info("PWM FL{} FR{} DL{} DR{}".format(pwm[0], pwm[1], pwm[2], pwm[3]))
            #self.send_pwm(pwm)
            feedback_msg = Waypoint.Feedback()
            feedback_msg.distance_to_waypoint = self.navigator_.getDistanceToWaypoint(self.lat_, self.lon_, self.wp_lat_, self.wp_lon_)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("Distance to waypoint: {}".format(self.navigator_.getDistanceToWaypoint(self.lat_, self.lon_, self.wp_lat_, self.wp_lon_)))
            geo = self.get_gnss()
            self.lat_ = geo.latitude
            self.lon_ = geo.longitude
            self.heading_ = geo.altitude

        goal_handle.succeed()
        result = Waypoint.Result()
        result.arrived_at_waypoint = True
        return result
    
    def profile_callback(self, goal_handle):
        self.get_logger().info('profiling water column...')
        desiredDepth = goal_handle.request.desired_depth
        self.htovFlip() #flip orientation
        self.depth = self.get_depth()
        while(self.depth>=desiredDepth): #or close to sea floor
            #update depth
            pwm = self.navigator_.descendToPwm(self.depth, desiredDepth, self.heading_)
            self.send_pwm(pwm)
            self.depth = self.get_depth()           

        while(self.depth<1.0):
            #get and log sensor data
            pwm = self.navigator_.ascendToPwm(self.depth, desiredDepth, self.heading_)
            self.send_pwm(pwm)
            self.depth = self.get_depth()

        self.vtohFlip() #flip orientation back
        goal_handle.succeed()
        result = Profile.Result()
        result.ending_depth = self.depth
        return result
        

    def htovFlip(self): #script to flip profiler horizontal to vertical
        self.get_logger().info('gamer flip')


    def vtohFlip(self): #script to flip profiler vertical to horizontal
        self.get_logger().info('gamer flip back')

def main(args=None):
    rclpy.init()
    rclpy.spin(VelocityCommanderNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
