import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from time import time
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import String
from profiler_msgs.action import Waypoint
from profiler_msgs.action import Profile



MAX_DURATION_SECONDS = 600
SPIN_FREQUENCY = 100

state = "standby"
maxDepth = 20 #ideal dive depth in meters, may want to tie it to individual waypoints
waypoints = [] #stack of waypoints push new ones to back pop old ones from front

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('StateMachineNode')
        ###################
        # Publishers ######
        ###################
        self.kill_pub_ = self.create_publisher(Int16, "/KILL", 10)
        self.state_pub_ = self.create_publisher(String, "State", 10)

        ##################
        # Subscribers ####
        ##################
        self.command_sub_ = self.create_subscription(String,"/Command",self.command_callback,10)
        self.command_sub_  # prevent unused variable warning
        self.health_sub_ = self.create_subscription(String,"/Health",self.health_callback,10)
        self.health_sub_  # prevent unused variable warning

        ##################
        # Actions ########
        ##################
        self.waypoint_client_ = ActionClient(self, Waypoint, 'waypoint')
        self.profile_client_ = ActionClient(self, Profile, 'profile')

        ###################
        # Timers
        ##################
        self.spin_timer_ = self.create_timer(1/SPIN_FREQUENCY, self.Spin)

        self.start_time_ = time()
        self.cur_time_ = self.start_time_

        if(len(waypoints)>0):
                state = "waypoint" #once arduino sends message that it is ready standby complete
                self.state_pub_.publish(String(data=state))
                coords = {5.0,8.4}
                self.send_waypoint(coords)

    #Parse command from base station 
    def command_callback(self, msg):
        splitmsg = msg.split(':')
        if(splitmsg[0]=="W"):
            splitcoords = splitmsg.split(',')
            waypoints.append(splitcoords[0],splitcoords[1])
        elif(splitmsg[0]=="K"):
            self.kill_pub_.publish(Int16(data=1)) #kill command
        elif(splitmsg[0]=="S"):
            if(len(waypoints)>0):
                state = "waypoint" #once arduino sends message that it is ready standby complete
                self.state_pub_.publish(String(data=state))
                coords = [waypoints[0][0],waypoints[0][1]]
                self.send_waypoint(coords)

    def health_callback(self,msg):
        if(msg=="leak"): #if leak suspected stop all actions and return to surface
            self.cancel_waypoint()
            self.cancel_profile()
            state = "return"
            self.state_pub_.publish(String(data=state))

    def cancel_waypoint(self):
        self.waypoint_result = self.waypoint_client_.cancel_goal_async()
        self.waypoint_result.add_done_callback(self.waypoint_cancel_response)
    def waypoint_cancel_response(self,future):
        cancel_handle = future.result()
        if not cancel_handle.OK:
            self.cancel_waypoint()

    def cancel_profile(self):
        self.profile_result = self.profile_client_.cancel_goal_async()
        self.profile_result.add_done_callback(self.profile_cancel_response)
    def profile_cancel_response(self,future):
        cancel_handle = future.result()
        if not cancel_handle.OK:
            self.cancel_profile()
        
    #Waypoint functions
    def send_waypoint(self, coords):
        goal_msg = Waypoint.Goal()
        goal_msg.waypoint_coords = coords #Geopoint

        self.waypoint_client_.wait_for_server()

        self.waypoint_result = self.waypoint_client_.send_goal_async(goal_msg)

        self.waypoint_result.add_done_callback(self.waypoint_goal_response)

    def waypoint_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.waypoint_result_future = goal_handle.get_result_async()
        self.waypoint_result_future.add_done_callback(self.waypoint_result_callback)

    def waypoint_result_callback(self, future):
        result = future.result().result
        if(result): #made it to waypoint 
            waypoints.pop(0)
            state="profiling"
            self.state_pub_.publish(String(data=state))
            self.send_profile(maxDepth)
    #profile functions
    def send_profile(self, depth):
        goal_msg = Waypoint.Goal()
        goal_msg.desiredDepth = depth

        self.profile_client_.wait_for_server()

        self.profile_result = self.profile_client_.send_goal_async(goal_msg)

        self.profile_result.add_done_callback(self.profile_goal_response)

    def profile_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.profile_result_future = goal_handle.get_result_async()
        self.profile_result_future.add_done_callback(self.profile_result_callback)

    def profile_result_callback(self, future):
        result = future.result().result
        if(result<1):#if on surface
            if(len(waypoints)>0):
                    state = "waypoint" #once arduino sends message that it is ready standby complete
                    self.state_pub_.publish(String(data=state))
                    coords = self.CreatePoint([waypoints[0][0],waypoints[0][1],0.0])
                    self.send_waypoint(coords)
            else:
                state="return"
                self.state_pub_.publish(String(data=state))

    def Spin(self):
        self.UpdateTime()
        #if (self.ShouldDie()):
        #    self.kill_pub_.publish(Int16(data=1))
        #else:
        #    self.kill_pub_.publish(Int16(data=0))

    def UpdateTime(self):
        self.cur_time_ = time()
        

    def ShouldDie(self):
        return self.cur_time_ - self.start_time_ > MAX_DURATION_SECONDS
            
    def CreatePoint(self, data):
        lat, long, heading = data[0], data[1], data[2]
        dec_lat, dec_long = self.LatLongDegreesToDecimal(lat, long)
        return GeoPoint(latitude=dec_lat, longitude=dec_long, altitude=heading)
        
def main(args=None):
    rclpy.init()
    rclpy.spin(StateMachineNode())
    rclpy.shutdown()

if __name__ == '__main __':
    main()