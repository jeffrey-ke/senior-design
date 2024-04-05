import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from time import time
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from std_msgs.msg import String
from profiler_msgs.srv import GetMinionStatus
from profiler_msgs.action import Waypoint
from profiler_msgs.action import Profile
from .StateMachine import StateMachine



MAX_DURATION_SECONDS = 600
SPIN_FREQUENCY = 100

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('StateMachineNode') #Important!
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

        #################
        # Services ####
        #################
        self.minion_srv_ = self.create_client(GetMinionStatus, 'minion_status')

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

        self.state_machine_ = StateMachine()

        self.state_machine_.pushBackWP(GeoPoint(latitude=37.351812,longitude=121.941060,altitude=20.0))
        # 37.350204,-121.938166
        self.get_logger().info('init succesful')
        self.waitForSetup()
        self.get_logger().info('minion ready')

    #state has just change do something based on new state
    def stateChange(self):
        if(self.state_machine_.state()=="idle"):
            return #do nothing just wait for othercallback

        elif(self.state_machine_.state()=="profile"):
            currentWP = self.state_machine_.popWP()
            self.send_profile(currentWP.altitude)

        elif(self.state_machine_.state()=="waypoint"):
            self.send_waypoint(self.state_machine_.currentWP())

        elif(self.state_machine_.state()=="setup"):
            self.waitForSetup()
        #elif(self.state_machine_.state()=="return"): 
        #elif(self.state_machine_.state()=="estop"): 

        self.state_pub_.publish(String(data=self.state_machine_.state()))


    def get_minion_status(self):
        req = GetMinionStatus.Request()
        self.future = self.minion_srv_.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().alive

    def waitForSetup(self):
        while(self.get_minion_status()==0): #wait for minion to be ready to accept commands
            self.get_logger().info('waiting for minion')
        if(self.state_machine_.assessState()):
            self.stateChange()

        
    #Parse command from base station 
    def command_callback(self, msg):
        splitmsg = msg.split(':')
        if(splitmsg[0]=="W"): #add waypoint command
            splitcoords = splitmsg.split(',')
            state_machine_.pushBackWP(GeoPoint(latitude=splitcoords[0],longitude=splitcoords[1],altitude=0.0))
            if(self.state_machine_.assessState()):
                self.stateChange()
        elif(splitmsg[0]=="K"):
            self.kill_pub_.publish(Int16(data=1)) #kill command
        

    def health_callback(self,msg):
        if(msg=="leak"): #if leak suspected stop all actions and return to surface
            if(self.state_machine_.assessState()):
                self.stateChange()


    #https://robotics.stackexchange.com/questions/104367/actions-in-humble-cancel-action-in-action-client 
    #def cancel_waypoint(self):
    #    self.waypoint_result = self.waypoint_client_._cancel_goal_async()
    #     self.waypoint_result.add_done_callback(self.waypoint_cancel_response)
    #def waypoint_cancel_response(self,future):
    #   cancel_handle = future.result() 
    #    if not cancel_handle.OK:
    #        self.cancel_waypoint()
    #def cancel_profile(self):
    #    self.profile_result = self.profile_client_._cancel_goal_async()
    #    self.profile_result.add_done_callback(self.profile_cancel_response)
    #def profile_cancel_response(self,future):
    #    cancel_handle = future.result()
    #    if not cancel_handle.OK:
    #        self.cancel_profile()
        
    #Waypoint functions
    def send_waypoint(self, coords):
        goal_msg = Waypoint.Goal()
        goal_msg.waypoint_coords = coords #Geopoint
        self.get_logger().info('preparing to send waypoint')
        self.waypoint_client_.wait_for_server()

        self.waypoint_result = self.waypoint_client_.send_goal_async(goal_msg,
         feedback_callback=self.waypoint_feedback)

        self.waypoint_result.add_done_callback(self.waypoint_goal_response)
        self.get_logger().info('waypoint sent')
    def waypoint_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('waypoint accepted :)')

        self.waypoint_result_future = goal_handle.get_result_async()
        self.waypoint_result_future.add_done_callback(self.waypoint_result_callback)
    def waypoint_feedback(self, feedback):
        pass
        #self.get_logger().info('Distance to Waypoint: {0}'.format(feedback.feedback.distance_to_waypoint))

    def waypoint_result_callback(self, future):
        result = future.result().result.arrived_at_waypoint
        if(result): #made it to waypoint 
            if(self.state_machine_.assessState()):
                self.stateChange()

    #profile functions
    def send_profile(self, depth):
        goal_msg = Profile.Goal()
        goal_msg.desired_depth = depth

        self.profile_client_.wait_for_server()

        self.profile_result = self.profile_client_.send_goal_async(goal_msg)

        self.profile_result.add_done_callback(self.profile_goal_response)

    def profile_goal_response(self, future):
        goal_handle = future.result() 
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('profile accepted :)')

        self.profile_result_future = goal_handle.get_result_async()
        self.profile_result_future.add_done_callback(self.profile_result_callback)

    def profile_result_callback(self, future):
        result = future.result().result.ending_depth
        if(result<1.0):#if on surface
           if(self.state_machine_.assessState()):
                self.stateChange()

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