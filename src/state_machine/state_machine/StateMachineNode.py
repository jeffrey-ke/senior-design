import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from time import time
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Int16
from std_msgs.msg import String
from Waypoint.action import Waypoint


MAX_DURATION_SECONDS = 600
SPIN_FREQUENCY = 100

lat = 0.0
lon = 0.0
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
        self.subscription  # prevent unused variable warning

        ##################
        # Actions ########
        ##################
        self.waypoint_client_ = ActionClient(self, Waypoint, 'waypoint')

        ###################
        # Timers
        ##################
        self.spin_timer_ = self.create_timer(1/SPIN_FREQUENCY, self.Spin)

        self.start_time_ = time()
        self.cur_time_ = self.start_time_

    #Parse command from base station 
    def command_callback(self, msg):
        splitmsg = msg.split(':')
        if(splitmsg[0]=="W"):
            splitcoords = splitmsg.split(',')
            waypoints.append(splitcoords[0],splitcoords[1])
        elif(splitmsg[0]=="K"):
            self.kill_pub_.publish(1) #kill command

    def send_waypoint(self, coords):
        goal_msg = Waypoint.Goal()
        goal_msg.waypointCoords = coords

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
        if(result):
            waypoints.pop(0)
            

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