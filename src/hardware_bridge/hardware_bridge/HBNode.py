import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from HardwareBridge import HardwareBridge
from MessageCreator import MessageCreator
from geometry_msgs.msg import Point, Quaternion

BRIDGE_SPIN_FREQUENCY = 10 # Hz, make this a ros2 param
BRIDGE_READ_FREQUENCY = 10
MSG_CR_SPIN_FREQUENCY = 10
MSG_CR_READ_FREQUENCY = 10


class HBNode(Node):

    bridge_ = HardwareBridge("ACM0", 115200, 1) #Add port and baud and timeout
    msg_creator_ = MessageCreator()

    bridge_callback_g_ = ReentrantCallbackGroup()
    msg_cr_callback_g_ = ReentrantCallbackGroup()


    def __init__(self):
        super().__init__("HBNode")
        
        # timer to spin bridge
        self.bridge_spin_timer_ = self.create_timer(1/BRIDGE_SPIN_FREQUENCY, 
                                                    lambda _: self.bridge_.Spin(), 
                                                    callback_group=self.bridge_callback_g_)

        # timer to read from bridge and publish
        self.bridge_read_timer_ = self.create_timer(1/BRIDGE_READ_FREQUENCY, 
                                                    self.BridgeReadCb, 
                                                    callback_group=self.bridge_callback_g_)
        
        self.msg_cr_spin_timer_ = self.create_timer(1/MSG_CR_SPIN_FREQUENCY,
                                                    lambda _: self.msg_creator_.Spin(),
                                                    callback_group=self.msg_cr_callback_g_)
        self.msg_cr_read_timer_ = self.create_timer(1/MSG_CR_READ_FREQUENCY,
                                                    self.ReadFromMessageCreatorAndPublish(), 
                                                    callback_group=self.msg_cr_callback_g_)
        
        self.raw_GPS_pub_ = self.create_publisher(Point, "/raw_gps", 10)
        self.raw_IMU_pub_ = self.create_publisher(Quaternion, "/raw_imu", 10)
        # subscribers that feed the bridge

        
    def BridgeReadCb(self):
        raw = self.bridge_.Read()
        self.msg_creator_.EnqueueRaw(raw)
        self.get_logger().info("msg: " + str(raw))
        

        # Publishers
    def ReadFromMessageCreatorAndPublish(self):
        msg = self.msg_creator_.Read()
        if (msg is None):
            return
        if isinstance(msg, Point):
            self.raw_GPS_pub_.publish(msg)
        elif isinstance(msg, Quaternion):
            self.raw_IMU_pub_.publish(msg)

def main(args=None):
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(HBNode)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        