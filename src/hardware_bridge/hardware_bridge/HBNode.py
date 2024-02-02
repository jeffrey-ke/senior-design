import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from .HardwareBridge import HardwareBridge
from .MessageCreator import MessageCreator
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPoint


BRIDGE_SPIN_FREQUENCY = 10 # Hz, make this a ros2 param
BRIDGE_READ_FREQUENCY = 10
MSG_CR_SPIN_FREQUENCY = 10
MSG_CR_READ_FREQUENCY = 10


class HBNode(Node):

    

    bridge_callback_g_ = ReentrantCallbackGroup()
    msg_cr_callback_g_ = ReentrantCallbackGroup()


    def __init__(self):
        super().__init__("HBNode")

        
        if (self.InitAll() is False):
            self.get_logger().info("INIT FAILED! SHUTTING DOWN")
            self.Shutdown()
    
        # timer to spin bridge
        self.bridge_spin_timer_ = self.create_timer(1/BRIDGE_SPIN_FREQUENCY, 
                                                    self.BridgeSpin, 
                                                    callback_group=self.bridge_callback_g_)

        # timer to read from bridge and publish
        self.bridge_read_timer_ = self.create_timer(1/BRIDGE_READ_FREQUENCY, 
                                                    self.BridgeReadCb, 
                                                    callback_group=self.bridge_callback_g_)
        
        self.msg_cr_spin_timer_ = self.create_timer(1/MSG_CR_SPIN_FREQUENCY,
                                                    self.MsgCrSpin,
                                                    callback_group=self.msg_cr_callback_g_)
         
        self.msg_cr_read_timer_ = self.create_timer(1/MSG_CR_READ_FREQUENCY,
                                                    self.ReadFromMessageCreatorAndPublish, 
                                                    callback_group=self.msg_cr_callback_g_)
        
        self.raw_GPS_pub_ = self.create_publisher(GeoPoint, "/gps", 10)
        self.raw_IMU_pub_ = self.create_publisher(Quaternion, "/imu", 10)
        # subscribers that feed the bridge

    def BridgeSpin(self):
        self.get_logger().info("\tBridge spinning..")
        self.bridge_.Spin()

    def MsgCrSpin(self):
        self.get_logger().info("\tMsg Cr Spinning..")
        self.msg_creator_.Spin()

    def Shutdown(self):
        rclpy.shutdown()

    def InitAll(self) -> bool:
        bridge_init = self.InitBridge()
        msg_cr_init = self.InitMsgCr()
        print("Bridge init: " + str(bridge_init))
        print("Message Creator init: " + str(msg_cr_init))

        return (bridge_init is True) and (msg_cr_init is True)


    def InitBridge(self) -> bool:
        self.bridge_ = HardwareBridge("ACM0", 115200, 1) #Add port and baud and timeout
        return self.bridge_.init_successful_

    def InitMsgCr(self) -> bool:
        self.msg_creator_ = MessageCreator()
        return True
        
    def BridgeReadCb(self):
        raw = self.bridge_.Read()
        self.msg_creator_.EnqueueRaw(raw)
        self.get_logger().info("msg: " + str(raw))
        
    # Publishers
    def ReadFromMessageCreatorAndPublish(self):
        msg = self.msg_creator_.Read()
        if (msg is None):
            return
        if isinstance(msg, GeoPoint):
            self.raw_GPS_pub_.publish(msg)
        elif isinstance(msg, Quaternion):
            self.raw_IMU_pub_.publish(msg)

def main(args=None):
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(HBNode())
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        