import rclpy
from rclpy.node import Node

from profiler_msgs.srv import GetDepth, GetGnss, GetOrientation, SendKill, SendPwm, GetMinionStatus
from tokens import *

from .HardwareBridge import HardwareBridge

class HBNode(Node):
    def __init__(self):
        super().__init__("hardware_bridge_node")
        self.create_service(GetDepth, 'get_depth', self.RequestDepth)
        self.create_service(GetGnss, 'get_gnss', self.RequestGnss)
        self.create_service(GetOrientation, 'get_orientation', self.RequestOrientation)
        self.create_service(SendKill, 'send_kill', self.SendKill)
        self.create_service(SendPwm, 'send_pwm', self.SendPwm)
        self.create_service(GetMinionStatus, 'minion_status', self.GetMinionStatus)

        self.bridge_ = HardwareBridge("ACM0", 115200)

        
            

    def GetMinionStatus(self, request, response):
        self.bridge_.Command("{} {}".format(STATUS, str(request.timeout))) 
        msg = self.bridge_.Response()
        if msg is None:
            response.success = False
        else:
            response.success = True
            response.alive = msg
        return response

    def SendPwm(self, request, response):
        FL, FR, DL, DR = request.forward_l_pwm, request.forward_r_pwm, request.down_l_pwm, request.down_r_pwm
        self.bridge_.Command("{} {} {} {} {}".format(PWM, str(FL), str(FR), str(DL), str(DR)))



        self.get_logger().info("Pwm sent:\n\t{}\n\t{}\n\t{}\n\t{}".format(FL, FR, DL, DR))
        return response

    def SendKill(self, request, response):
        self.bridge_.Command(KILL)
        return response

    def RequestOrientation(self, request, response):
        self.bridge_.Command("{} {}".format(IMU, str(request.timeout)))
        msg = self.bridge_.Response()
        if msg is None:
            response.success = False
        else:
            response.success = True
            response.orientation = msg
        return response


    def RequestGnss(self, request, response):
        self.bridge_.Command("{} {}".format(GNSS, str(request.timeout)))
        msg = self.bridge_.Response()
        if msg is None:
            response.success = False
        else:
            response.success = True
            response.gnss = msg
        return response


    def RequestDepth(self, request, response):
        self.bridge_.Command("{} {}".format(DEPTH, str(request.timeout)))
        msg = self.bridge_.Response()
        if msg is None:
            response.success = False
        else:
            response.success = True
            response.depth = msg
        return response
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HBNode())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
