import rclpy
from rclpy.node import Node

from profiler_msgs.srv import GetDepth, GetGnss, GetOrientation, SendKill, SendPwm

from .HardwareBridge import HardwareBridge

class HBNode(Node):
    def __init__(self):
        super().__init__("hardware_bridge_node")
        self.create_service(GetDepth, 'get_depth', self.RequestDepth)
        self.create_service(GetGnss, 'get_gnss', self.RequestGnss)
        self.create_service(GetOrientation, 'get_orientation', self.RequestOrientation)
        self.create_service(SendKill, 'send_kill', self.SendKill)
        self.create_service(SendPwm, 'send_pwm', self.SendPwm)

        self.bridge_ = HardwareBridge("ACM0", 9600, 5)

    def SendPwm(self, request, response):
        FL, FR, DL, DR = request.forward_l_pwm, request.forward_r_pwm, request.down_l_pwm, request.down_r_pwm
        result = self.bridge_.SendPWM(FL, FR, DL, DR)
        response.success = True if result is not None else False
        return response

    def SendKill(self, request, response):
        pass

    def RequestOrientation(self, request, response):
        from geometry_msgs.msg import Quaternion
        result = self.bridge_.AskForIMU()
        success = True
        if result is None:
            result = Quaternion()
            success = False
        response.orientation = result
        response.success = success
        return response


    def RequestGnss(self, request, response):
        from geographic_msgs.msg import GeoPoint
        result = self.bridge_.AskForGps()
        success = True

        if result is None:
            result = GeoPoint()
            success = False

        response.gnss = result
        response.success = success
        return response


    def RequestDepth(self, request, response):
        pass
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HBNode())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
        