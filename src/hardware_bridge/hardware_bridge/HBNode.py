import rclpy
from rclpy.node import Node

from profiler_msgs.srv import GetDepth, GetGnss, GetOrientation, SendKill, SendPwm, GetMinionStatus

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
        self.bridge_.Command("status {}".format(str(request.timeout))) 
        msg = self.bridge_.Response()
        if msg is None:
            response.success = False
            return response
        else:
            response = msg
            return response

    def SendPwm(self, request, response):
        FL, FR, DL, DR = request.forward_l_pwm, request.forward_r_pwm, request.down_l_pwm, request.down_r_pwm
        self.get_logger().info("Pwm sent:\n\t{}\n\t{}\n\t{}\n\t{}".format(FL, FR, DL, DR))
        self.bridge_.Command("pwm {} {} {} {}".format(str(FL), str(FR), str(DL), str(DR)))
        return response

    def SendKill(self, request, response):
        pass

    def RequestOrientation(self, request, response):
        from geometry_msgs.msg import Quaternion
        result = self.bridge_.AskForIMU(ros_msg_type=Quaternion)
        success = True
        if result is None:
            result = Quaternion()
            success = False
        self.get_logger().info("Orientation received: {}, {}, {}, {}".format(result.w, result.x, result.y, result.z))
        response.orientation = result
        response.success = success
        return response


    def RequestGnss(self, request, response):
        from geographic_msgs.msg import GeoPoint
        result = self.bridge_.AskForGps(GeoPoint)
        success = True

        if result is None:
            result = GeoPoint()
            success = False
        self.get_logger().info("GNSS: {}, {}, {}".format(result.latitude, result.longitude, result.altitude))
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
        
