from serial import Serial
from tokens import *
from time import time



class HardwareBridge:

    def __init__(self, port, baud, timeout=1, test=False) -> None:
        if not test:
            self.serial_ = Serial('/dev/tty{}'.format(port), baud, timeout=timeout)
            self.init_successful_ = self.serial_.is_open
            self.serial_.reset_input_buffer()
        self.port_ = port
        self.timeout_ = timeout
        self.baud_ = baud
        self.token_bag_ = None
        self.msg_wait_duration_ = 0.0

        self.warning_expr_ = WARNING

    def Match(self, tok):
        if tok != self.token_bag_.Lookahead():
                raise Exception("Unexpected token for expression in parse. See Hardwarebridge.py:Match")
        self.token_bag_.NextToken()
    
    def Integer(self):
        try:
            int_val = int(self.token_bag_.Lookahead())
            self.token_bag_.NextToken()
            return int_val
        except ValueError:
            raise Exception("Tried to match an integer value, but an integer wasn\'t there.")
    
    def Float(self):
        try:
            float_val = float(self.token_bag_.Lookahead())
            self.token_bag_.NextToken()
            return float_val
        except ValueError:
            raise Exception("Tried to match a float value, but a float wasn\'t there.")
        

    def Command(self, master_command: str):
        """
            Command -> Command_Imu
                    | Command_Gnss
                    | Command_Pwm
                    | Command_Depth
                    | Command_Kill
                    | Command_Status
            
            Command_Imu -> imu int
            Command_Gnss -> gnss int
            Command_Pwm -> pwm int int int int
            Command_Depth -> depth int    
            Command_Kill -> kill  
            Command_Status -> status
        
        """
        self.token_bag_ = TokenBag(master_command.split())
        tok = self.token_bag_.Lookahead()
        if (tok == IMU):
            self.Command_Imu()
        elif (tok == GNSS):
            self.Command_Gnss()
        elif (tok == DEPTH):
            self.Command_Depth()
        elif (tok == PWM):
            self.Command_Pwm()
        elif (tok == KILL):
            self.Command_Kill()
        elif (tok == STATUS):
            self.Command_Status()
        
    def Response(self):
        """
            Response -> Response_Imu
                     |  Response_Gnss
                     |  Response_Depth
                     |  Response_Warning
                     |  Response_Status

            Response_Imu -> imu float float float
            Response_Gnss -> gnss float float float
            Response_Depth -> depth float
            Response_Warning -> warning
            Response_Status -> status int
        
        """
        self.token_bag_ = TokenBag(self.GetMinionResponse().split())
        tok = self.token_bag_.Lookahead()
        if (tok == IMU):
            return self.Response_Imu()
        elif (tok == GNSS):
            return self.Response_Gnss()
        elif (tok == DEPTH):
            return self.Response_Depth()
        elif (tok == WARNING):
            return self.Response_Warning()
        elif (tok == STATUS):
            return self.Response_Status()
    
    def Response_Imu(self):
        from profiler_msgs.msg import Imu
        self.Match(IMU)
        x = self.Float()
        y = self.Float()
        z = self.Float()
        msg = Imu()
        msg.x = x
        msg.y = y
        msg.z = z
        return msg
    
    def Response_Gnss(self):
        from profiler_msgs.msg import Gnss
        self.Match(gnss)
        lat = self.Float()
        lon = self.Float()
        heading = self.Float()
        msg = Gnss()
        msg.lat = lat
        msg.lon = lon
        msg.heading = heading
        return msg

    def Response_Depth(self):
        from std_msgs.msg import Float32
        self.Match(DEPTH)
        depth = self.Float()    
        return Float32(depth)
    
    def Response_Warning(self):
        self.Match(WARNING)
        return None
    
    def Response_Status(self):
        from std_msgs.msg import Bool
        self.Match(STATUS)
        alive = True if self.Integer() is not 0 else False
        return Bool(alive)
                
    def GetMinionResponse(self) -> str:
        start = time()
        while (time() - start < self.msg_wait_duration_):
            if (self.serial_.in_waiting > 0):
                return self.serial_.readline().decode()
        return self.warning_expr_
        
    def Command_Imu(self):
        self.SerialSend("I:\n")
        self.Match(IMU)
        self.msg_wait_duration_ = self.Integer()
        pass

    def Command_Gnss(self):
        self.SerialSend("G:\n")
        self.Match(GNSS)
        self.msg_wait_duration_ = self.Integer()
        pass

    def Command_Depth(self):
        self.SerialSend("D:\n")
        self.Match(DEPTH)
        self.msg_wait_duration_ = self.Integer()
        pass

    def Command_Pwm(self):
        self.SerialSend("P:")
        self.Match(PWM)
        for i in range(4):
            pwm = self.Integer()
            self.SerialSend(str(pwm))
            self.SerialSend(",")
        self.SerialSend("\n")
    
    def Command_Kill(self):
        self.SerialSend("Kill\n")
        self.Match(KILL)

    def Command_Status(self):
        self.SerialSend("S:\n")
        self.Match(STATUS)
    
    def SerialSend(self, msg: str):
        # self.serial_.write(msg.encode())
        print(msg, end="")

class TokenBag:
    def __init__(self, tokens_arr: list[str]) -> None:
        self.buf_ = tokens_arr
        self.i_ = 0
        
    def Lookahead(self):
        if self.i_ >= len(self.buf_):
            return None
        tok = self.buf_[self.i_]
        return tok
    
    def NextToken(self):
        self.i_ = self.i_ + 1

