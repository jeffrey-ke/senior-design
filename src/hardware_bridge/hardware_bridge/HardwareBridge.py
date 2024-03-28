from serial import Serial
from tokens import *

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

    def Match(self, tok):
        try:
            int_val = int(tok)
            tok_val = int(self.token_bag_.Lookahead())
            # If they're both int, then they automatically match
        except ValueError:
            if tok != self.token_bag_.Lookahead():
                raise Exception("Unexpected token for expression in parse. See Hardwarebridge.py:Match")
            
        self.token_bag_.NextToken()
    
    def Integer(self):
        try:
            int_val = int(self.token_bag_.Lookahead())
            self.Match(INT)
            return int_val
        except ValueError:
            raise Exception("Tried to match an integer value, but an integer wasn\'t there.")

    def SendMasterCommand(self, master_command: str):
        """
            Command -> Imu
                    | Gnss
                    | Pwm
                    | Depth
            
            Imu -> imu
            Gnss -> gnss
            Pwm -> pwm int int int int
            Depth -> depth      
        
        """
        self.token_bag_ = TokenBag(master_command.split())
        tok = self.token_bag_.Lookahead()
        if (tok == IMU):
            self.Imu()
        elif (tok == GNSS):
            self.Gnss()
        elif (tok == DEPTH):
            self.Depth()
        elif (tok == PWM):
            self.Pwm()
        
        
    def Imu(self):
        self.SerialSend("I:\n")
        self.Match(IMU)
        pass

    def Gnss(self):
        self.SerialSend("G:\n")
        self.Match(GNSS)
        pass

    def Depth(self):
        self.SerialSend("D:\n")
        self.Match(DEPTH)
        pass

    def Pwm(self):
        self.SerialSend("P:")
        self.Match(PWM)
        for i in range(4):
            pwm = self.Integer()
            self.SerialSend(str(pwm))
            self.SerialSend(",")
        self.SerialSend("\n")
    
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

