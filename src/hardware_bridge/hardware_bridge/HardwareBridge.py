from serial import Serial
import time
from .MessageCreator import MessageCreator
 

class HardwareBridge:
    def __init__(self, port, baud, timeout, logger) -> None:
        self.serial_ = Serial('/dev/tty{}'.format(port), baud, timeout=timeout)
        self.port_ = port
        self.timeout_ = timeout
        self.baud_ = baud
        self.init_successful_ = self.serial_.is_open
        self.serial_.reset_input_buffer()
        self.msg_creator_ = MessageCreator()
        self.logger_ = logger

    def TryInit(self):
        self.serial_ = Serial('/dev/tty{}'.format(self.port_), self.baud_, timeout=self.timeout_)
        self.init_successful_ = self.serial_.is_open

    def AskForStatus(self, ros_msg_type, timeout=1):
        self._Send("S:")
        msg = self._ReadWithCheck(timeout=timeout)
        created_msg = self._CreateMessageOrNone(msg)
        return created_msg if isinstance(created_msg, ros_msg_type) else None

    def WaitForInit(self):
        line = "S:0"
        while(line!="S:1"):
            self._Send("S:")
            line = self.serial_.readline().decode('utf-8').rstrip()
            time.sleep(1)
            print(line)
        self.serial_.reset_input_buffer()

    def AskForGps(self, ros_msg_type, timeout=1):
        self._Send("G:")
        msg = self._ReadWithCheck(timeout=timeout)
        self.logger_.info("\n\n\n\n\t==============THE __CHECKED__ MESSAGE:================")
        self.logger_.info(msg)
        created_msg = self._CreateMessageOrNone(msg)
        return created_msg if isinstance(created_msg, ros_msg_type) else None
        
        
    def AskForIMU(self, ros_msg_type, timeout=1):
        self._Send("I:")
        msg = self._ReadWithCheck(timeout=timeout)
        created_msg = self._CreateMessageOrNone(msg)
        return created_msg if isinstance(created_msg, ros_msg_type) else None


    def SendPWM(self, FL, FR, DL, DR, timeout=1):
        self._Send("T:{},{},{},{}".format(FL, FR, DL, DR))
        msg = self._ReadWithCheck(timeout=timeout)
        return msg is not None

    def _CreateMessageOrNone(self, msg):
        if msg is None:
            return None
        else:
            return self.msg_creator_.CreateMessage(msg)


    def _ReadWithCheck(self, timeout):
        # if self.serial_.in_waiting > 0:
        self.logger_.info("\n\n\n\n\t==============THE READ MESSAGE:================")
        msg = self.serial_.readline().decode('utf-8').rstrip()
        self.logger_.info(msg)
        return msg
        

    def _Send(self, msg):
        msg = msg + "\n"
        self.serial_.write(msg.encode("utf-8"))
"""
G:
I:
T:1,2,3,4
"""