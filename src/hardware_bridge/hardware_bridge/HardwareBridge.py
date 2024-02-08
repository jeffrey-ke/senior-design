from serial import Serial
from time import time
from .MessageCreator import MessageCreator
 

class HardwareBridge:
    def __init__(self, port, baud, timeout) -> None:
        self.serial_ = Serial('/dev/tty{}'.format(port), baud, timeout=timeout)
        self.port_ = port
        self.timeout_ = timeout
        self.baud_ = baud
        self.init_successful_ = self.serial_.is_open
        self.serial_.reset_input_buffer()
        self.timer_ = Timer()
        self.msg_creator_ = MessageCreator()

    def TryInit(self):
        self.serial_ = Serial('/dev/tty{}'.format(self.port_), self.baud_, timeout=self.timeout_)
        self.init_successful_ = self.serial_.is_open

    def AskForGps(self, timeout=1):
        self._Send("G:")
        msg = self._ReadWithCheck(timeout=timeout)
        return self._CreateMessageOrNone(msg)
        
        
    def AskForIMU(self, timeout=1):
        self._Send("I:")
        msg = self._ReadWithCheck(timeout=timeout)
        return self._CreateMessageOrNone(msg)

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
        self.timer_.Start()
        while (self.timer_.TimeElapsed() < timeout):
            if self.serial_.in_waiting > 0:
                msg = self.serial_.readline()
                return msg if msg[-1] == '\n' else None
        return None
        

    def _Send(self, msg):
        msg = msg + "\n"
        self.serial_.write(msg.encode("utf-8"))
"""
G:
I:
T:1,2,3,4

"""

class Timer():
    from time import time
    def __init__(self) -> None:
        pass

    def Start(self):
        self.start = time()

    def TimeElapsed(self):
        return time() - self.start