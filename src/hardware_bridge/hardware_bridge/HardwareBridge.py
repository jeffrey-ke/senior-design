from serial import Serial
from collections import deque
class HardwareBridge:

    incoming_msg_q_ = deque()
    outgoing_msg_q_ = deque()
    serial_ = None


    def __init__(self, port, baud, timeout) -> None:
        self.serial_ = Serial('/dev/{}'.format(port), baud, timeout=timeout)
        self.serial_.reset_input_buffer()

    def Spin(self):
        if (self.IsOutgoingMessageAvailable()):
            self.serial_.write(self.DequeueOutgoing())

        if (self.serial_.in_waiting > 0):
            self.QueueIncoming(self.serial_.readline().decode('utf-8').rstrip())      

    def Send(self, msg):
        self.QueueOutgoing(msg)
    
    def Read(self):
        return self.DequeueIncoming() if self.IsIncomingMessageAvailable() else None
        
    def IsOutgoingMessageAvailable(self) -> bool:
        return len(self.outgoing_msg_q_) > 0

    def IsIncomingMessageAvailable(self) -> bool:
        return len(self.incoming_msg_q_) > 0

    def QueueIncoming(self, msg):
        self.incoming_msg_q_.append(msg)
        
    def QueueOutgoing(self, msg):
        self.outgoing_msg_q_.append(msg)

    def DequeueIncoming(self) -> str:
        return self.incoming_msg_q_.popleft()
    
    def DequeueOutgoing(self) -> str:
        return self.outgoing_msg_q_.popleft()