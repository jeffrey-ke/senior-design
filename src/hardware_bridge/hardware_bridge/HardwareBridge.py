from serial import Serial
from collections import deque
from multiprocessing import Lock

class HardwareBridge:

    incoming_msg_q_ = deque()
    outgoing_msg_q_ = deque()
    serial_ = None


    def __init__(self, port, baud, timeout, incoming_buf_lock=Lock(), outgoing_buf_lock=Lock()) -> None:
        self.serial_ = Serial('/dev/tty{}'.format(port), baud, timeout=timeout)
        self.serial_.reset_input_buffer()
        self.incoming_buf_lock_ = incoming_buf_lock
        self.outgoing_buf_lock_ = outgoing_buf_lock

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
        with self.outgoing_buf_lock_:
            return len(self.outgoing_msg_q_) > 0

    def IsIncomingMessageAvailable(self) -> bool:
        with self.incoming_buf_lock_:    
            return len(self.incoming_msg_q_) > 0

    def QueueIncoming(self, msg):
        with self.incoming_buf_lock_:
            self.incoming_msg_q_.append(msg)
        
    def QueueOutgoing(self, msg):
        with self.outgoing_buf_lock_:
            self.outgoing_msg_q_.append(msg)

    def DequeueIncoming(self) -> str:
        with self.incoming_buf_lock_:
            return self.incoming_msg_q_.popleft()
    
    def DequeueOutgoing(self) -> str:
        with self.outgoing_buf_lock_:
            return self.outgoing_msg_q_.popleft()