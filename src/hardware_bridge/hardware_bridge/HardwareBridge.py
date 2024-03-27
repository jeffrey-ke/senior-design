from serial import Serial
import time
 

class HardwareBridge:
    def __init__(self, port, baud, timeout) -> None:
        self.serial_ = Serial('/dev/tty{}'.format(port), baud, timeout=timeout)
        self.port_ = port
        self.timeout_ = timeout
        self.baud_ = baud
        self.init_successful_ = self.serial_.is_open
        self.serial_.reset_input_buffer()

    def SendMasterCommand(self, master_command: str):
        """
            Command -> Imu
                    | Gnss
                    | Pwm
                    | Depth
            
            Imu -> imu
            Gnss -> gnss
            Pwm -> pwm
            Depth -> depth      
        
        """
        pass
