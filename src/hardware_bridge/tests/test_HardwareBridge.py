from hardware_bridge.HardwareBridge import HardwareBridge

def test():
    hb = HardwareBridge("ACM0", 115200, test=True)
    hb.SendMasterCommand("imu")
    assert False
    