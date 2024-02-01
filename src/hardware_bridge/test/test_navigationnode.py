from hardware_bridge import NavigationNode
from hardware_bridge.NavigationNode import waypointToVelocity
from hardware_bridge.NavigationNode import velocityToPWM

def test_navigationnode():
    temp = waypointToVelocity((3,6,0),(7,6,0),5.786)
    print(temp[0])
    print(temp[5])
    assert temp[5]!=0
    assert temp[0]==0
    pwm = velocityToPWM(temp)
    print(pwm[0])
    print(pwm[1])
    assert pwm[0]==0


