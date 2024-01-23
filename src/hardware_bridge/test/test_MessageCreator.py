from hardware_bridge.MessageCreator import MessageCreator
from hardware_bridge.MessageCreator import ParserConverter


def test_add_raw():
    mc = MessageCreator()

    assert len(mc.raw_msgs_) == 0

    raw = "G:10,10"
    mc.EnqueueRaw(raw)

    assert len(mc.raw_msgs_) == 1


def test_identify():
    pc = ParserConverter()

    raw = "G:10,10"
    assert pc.Identify(raw) == ("G", "10,10")

    raw = "X:10,10"
    assert pc.Identify(raw) == ("X", "10,10")

def test_parse_convert():
    pc = ParserConverter()        

    raw = "G:10,10"
    assert pc.ParseConvert(raw) == {"type" : "G", 
                                    "data" : 
                                        {
                                            "lat" : 10.0, 
                                            "long" : 10.0
                                        }
                                    }
    raw = "G:69,69"
    assert pc.ParseConvert(raw) == {"type" : "G", 
                                    "data" : 
                                        {
                                            "lat" : 69.0, 
                                            "long" : 69.0
                                        }
                                    }


    raw = "I:10,10,10"
    assert pc.ParseConvert(raw) == {"type": "I",
                                    "data" : 
                                        {
                                            "x" : 10.0,
                                            "y" : 10.0,
                                            "z" : 10.0
                                        }
                                    }
    
"""
    We are converting lat long to cartesian
"""
def test_create_point_msg():
    from math import sin,cos
    from geometry_msgs.msg import Point

    mc = MessageCreator()
    raw = "G:1010.10,1010.10"
    assert mc.CreateMessage(raw) == Point(x=3455636.1568216668, y=3177908.05122314,z=-4317419.456157454)

    raw = "G:2020.20,3030.30"
    assert ArePointsClose(mc.CreateMessage(raw), 
                          Point(
                              x=6.378137e6 * cos(30 + 30.30/60) * cos(20 + 20.20/60), 
                              y=6.378137e6 * sin(30 + 30.30/60) * cos(20 + 20.20/60),
                              z=6.378137e6 * sin(20 + 20.20/60))
                          )

def ArePointsClose(p1, p2, tolerance=1e-2) -> bool:
    from numpy import allclose
    return allclose([p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z], atol=tolerance)
        
def test_create_quat_msg():
    from geometry_msgs.msg import Quaternion

    mc = MessageCreator()
    
    raw = "I:10,20,30"

    assert AreQuatsClose(mc.CreateMessage(raw), 
                         Quaternion(w=0.52005444, x=-0.51089824, y=0.64045922, z=0.24153336))

def AreQuatsClose(q1, q2) -> bool:
    from numpy import allclose
    return allclose([q1.w, q1.x, q1.y, q1.z], [q2.w, q2.x, q2.y, q2.z])

def test_spin():
    from geometry_msgs.msg import Point as P
    from geometry_msgs.msg import Quaternion as Q
    mc = MessageCreator()


    raws = ["G:10,20", "I:10,20,30", "G:15,20", "I:1,2,3", "G:-20,-30"]
    correct_msgs = [mc.CreateMessage(raw) for raw in raws]
    for raw in raws:
        mc.EnqueueRaw(raw)

    for correct in correct_msgs:
        mc.Spin()
        assert correct == mc.Read()


    