from hardware_bridge.MessageCreator import MessageCreator
from hardware_bridge.MessageCreator import ParserConverter


def test_add_raw():
    mc = MessageCreator()

    assert len(mc.raw_msgs_) == 0

    raw = "G:10,10"
    mc.Queue(raw)

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
def test_create_msg():
    from math import sin,cos
    from geometry_msgs.msg import Quaternion, Point

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
        

    

