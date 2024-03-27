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

    raw = "G:10,10,10"
    assert pc.ParseConvert(raw) == {"type" : "G", 
                                    "data" : 
                                        {
                                            "lat" : 10.0, 
                                            "long" : 10.0,
                                            "heading": 10.0
                                        }
                                    }
    raw = "G:69,69,69"
    assert pc.ParseConvert(raw) == {"type" : "G", 
                                    "data" : 
                                        {
                                            "lat" : 69.0, 
                                            "long" : 69.0,
                                            "heading": 69.0
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
    from geographic_msgs.msg import GeoPoint

    mc = MessageCreator()
    raw = "G:1010.10,1010.10,69.0"
    assert ArePointsClose(mc.CreateMessage(raw),GeoPoint(latitude=(10 + 10.1/60), longitude=(10 + 10.1/60),altitude=69.0))

    raw = "G:2020.20,3030.30,69.0"
    assert ArePointsClose(mc.CreateMessage(raw),GeoPoint(latitude=(20 + 20.20/60), longitude=(30 + 30.30/60), altitude=69.0))


def ArePointsClose(p1, p2, tolerance=1e-2) -> bool:
    from numpy import allclose
    return allclose([p1.latitude, p1.longitude], [p2.latitude, p2.longitude], atol=tolerance)
        
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


    raws = ["G:10,20,69", "I:10,20,30", "G:15,20,69", "I:1,2,3", "G:-20,-30,69"]
    correct_msgs = [mc.CreateMessage(raw) for raw in raws]
    for raw in raws:
        mc.EnqueueRaw(raw)

    for correct in correct_msgs:
        mc.Spin()
        assert correct == mc.Read()


    