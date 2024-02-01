
from collections import deque
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPoint
from multiprocessing import Lock

class MessageCreator():
    
    
    def __init__(self, raws_buf_lock=Lock(), processed_buf_lock=Lock()) -> None:
        self.raw_msgs_ = deque()
        self.msgs_ = deque()
        self.parser_ = ParserConverter()
        self.raw_msgs_lock_ = raws_buf_lock
        self.msgs_lock_ = processed_buf_lock

    def EnqueueRaw(self, raw_msg):
        if(raw_msg == None):
            return
        with self.raw_msgs_lock_:
            self.raw_msgs_.append(raw_msg)
    
    def DequeueRaw(self):
        with self.raw_msgs_lock_:
            return self.raw_msgs_.popleft() if len(self.raw_msgs_) > 0 else None
        
    def EnqueueProcessedMsg(self, procs_msg):
        with self.msgs_lock_:
            self.msgs_.append(procs_msg)
   
    def Read(self):
        with self.msgs_lock_:
            return self.msgs_.popleft() if len(self.msgs_) > 0 else None

    def CreateMessage(self, raw):
        parsed = self.parser_.ParseConvert(raw)
        type = parsed["type"]
        data = parsed["data"]

        if type == "G":
            return self.CreatePoint(data)

        elif type == "I":
            return self.CreateQuaternion(data)

    def CreatePoint(self, data):
        lat, long = data["lat"], data["long"]
        dec_lat, dec_long = self.LatLongDegreesToDecimal(lat, long)
        return GeoPoint(latitude=dec_lat, longitude=dec_long, altitude=0.0)

    def CreateQuaternion(self, data):
        x, y, z = data["x"], data["y"], data["z"]
        q_w, q_x, q_y, q_z = self.TranslateToQuaternion(x, y, z)
 
        return Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)

    def Spin(self):
        raw = self.DequeueRaw()
        if raw is not None:
            self.EnqueueProcessedMsg(self.CreateMessage(raw))
   
    def TranslateToQuaternion(self, x, y, z):
        from transforms3d._gohlketransforms import quaternion_from_euler
        return quaternion_from_euler(x, y, z)

    def TranslateToCartesian(self, lat, long):
        from math import sin, cos
        EARTH_RADIUS_METERS = 6.378137e6
        dec_lat, dec_long = self.LatLongDegreesToDecimal(lat, long)
        return (EARTH_RADIUS_METERS * cos(dec_long) * cos(dec_lat), 
                EARTH_RADIUS_METERS * sin(dec_long) * cos(dec_lat),
                EARTH_RADIUS_METERS * sin(dec_lat))
    
    """
        NMEA format: DDMM.MMMM
    """
    def LatLongDegreesToDecimal(self, lat, long):
        degs_lat = lat // 100 # extracts DD
        minutes_lat = lat % 100 # extracts MM.MMMM

        degs_long = long // 100
        minutes_long = long % 100

        return (degs_lat + minutes_lat / 60, degs_long + minutes_long / 60)
        

    
        

class ParserConverter():
    def __init__(self) -> None:
        pass

    def Identify(self, raw):
        id = raw[0]
        raw_sliced = raw.split(":")[1]
        return (id, raw_sliced)

    def ParseConvert(self, raw):
        id, raw_sliced = self.Identify(raw)
        tokenized = raw_sliced.split(",")

        """
        GPS:
            G:lat,long
        IMU:
            I:x,y,z
        Thruster:

        """

        if (id == "G"):
            return {"type" : "G", 
                    "data" : 
                        {
                            "lat" : float(tokenized[0]), 
                            "long" : float(tokenized[1])
                        }
                    }
        elif (id == "I"):
            return {"type": "I",
                    "data" : 
                        {
                            "x" : float(tokenized[0]),
                            "y" : float(tokenized[1]),
                            "z" : float(tokenized[2])
                        }
                    }

 
