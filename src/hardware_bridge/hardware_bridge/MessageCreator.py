
from collections import deque
from geometry_msgs.msg import Quaternion, Point

class MessageCreator():
    
    
    def __init__(self) -> None:
        self.raw_msgs_ = deque()
        self.parser_ = ParserConverter()

    def Queue(self, raw_msg):
        self.raw_msgs_.append(raw_msg)
    
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
        x, y, z = self.TranslateToCartesian(lat, long)
        return Point(x=x, y=y, z=z)

    def CreateQuaternion(self, data):
        x, y, z = data["x"], data["y"], data["z"]
        q_w, q_x, q_y, q_z = self.TranslateToQuaternion(z, y, z)
 
        return Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)

    def Spin(self):
        # create message
            #Get the parsed
            #create a message
        pass

    def TranslateToQuaternion(self, x, y, z):
        return (w, x, y, z)

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

 
