
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Int16

class MessageCreator():
    
    
    def __init__(self) -> None:
        self.parser_ = ParserConverter()

    def CreateMessage(self, raw):
        parsed = self.parser_.ParseConvert(raw)

        if parsed is None:
            return None
        
        msg_id = parsed["msg_id"]
        data = parsed["data"]

        if msg_id == "G":
            return self.CreatePoint(data)

        elif msg_id == "I":
            return self.CreateQuaternion(data)
        
        elif msg_id == "S":
            
            return Int16(data=data["alive"])

    def CreatePoint(self, data):
        lat, long, heading = data["lat"], data["long"], data["heading"]
        return GeoPoint(latitude=lat, longitude=long, altitude=heading)

    def CreateQuaternion(self, data):
        x, y, z = data["x"], data["y"], data["z"]
        q_w, q_x, q_y, q_z = self.TranslateToQuaternion(x, y, z)
 
        return Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)


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
        self._AllowedIds = ["I", "G", "T", "S"]
        self._MsgLengths = {
            "I": 3, # x y z
            "G": 3, # lat long heading
            "S": 1 # alive
        }

    def Identify(self, raw):
        if (raw == "Initialization Done"):
            return
        msg_id = raw[0]
        print(raw)
        raw_sliced = raw.split(":")[1]
        return (msg_id, raw_sliced)
    
    def _CheckMsgTokenized(self, msg_id, tokenized):
        return msg_id  in self._AllowedIds and self._MsgLengths[msg_id] == len(tokenized)
         

    def ParseConvert(self, raw):
        msg_id, raw_sliced = self.Identify(raw)
        tokenized = raw_sliced.split(",")

        if self._CheckMsgTokenized(msg_id, tokenized) is False:
            return None

        if (msg_id == "G"):
            return {"msg_id" : "G", 
                    "data" : 
                        {
                            "lat" : float(tokenized[0]), 
                            "long" : float(tokenized[1]),
                            "heading": float(tokenized[2])
                        }
                    }
        elif (msg_id == "I"):
            return {"msg_id": "I",
                    "data" : 
                        {
                            "x" : float(tokenized[0]),
                            "y" : float(tokenized[1]),
                            "z" : float(tokenized[2])
                        }
                    }
        elif (msg_id == "S"):
            return {"msg_id": "S",
                    "data":
                        {
                            "alive" : int(tokenized[0])
                        }

            }

 
