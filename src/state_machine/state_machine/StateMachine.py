from geographic_msgs.msg import GeoPoint

class StateMachine:
    def __init__(self):
        #overide max depth determined by hardware constraints
        self.MAXDEPTH = 20.0
        #list of geopoints that are lat, long, depth
        self.waypoints_ = []
        #This variable tracks the state as a string default is standby
        self.state_ = "setup" #valid value: setup, idle, waypoint, profile, return, estop

        self.home_ = GeoPoint(latitude=0.0, longitude=0.0, altitude=0.0)#home coords to return to

    def pushBackWP(self, geopoint):
        if(geopoint.altitude>self.MAXDEPTH):
            geopoint.altitude = self.MAXDEPTH
        self.waypoints_.append(geopoint)

    def pushWP(self, geopoint):
        if(geopoint.altitude>self.MAXDEPTH):
            geopoint.altitude = self.MAXDEPTH
        self.waypoints_.insert(0, geopoint)

    def popWP(self):
        return self.waypoints_.pop(0)

    def currentWP(self):
        return self.waypoints_[0]

    def setHome(self, geopoint):
        self.home_ = geopoint

    #after pivotal ROS callback such as waypoint or profile reasses state
    #takes in as input whether the objective of the current state was achieved
    def assessState(self):
        if(self.state_=="setup"):
            if(len(self.waypoints_)>0):
                self.state_ = "waypoint"
                return True
            else:
                self.state_ = "idle"
                return True
        elif(self.state_=="profile"):
            if(len(self.waypoints_)>0):
                self.state_ = "waypoint"
                return True
            else:
                self.state_ = "idle"
                return True
        elif(self.state_=="waypoint"):
            self.state_ = "profile"
            return True
        elif(self.state_=="idle" and len(self.waypoints_)>0):
            self.state_ = "waypoint"
            return True
        return False
        
    
    def state(self):
        return self.state_



