
from typing import Tuple
from math import sin, cos, sqrt, radians, atan2, pi

class NavigationNode:
  K_l = 0.5
  K_a = 0.5
  distanceTolerance = 0.1
  angleTolerance = 0.1 


  def getHeadingError(self, lat1, long1, lat2, long2, current_heading) -> float:
    y = sin(long2 - long1) * sin(lat2) 
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(long2 - long1)
    bearing = atan2(y, x)
    h_err = bearing - current_heading

    if (h_err > pi):
      h_err = h_err - (2 * pi)  
    if (h_err < -pi):
      h_err = h_err + (2 * pi)

    return h_err

  def atWaypoint(self, lat1, long1, lat2, long2) -> bool:
    if(self.getDistanceToWaypoint(lat1, long1, lat2, long2)<self.distanceTolerance):
      return True
    return False
  def getDistanceToWaypoint(self, lat1, long1, lat2, long2) -> float:
    return self.get_distance(lat1, long1, lat2, long2)

  def get_bearing(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    #print(long1)
    x = cos(radians(lat2)) * sin(radians(dLon))
    y = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(dLon))
    brng = numpy.arctan2(x,y)
    return brng

  def get_distance(self, lat1, long1, lat2, long2):
    dLat = (radians(lat2) - radians(lat1))
    dLon = (radians(long2) - radians(long1))
    R = 6373.0
    a = sin(dLat/2)**2 + cos(lat1) * cos(lat2) * sin(dLon/2)**2
    c = 2* atan2(sqrt(a), sqrt(1-a))
    distance = R*c*1000
    return distance
    
  def waypointToPwm(self,lat1, long1, lat2, long2, current_heading):
    velocity = self.waypointToVelocity(lat1, long1, lat2, long2, current_heading)
    pwm = self.velocityToPWM(velocity)
    return pwm

  #takes in the current point and goal point as x,y,z as well as heading and returns the linear and angular velocity as (lx,ly,lz,ax,ay,az)
  def waypointToVelocity(self, lat1, long1, lat2, long2, current_heading) -> Tuple[float, float, float, float, float, float]: 
  #Get the heading error i.e. how many radians does the robot need to turn to head towards the waypoint 
    heading_error = self.getHeadingError(radians(lat1), radians(long1), radians(lat2), radians(long2), current_heading)  
  
    distanceToWaypoint = self.getDistanceToWaypoint(lat1, long1, lat2, long2)

    if (abs(distanceToWaypoint) > self.distanceTolerance):    
      # If the robot's heading is off, fix it.
      if (abs(heading_error) > self.angleTolerance):
        return (0.0,0.0,0.0,0.0,0.0, self.K_a  * heading_error)
      #Just fix the distance gap between current pose and waypoint. The magnitude of the robot's velocity 
      #is directly proportional to the distance the robot is from the goal.
      else:
        return (self.K_l * distanceToWaypoint,0.0,0.0,0.0,0.0,0.0)
    else:
      return (0.0,0.0,0.0,0.0,0.0,0.0)



  #Takes in desired velocity as (lx,ly,lz,ax,ay,az) and returns PMU commands for thruster as (FL,FR,DL,DR)
  def velocityToPWM(self, vel: Tuple[float, float, float, float, float, float]) -> Tuple[float, float, float, float]:
      #cap velocity values to 10m/s across water and 2m/s while diving
      lx = vel[0]
      ly = vel[1]
      az = vel[5]
      if(vel[0]>10.0):
          lx=10
      elif(vel[0]<-10.0):
          lx=-10
      if(vel[1]>2.0):
          ly=2
      elif(vel[1]<-2.0):
          ly=-2
      if(vel[5]>2.0):
          az=2
      elif(vel[5]<-2.0):
          az=-2
      
      #determine PWM commands
      if(vel[3]==0 and vel[4]==0 and vel[5]==0): #no angular change
          return (lx*40 + 1500, 
                  lx*40 + 1500,
                  1500,
                  1500)
      elif(vel[5]!=0):
          return (40*az + 1500,
                  -40*az + 1500,
                  1500,
                  1500)
  def descendToPwm(self, currentDepth, desiredDepth, heading):
    print('descending')
    return (0.0,0.0,0.0,0.0)

  def ascendToPwm(self, currentDepth, desiredDepth, heading):
    print('ascending')
    return (0.0,0.0,0.0,0.0)
