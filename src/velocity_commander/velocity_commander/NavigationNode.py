
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
    if(getDistanceToWaypoint(lat1, long1, lat2, long2)<distanceTolerance):
      return True
    return False
  def getDistanceToWaypoint(self, lat1, long1, lat2, long2) -> float:
    RADIUS_EARTH = 6378.1370e3
    lat1 = radians(lat1)
    long1 = radians(long1)  
    lat2 = radians(lat2)
    long2 = radians(long2)

    delta_lat = lat2 - lat1
    delta_long = long2 - long1

    a = sin(delta_lat/2)**2 + cos(lat1) * cos(lat2) * sin(delta_long/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return RADIUS_EARTH * c

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
      if(vel[0]>10.0):
          vel[0]=10
      elif(vel[0]<-10.0):
          vel[0]=-10
      if(vel[1]>2.0):
          vel[1]=2
      elif(vel[1]<-2.0):
          vel[1]=-2
      if(vel[5]>2.0):
          vel[5]=2
      elif(vel[5]<-2.0):
          vel[5]=-2
      
      #determine PWM commands
      if(vel[3]==0 and vel[4]==0 and vel[5]==0): #no angular change
          return (vel[0]*10, 
                  vel[0]*10,
                  0.0,
                  0.0)
      elif(vel[5]!=0):
          return (-5*vel[5],
                  5*vel[5],
                  0.0,
                  0.0)
  def descendToPwm(self, currentDepth, desiredDepth, heading):
    print('descending')
    return (0.0,0.0,0.0,0.0)

  def ascendToPwm(self, currentDepth, desiredDepth, heading):
    print('ascending')
    return (0.0,0.0,0.0,0.0)
