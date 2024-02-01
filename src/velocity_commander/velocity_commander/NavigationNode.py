
from ast import Constant
from tokenize import Double
from typing import Tuple
import math
PI = 3.141592654
K_l = 0.5
K_a = 0.5
distanceTolerance = 0.1
angleTolerance = 0.1 
#takes in the current point and goal point as x,y,z as well as heading and returns the linear and angular velocity as (lx,ly,lz,ax,ay,az)
def waypointToVelocity(currentPoint: Tuple[float, float, float], goalPoint: Tuple[float, float, float], heading: float ) -> Tuple[float, float, float, float, float, float]: 
#Get the heading error i.e. how many radians does the robot need to turn to head towards the waypoint 
  deltaX = goalPoint[0] - currentPoint[0]
  deltaY = goalPoint[1] - currentPoint[1]
  waypointHeading = math.atan2(deltaY, deltaX)
  headingError = waypointHeading - heading;   
   
  #Make sure heading error falls within -PI to PI range
  if (headingError > PI):
    headingError = headingError - (2 * PI)
  if (headingError < -PI):
    headingError = headingError + (2 * PI)
 
  distanceToWaypoint = getDistanceToWaypoint(currentPoint,goalPoint)

  if (abs(distanceToWaypoint) > distanceTolerance):    
    # If the robot's heading is off, fix it.
    if (abs(headingError) > angleTolerance):
      return (0.0,0.0,0.0,0.0,0.0,K_a  * headingError)
    #Just fix the distance gap between current pose and waypoint. The magnitude of the robot's velocity 
    #is directly proportional to the distance the robot is from the goal.
    else:
      return (K_l * distanceToWaypoint,0.0,0.0,0.0,0.0,0.0)
  else:
    return (0.0,0.0,0.0,0.0,0.0,0.0)

def getDistanceToWaypoint(currentPoint: Tuple[float, float, float], goalPoint: Tuple[float, float, float]) -> float:
  return math.sqrt(pow(goalPoint[0] - currentPoint[0], 2) + pow(
    goalPoint[0] - currentPoint[0], 2))

#Takes in desired velocity as (lx,ly,lz,ax,ay,az) and returns PMU commands for thruster as (FL,FR,DL,DR)
def velocityToPWM(vel: Tuple[float, float, float, float, float, float]) -> Tuple[float, float, float, float]:
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
        return (vel[0]*10,vel[0]*10,0,0)
    elif(vel[5]!=0):
        return (-5*vel[5],5*vel[5],0.0,0.0)
    
