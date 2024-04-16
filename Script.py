#!/usr/bin/env python3

from serial import Serial
import time
import sys

class TokenBag:
    def __init__(self, tokens_arr: list[str]) -> None:
        self.buf_ = tokens_arr
        self.i_ = 0
        
    def Lookahead(self):
        if self.i_ >= len(self.buf_):
            return None
        tok = self.buf_[self.i_]
        return tok
    
    def NextToken(self):
        self.i_ = self.i_ + 1

def WaitForSer():
    global ser
    while not ser.is_open:
        print("Waiting for serial connection")
        time.sleep(1)

def WaitForHandshake():
    global ser
    line = ser.readline().decode().rstrip()
    while (line != "Ready"):
        print("Got: {}".format(line))
        print("\tWaiting for handshake")
        ser.write(b"Pi Ready\n")
        line = ser.readline().decode().rstrip()

def GetArgTokens():
    return sys.argv[1:]

def Match(tok):
    global tokens
    if tok != tokens.Lookahead():
            raise Exception("Unexpected token for expression in parse. See Hardwarebridge.py:Match")
    tokens.NextToken()

def Integer():
    try:
        int_val = int(float(tokens.Lookahead()))
        tokens.NextToken()
        return int_val
    except ValueError:
        raise Exception("Tried to match an integer value, but an integer wasn\'t there.")

def Float():
    try:
        float_val = float(tokens.Lookahead())
        tokens.NextToken()
        return float_val
    except ValueError:
        raise Exception("Tried to match a float value, but a float wasn\'t there.")
file_name = input("\n\nFile name: (append \'.csv\' to the end) ")
comment = input("Test description (appended to end of data file): ")
ser = Serial('/dev/ttyACM0', 115200, timeout=1)
WaitForSer()
ser.reset_input_buffer()
WaitForHandshake()
tokens = TokenBag(GetArgTokens())
"""
    test_cmd -> dive_test
              | flip_test
              | pressure_test
              | flip_unflip_test
              | waypoint_test

    dive_test -> DIVE float (Kp) float (Ki) float (Kd) float (duration) float (target depth)
    flip_test -> FLIP float (Kp) float (Ki) float (Kd) float (duration)
    pressure_test -> PRESSURE float (duration) float (max_deviation)
    flip_unflip_test -> FUN float (Kp) float (Ki) float (Kd) float (duration_vertical) float (duration_horizontal) int (pwm_forward)
    waypoint_test -> WAYPOINT float (Kp) float (Ki) float (Kd) float (duration) float (distance_threshold) float (heading_threshold) float (goal_lat) float (goal_lon)                
    quit -> QUIT

"""
def Dive():
    global ser
    Match(DIVE)
    kp = Float()
    ki = Float()
    kd = Float()
    duration = Float()
    target_depth = Float()
    ser.write("DIVE,{},{},{},{},{}\n".format(
        str(kp),
        str(ki),
        str(kd),
        str(duration),
        str(target_depth)
    ).encode())
    
def Flip():
    global ser
    Match(FLIP)
    kp = Float()
    ki = Float()
    kd = Float()
    duration = Float()
    ser.write("FLIP,{},{},{},{}\n".format(
        str(kp),
        str(ki),
        str(kd),
        str(duration)
    ).encode())

def Pressure():
    global ser
    Match(PRESSURE)
    duration = Float()
    max_dev = Float()
    ser.write("PRESSURE,{},{}\n".format(
        str(duration),
        str(max_dev)
    ).encode())

def Fun():
    global ser
    Match(FUN)
    kp = Float()
    ki = Float()
    kd = Float()
    duration_vertical = Float()
    duration_horizontal = Float()
    pwm_forward = Integer()
    ser.write("FUN,{},{},{},{},{},{}\n".format(
        str(kp),
        str(ki),
        str(kd),
        str(duration_vertical),
        str(duration_horizontal),
        str(pwm_forward)
    ).encode())

def Waypoint():
    global ser
    Match(WAYPOINT)
    kp = Float()
    ki = Float()
    kd = Float()
    duration = Float()
    distance_threshold = Float()
    heading_threshold = Float()
    goal_lat = Float()
    goal_lon = Float()
    ser.write("WAYPOINT,{},{},{},{},{},{},{},{}\n".format(
        str(kp),
        str(ki),
        str(kd),
        str(duration),
        str(distance_threshold),
        str(heading_threshold),
        str(goal_lat),
        str(goal_lon)
    ))

from tokens import *
test_type = tokens.Lookahead()
if tokens.Lookahead() == DIVE:
    Dive()
elif tokens.Lookahead() == FLIP:
    Flip() 
elif tokens.Lookahead() == PRESSURE:
    Pressure()
elif tokens.Lookahead() == FUN:
    Fun()
elif tokens.Lookahead == WAYPOINT:
    Waypoint()
elif tokens.Lookahead() == QUIT:
    print("quitting")
    sys.exit()
else:
    print("Not how you use this script.")
    sys.exit()

from collections import deque
data = deque()
line = ser.readline().decode().rstrip()
while (line != "done"):
    print(line)
    data.append(line + "\n")
    line = ser.readline().decode().rstrip()

if file_name == "q":
    print("trashing file...")
    sys.exit()

def GetTestHeader(test_type: str):
    if test_type == DIVE:
        return "Depth(m),Orientation(degrees),FL,FR,DL,DR\n" 
    elif test_type == FLIP:
        return "Orientation(degrees),FL,FR,DL,DR\n"
    elif test_type == PRESSURE:
        return "Pressure(mmHg)\n"
    elif test_type == FUN:
        return "Orientation(degrees),FL,FR,DL,DR\n"
    elif test_type == WAYPOINT:
        return "Time(ms),Waypoint_id,Waypoint_lat,Waypoint_lon,Bearing(degrees),Current_lat,Current_lon,Heading(degrees),FL,FR,DL,DR\n"
    else:
        print("Garbage input. Killing program.")
        sys.exit()


with open(file_name, 'w') as file:
    file.write(GetTestHeader(test_type))
    for line in data:
        file.write(line)
    file.write("\n\n\n\n\n\n")
    file.write(comment)

