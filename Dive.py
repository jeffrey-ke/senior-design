#!/usr/bin/env python3
import serial
import time
import csv
import sys


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    orientationData = []
    while not ser.is_open:
        print("Waiting for serial connection")
        time.sleep(1)
    line = ""
    ser.reset_input_buffer()
    while line != "Ready":
        print("Waiting for handshake")
        ser.write(b"Pi Ready\n")
        line = ser.readline().decode('utf-8').rstrip()
        print(line)

    #send Test arguments
    #Dive test (LABEL,P,I,D,Duration,Target Depth)
    ser.write(b"DIVE,1.0,1.0,1.0,20000.0,20.0\n")
    while True:
        data = ser.readline().decode('utf-8').rstrip()
        print(data)
        if(data=="done"):
            break
        orientationData.append(data)

    with open('DiveOrientation.csv', 'w') as csvfile:
        csvfile.write("Depth,Orientation,FL,FR,DL,DR\n")
        for orientation in orientationData:
            csvfile.write(orientation+"\n")
        