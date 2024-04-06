#!/usr/bin/env python3
import serial
import time
import csv


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    pressureData = []
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
    #ser.reset_input_buffer()
    #time.sleep(1) #let arduino clear buffer    
    #send PID args
    ser.write(b"PRESSURE,10000.0,25.0\n")
    while True:
        data = ser.readline().decode('utf-8').rstrip()
        print(data)
        if(data=="done"):
            break
        pressureData.append(data)
    with open('pressure.csv', 'w') as csvfile:
        csvfile.write("Pressure\n")
        for pressure in pressureData:
            csvfile.write(pressure+ "\n")
        
