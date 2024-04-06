#!/usr/bin/env python3
import serial
import time
import csv


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
    #Flip test (LABEL,P,I,D,Duration)
    ser.write(b"FLIP,2,0.2,0.1,60000.0\n")
    while True:
        data = ser.readline().decode('utf-8').rstrip()
        print(data)
        if(data=="done"):
            break
        orientationData.append(data)


    with open('Kp2Ki0_2Kd0_1seconds60_2.csv', 'w') as csvfile:
        csvfile.write("Orientation, FL, FR, DL, DR\n")
        for orientation in orientationData:
            csvfile.write(orientation + "\n")
        
