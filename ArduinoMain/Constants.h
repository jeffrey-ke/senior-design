/*  This file acts as a centralized place for all constants, ports, and pin numbers.
    simply change the value here to reflect it in rest of program. Make sure to 
    #include this file in all other files
*/
#ifndef Constants_h
#define Constants_h

#include "Arduino.h"


/*Serial  USB
  Serial1 19(RX1), 18(TX1)
  Serial2 17(RX2), 16(TX2)
  Serial3 15(RX3), 14(TX3)
*/
#define PISerial Serial
#define GPSSerial Serial1

#define PingSerial Serial2
#define DebugSerial Serial1
const int RFM95_CS = 4;
const int RFM95_RST = 2;
const int RFM95_INT = 3;

const int FL_PIN = 7;
const int FR_PIN = 8;
const int DL_PIN = 9;
const int DR_PIN = 10;

#define RF95_FREQ 915.0 //Standard frequency for use in the US

#endif
