/*  This file acts as a centralized place for all constants, ports, and pin numbers.
    simply change the value here to reflect it in rest of program. Make sure to 
    #include this file in all other files
*/
#ifndef Constants_h
#define Constants_h
#include "Arduino.h"

#define PISerial Serial
#define GPSSerial Serial3
#define PingSerial Serial2
#define DebugSerial Serial1
const int RFM95_CS = 4;
const int RFM95_RST = 2;
const int RFM95_INT = 3;
const int TPIN1 = 7;
const int TPIN2 = 8;
const int TPIN3 = 9;
const int TPIN4 = 10;
#define RF95_FREQ 915.0 //Standard frequency for use in the US

#endif
