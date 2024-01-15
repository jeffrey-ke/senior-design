#ifndef Constants_h
#define Constants_h
#include "Arduino.h"

#define PISerial Serial1
#define GPSSerial Serial3
#define PingSerial Serial2
#define DebugSerial Serial
// DebugSerial.begin(115200);
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
const int TPIN1 = 7;
const int TPIN2 = 8;
const int TPIN3 = 9;
const int TPIN4 = 10;
#define RF95_FREQ 915.0 //Standard frequency for use in the US

#endif
