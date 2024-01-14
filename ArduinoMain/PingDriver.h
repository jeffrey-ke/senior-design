#ifndef PingDriver_h
#define PingDriver_h
#include "Arduino.h" 
#include "ping1d.h"

#define DebugSerial Serial3

class PingDriver{
private:
  Ping1D ping; //Ping Sensor Object
public:
  PingDriver(){ }
  int getData(){ }
};
#endif
