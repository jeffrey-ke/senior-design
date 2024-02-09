#ifndef PingDriver_h
#define PingDriver_h
#include "Arduino.h" 
#include "ping1d.h"
#include "Constants.h"
static Ping1D ping { PingSerial }; 
class PingDriver{
private:
  //Ping Sensor Object
public:
  PingDriver();
  int getData();
};
#endif
