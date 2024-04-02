#ifndef RadioDriver_h
#define RadioDriver_h
#include "Arduino.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "Constants.h"
class RadioDriver{
private:
  RH_RF95 rf95;
public:
  RadioDriver();
  void transmitMessage(String msg);
};
#endif