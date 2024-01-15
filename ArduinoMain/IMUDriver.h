#ifndef IMUDriver_h
#define IMUDriver_h
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "Arduino.h" 
#include "Constants.h"

class IMUDriver{
  private:
    Adafruit_BNO055 imu;
  public:
    IMUDriver();
    sensors_event_t getData();

};
#endif