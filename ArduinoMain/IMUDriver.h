#ifndef IMUDriver_h
#define IMUDriver_h
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "Arduino.h" 
#include "Constants.h"
#include <Adafruit_Sensor.h>


class IMUDriver{
  private:
    Adafruit_BNO055 imu;
    bool alive;
  public:
    IMUDriver();
    sensors_event_t getData();
    void Init();
    void displayCalStatus(void);
};
#endif