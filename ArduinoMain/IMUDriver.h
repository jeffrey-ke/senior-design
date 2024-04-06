#ifndef IMUDriver_h
#define IMUDriver_h
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Msgs.h"

class IMUDriver{
  private:
    Adafruit_BNO055 imu_;
    bool alive_;
  public:
    IMUDriver();
    void Init();
    Msg::RPY GetData();
    bool GetAlive() const {return alive_;}


};
#endif