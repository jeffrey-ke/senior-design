#ifndef ThrusterDriver_h
#define ThrusterDriver_h
#include "Arduino.h" 
#include <Servo.h>
#include "Constants.h"
class ThrusterDriver{
  private:
    Servo thruster;
  public:
    ThrusterDriver();
    ThrusterDriver(const int servoPin);
    void setVelocity(int vel);

};
#endif