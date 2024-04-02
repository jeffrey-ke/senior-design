#ifndef ThrusterDriver_h
#define ThrusterDriver_h
#include "Arduino.h" 
#include <ServoTimer2.h>
#include "Constants.h"
class ThrusterDriver{
  private:
    ServoTimer2 thruster;
  public:
    ThrusterDriver();
    ThrusterDriver(const int servoPin);
    void setVelocity(int vel);

};
#endif