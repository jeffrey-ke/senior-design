#ifndef ThrusterDriver_h
#define ThrusterDriver_h
#include "Arduino.h" 
#include <Servo.h>
class ThrusterDriver{
  private:
    Servo thruster;
  public:
    ThrusterDriver(int servoPin){ }
    void setVelocity(int vel){ }

};
#endif