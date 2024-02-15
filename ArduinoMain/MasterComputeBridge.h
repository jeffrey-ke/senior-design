/* This class controls the communication between the arduino and PI.
  It contains all the necesary drivers for the different sensors.
*/
#ifndef MasterComputeBridge_h
#define MasterComputeBridge_h
#include "Arduino.h" 
#include "Constants.h"
#include "_GPSDriver.h"
#include "IMUDriver.h"
#include "PingDriver.h"
#include "ThrusterDriver.h"
//#include "RadioDriver.h"
class MasterComputeBridge{
  private:
    ThrusterDriver thruster1;
    ThrusterDriver thruster2;
    ThrusterDriver thruster3;
    ThrusterDriver thruster4;
    IMUDriver IMU;
    PingDriver ping;
    _GPSDriver GPS;
    //RadioDriver Lora;
    String functionReturn;
  public:
    MasterComputeBridge();
    void IMUSetup();
    void thrusterSetup();
    void giveCommand(String command);
    String returnCommand();
};
#endif