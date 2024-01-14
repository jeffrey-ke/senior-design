/* This class controls the communication between the arduino and PI.
  It contains all the necesary drivers for the different sensors.
*/
#ifndef MasterComputeBridge_h
#define MasterComputeBridge_h
#include "Arduino.h" 
#include "Constants.h"
#include ".GPSDriver/GPSDriver.h"
//#include "IMUDriver.h"
//#include "PingDriver.h"
#include "ThrusterDriver/ThrusterDriver.h"
#include "SoftwareSerial.h"

//Pinouts



class MasterComputeBridge{
  private:
    ThrusterDriver thruster1{7};
    ThrusterDriver thruster2{7};
    ThrusterDriver thruster3{7};
    ThrusterDriver thruster4{7};
    IMUDriver IMU;
    //PingDriver ping;
    GPSDriver GPS;
  public:
    MasterComputeBridge(Stream *GPSport){ }
    void IMUSetup(){ }
    void thrusterSetup(){ }
    void giveCommand(){ }
    char * returnCommand(){ }
};
#endif