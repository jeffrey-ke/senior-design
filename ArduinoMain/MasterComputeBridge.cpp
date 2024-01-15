/* This class controls the communication between the arduino and PI.
  It contains all the necesary drivers for the different sensors.
*/
#include "MasterComputeBridge.h"

#define PISerial Serial
#define GPSSerial Serial1
#define PingSerial Serial2
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0 //Standard frequency for use in the US

MasterComputeBridge::MasterComputeBridge(){
  thrusterSetup();
}
void MasterComputeBridge::thrusterSetup(){
    thruster1 = ThrusterDriver(TPIN1);
    thruster2 = ThrusterDriver(TPIN2);
    thruster3 = ThrusterDriver(TPIN3);
    thruster4 = ThrusterDriver(TPIN4);
}
void MasterComputeBridge::giveCommand(String command){
  int seperator = command.indexOf(":");
  if(command.substring(0,seperator) == "T"){
    int vel = command.substring(seperator+1).toInt();
    DebugSerial.println(vel);
    thruster1.setVelocity(vel);
  }
  else if(command.substring(0,seperator) == "P"){
      //Parse and execute radio command
  }
  else if(command.substring(0,seperator) == "I"){
    sensors_event_t orientationData = IMU.getData();
    DebugSerial.print("X Orientation: ");
    DebugSerial.println(orientationData.orientation.x);
    DebugSerial.print("Y Orientation: ");
    DebugSerial.println(orientationData.orientation.y);
    DebugSerial.print("Z Orientation: ");
    DebugSerial.println(orientationData.orientation.z);
    DebugSerial.println();
  }
  else if(command.substring(0,seperator) == "G"){
      //Parse and execute radio command
  }
  else if(command.substring(0,seperator) == "E"){
      //Parse and execute EStop command
  }
  else {
    DebugSerial.print("Unable to parse command- ");
    DebugSerial.println(command);
  } 
}
char * MasterComputeBridge::returnCommand(){

}
