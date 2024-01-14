/* This class controls the communication between the arduino and PI.
  It contains all the necesary drivers for the different sensors.
*/
#include "GPSDriver.h"
#include "IMUDriver.h"
#include "PingDriver.h"
#include "ThrusterDriver.h"
#include "SoftwareSerial.h"
#include "Arduino.h"
//Pinouts
#define PISerial Serial
#define GPSSerial Serial1
#define PingSerial Serial2
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0 //Standard frequency for use in the US

MasterComputeBridge::MasterComputeBridge(Stream *GPSport){
    //thrusterSetup();
    IMUSetup();
    GPS = GPSDriver(GPSPort)

}
void MasterComputeBridge::IMUSetup(){
    IMU = new IMUDriver();
}
void MasterComputeBridge::thrusterSetup(){
    thruster1 = new ThrusterDriver(7);
    thruster2 = new ThrusterDriver(8);
    thruster3 = new ThrusterDriver(9);
    thruster4 = new ThrusterDriver(10);
}
void MasterComputeBridge::giveCommand(){

}
char * MasterComputeBridge::returnCommand(){

}
