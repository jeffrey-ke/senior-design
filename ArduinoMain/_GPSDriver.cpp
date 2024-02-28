#include "_GPSDriver.h"

_GPSDriver::_GPSDriver(): GPS(&GPSSerial){
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); //Set update rate
    GPS.sendCommand(PGCMD_ANTENNA); //set antenna on
    delay(1000);
    DebugSerial.println("Waiting for gps fix...");
    while(!GPS.fix){
      char c = GPS.read();
      if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA());
      }
    }
    fix_ = true;
    DebugSerial.println("got a fix!");
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //once fix is aquired only need minimum data
}

/* Calls GPS.read to clear serial buffer and updates member variables if new valid NMEA recieved
*/
void _GPSDriver::Refresh(){
  char c = GPS.read(); 
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())){ // this also sets the newNMEAreceived() flag to false
      lat_ = (GPS.lat == 'N')? GPS.latitudeDegrees : -GPS.latitudeDegrees;
      long_ = (GPS.lon == 'W')? GPS.longitudeDegrees: -GPS.longitudeDegrees;
      heading_ = GPS.angle;
      //DebugSerial.println("Set values properly");
      //DebugSerial.println(lat_);
      //DebugSerial.println(long_);
    } 
  }    
}