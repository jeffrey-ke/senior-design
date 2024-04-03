#include "_GPSDriver.h"
float decimalDegrees(float nmeaCoord) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  return wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0;
}

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
    lat_ = (GPS.lat == 'N')? decimalDegrees(GPS.latitude) : -decimalDegrees(GPS.latitude);
    long_ = (GPS.lon == 'W')? decimalDegrees(GPS.longitude): -decimalDegrees(GPS.longitude);
    heading_ = GPS.angle*PI/180;
    DebugSerial.println("Set values properly");
    DebugSerial.println(lat_, 5);
    DebugSerial.println(long_, 5);
    DebugSerial.print("got a fix:\t");
    DebugSerial.println(GPS.fix);
    DebugSerial.println(GPS.antenna);
    DebugSerial.println(fix_);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //once fix is aquired only need minimum data
}

/* Calls GPS.read to clear serial buffer and updates member variables if new valid NMEA recieved
*/
void _GPSDriver::Refresh(){
  char c = GPS.read(); 
  GPS.lastNMEA();
  while(!GPS.newNMEAreceived()){
    char c = GPS.read();
  }
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())){ // this also sets the newNMEAreceived() flag to false
      lat_ = (GPS.lat == 'N')? decimalDegrees(GPS.latitude) : -decimalDegrees(GPS.latitude);
      long_ = (GPS.lon == 'W')? decimalDegrees(GPS.longitude): -decimalDegrees(GPS.longitude);
      heading_ = GPS.angle*PI/180;
      /*DebugSerial.println("Set values properly");
      DebugSerial.println(lat_, 5);
      DebugSerial.println(long_, 5);*/
    } 
  }  
}