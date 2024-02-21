#include "_GPSDriver.h"

_GPSDriver::_GPSDriver(): GPS(&GPSSerial){
    //GPS Initialization
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    timer_ = millis();
}


void _GPSDriver::Refresh(){
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
    if (millis() - timer_ > 2000) 
    {
        timer_ = millis();
        lat_ = (GPS.lat == 'N')? GPS.latitudeDegrees : -GPS.latitudeDegrees;
        long_ = (GPS.lon == 'W')? GPS.longitudeDegrees: -GPS.longitudeDegrees;
        heading_ = GPS.angle;
        fix_ = GPS.fix;
    }
}