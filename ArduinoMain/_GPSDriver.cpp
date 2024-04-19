#include "_GPSDriver.h"
static degrees decimalDegrees(float nmeaCoord) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  return wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0;
}

_GPSDriver::_GPSDriver(): GPS(&GPSSerial){}

void _GPSDriver::Init() {
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); //Set update rate
    GPS.sendCommand(PGCMD_ANTENNA); //set antenna on
    delay(1000);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //once fix is aquired only need minimum data
}

/* Calls GPS.read to clear serial buffer and updates member variables if new valid NMEA recieved
*/
void _GPSDriver::Refresh(){
    while(!GPS.fix){
        char c = GPS.read();
        if (GPS.newNMEAreceived()) {
          GPS.parse(GPS.lastNMEA());
        }
    }
    lat_ = (GPS.lat == 'N')? decimalDegrees(GPS.latitude) : -decimalDegrees(GPS.latitude);
    long_ = (GPS.lon == 'E')? decimalDegrees(GPS.longitude): -decimalDegrees(GPS.longitude);
    heading_ = GPS.angle*PI/180;  
}

void _GPSDriver::GetGNSS(){
    Refresh();
    return Msg::GNSS{GetLat(), GetLong(), GetHeading()};
}