#include "_GPSDriver.h"

_GPSDriver::_GPSDriver(){
    Adafruit_GPS GPS = Adafruit_GPS(&GPSSerial);
    //GPS Initialization
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);
    DebugSerial.println("GPS initialized succesfully");
}


double * _GPSDriver::getData(){
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
      DebugSerial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return nullptr; // we can fail to parse a sentence in which case we should just wait for another
  }
  DebugSerial.print("Fix: "); DebugSerial.print((int)GPS.fix);
  DebugSerial.print(" quality: "); DebugSerial.println((int)GPS.fixquality);
  if (GPS.fix) {
      DebugSerial.print("Location: ");
      DebugSerial.print(GPS.latitude, 4); DebugSerial.print(GPS.lat);
      DebugSerial.print(", ");
      DebugSerial.print(GPS.longitude, 4); DebugSerial.println(GPS.lon);
      DebugSerial.print("Speed (knots): "); DebugSerial.println(GPS.speed);
      DebugSerial.print("Angle: "); DebugSerial.println(GPS.angle);
      DebugSerial.print("Altitude: "); DebugSerial.println(GPS.altitude);
      DebugSerial.print("Satellites: "); DebugSerial.println((int)GPS.satellites);
      DebugSerial.print("Antenna status: "); DebugSerial.println((int)GPS.antenna);
  }
  double latlong[2];
  latlong[0] = GPS.latitude;
  latlong[1] = GPS.longitude;
  return latlong;
}