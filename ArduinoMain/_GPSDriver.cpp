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


void _GPSDriver::getData(){
    char c = GPS.read();
    /*if (GPS.newNMEAreceived()) {
        PISerial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
    PISerial.print("Fix: "); Serial.print((int)GPS.fix);
    PISerial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
        PISerial.print("Location: ");
        PISerial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        PISerial.print(", ");
        PISerial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        PISerial.print("Speed (knots): "); Serial.println(GPS.speed);
        PISerial.print("Angle: "); Serial.println(GPS.angle);
        PISerial.print("Altitude: "); Serial.println(GPS.altitude);
        PISerial.print("Satellites: "); Serial.println((int)GPS.satellites);
        PISerial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }*/
}