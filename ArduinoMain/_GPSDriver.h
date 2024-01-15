#ifndef _GPSDriver_h
#define _GPSDriver_h
#include <Adafruit_GPS.h>
#include "SoftwareSerial.h"
#include "Constants.h"

class _GPSDriver{
private:
  Adafruit_GPS GPS;
public:
  _GPSDriver();
  void getData();
};
#endif