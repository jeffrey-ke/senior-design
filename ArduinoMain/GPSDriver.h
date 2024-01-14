
#include <Adafruit_GPS.h>
#include "SoftwareSerial.h"

class GPSDriver{
private:
  Adafruit_GPS GPS;
  Stream *GPSSerial;
public:
  GPSDriver(Stream *port){ }
  void getData(){ }
};
