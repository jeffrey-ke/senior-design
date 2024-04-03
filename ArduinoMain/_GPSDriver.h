#ifndef _GPSDriver_h
#define _GPSDriver_h
#include <Adafruit_GPS.h>
#include "Constants.h"
#include "Msgs.h"

class _GPSDriver{
public:
  constexpr static double INVALID = -99.0;

  _GPSDriver();
  void Refresh();

  Msg::GNSS GetGNSS() const {return Msg::GNSS{GetLat(), GetLong(), GetHeading()};}
  double GetLat() const {return lat_;}
  double GetLong() const {return long_;}
  double GetHeading() const {return heading_;}
  bool GetFix() const {return fix_;}

  double test_SetLatLongHeading(double lat, double lon, double hed) {fix_ = true; lat_ = lat; long_ = lon; heading_ = hed;}
  double test_SetFixFalse() {fix_ = false;}
private:
  Adafruit_GPS GPS;
  double lat_{INVALID}, long_{INVALID}, heading_{INVALID};
  bool fix_{false};
};
#endif