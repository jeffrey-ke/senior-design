#ifndef _GPSDriver_h
#define _GPSDriver_h
#include <Adafruit_GPS.h>
#include "Constants.h"

class _GPSDriver{
public:
  constexpr static double INVALID = -99.0;

  _GPSDriver();
  void Refresh();
  Msg::GNSS GetGNSS() const {Refresh(); return (fix_)? Msg::GNSS{GetLat(), GetLong(), GetHeading()} : Msg::gnss_INVALID;}
  double GetLat() const {return (fix_)? lat_: INVALID;}
  double GetLong() const {return (fix_)? long_: INVALID;}
  double GetHeading() const {return (fix_) ? heading_: INVALID;}

  double test_SetLatLongHeading(double lat, double lon, double hed) {fix_ = true; lat_ = lat; long_ = lon; heading_ = hed;}
  double test_SetFixFalse() {fix_ = false;}
private:
  Adafruit_GPS GPS;
  double lat_{INVALID}, long_{INVALID}, heading_{INVALID};
  bool fix_{false};
  uint32_t timer_;
};
#endif