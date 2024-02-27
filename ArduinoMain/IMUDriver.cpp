#include "IMUDriver.h"
#include <Wire.h>
#include "Constants.h"
#include "Arduino.h"

IMUDriver::IMUDriver()
:
imu_(55) //IMU object
{
}

void IMUDriver::Init() {
    alive_ = imu_.begin();
    imu_.setExtCrystalUse(true);
}
Msg::RPY IMUDriver::GetData(){
  sensors_event_t event;
  imu_.getEvent(&event);
  return Msg::RPY{event.orientation.x, event.orientation.y, event.orientation.z};
}