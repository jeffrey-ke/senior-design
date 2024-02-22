#include "IMUDriver.h"
#include <Wire.h>
#include "Constants.h"

IMUDriver::IMUDriver()
:
imu_(55, 0x28) //IMU object
{
  if (!imu_.begin()){
    alive_ = false;
    Serial.println("IMU failed");
  }
  else 
    alive_ = true;
  imu_.setExtCrystalUse(true);
}
Msg::RPY IMUDriver::GetData(){
  sensors_event_t event;
  imu_.getEvent(&event);
  return Msg::RPY{event.orientation.x, event.orientation.y, event.orientation.z};
}