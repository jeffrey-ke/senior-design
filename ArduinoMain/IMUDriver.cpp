#include "IMUDriver.h"
#include <Wire.h>
#include "Constants.h"
#include "Arduino.h"

// Convenience aliases
using offsets_struct = adafruit_bno055_offsets_t;

IMUDriver::IMUDriver()
:
imu_(55) //IMU object
{
}

void IMUDriver::Init() {
    alive_ = imu_.begin(OPERATION_MODE_CONFIG);
    delay(19);
    //PISerial.print("IMU operation mode: ");
    //PISerial.println(imu_.getMode());
    //imu_.setExtCrystalUse(true);
    offsets_struct offsets;
    /*offsets.accel_offset_x = -27;
    offsets.accel_offset_y = 6;
    offsets.accel_offset_z = -31;
    offsets.gyro_offset_x = -1;
    offsets.gyro_offset_y = -8;
    offsets.gyro_offset_z = -2;
    offsets.mag_offset_x = -442;
    offsets.mag_offset_y = -263;
    offsets.mag_offset_z = -23;
    offsets.accel_radius = 1000;
    offsets.mag_radius = 669;*/
    offsets.accel_offset_x = -3;
    offsets.accel_offset_y = 22;
    offsets.accel_offset_z = -13;
    offsets.gyro_offset_x = -1;
    offsets.gyro_offset_y = -3;
    offsets.gyro_offset_z = 1;
    offsets.mag_offset_x = -314;
    offsets.mag_offset_y = -125;
    offsets.mag_offset_z = -39;
    offsets.accel_radius = 1000;
    offsets.mag_radius = 684;
    imu_.setSensorOffsets(offsets);
    imu_.setMode(OPERATION_MODE_NDOF);
    delay(10);
    //PISerial.print("IMU operation mode: ");
    //PISerial.println(imu_.getMode());
}
Msg::RPY IMUDriver::GetData(){
  /*sensors_event_t event;
  Serial.println(imu_.isFullyCalibrated());
  imu_.getEvent(&event);*/
  imu::Quaternion quat = imu_.getQuat();

  imu::Vector<3> eul = quat.toEuler();

  return Msg::RPY{-180/M_PI*eul.x(), -180/M_PI*eul.y(), -180/M_PI*eul.z()};
}