#include "IMUDriver.h"

IMUDriver::IMUDriver(){
  imu = Adafruit_BNO055(55, 0x28, &Wire); //IMU object
  if (!imu.begin()){
    DebugSerial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  DebugSerial.println("IMU initialized succesfully");
}
sensors_event_t IMUDriver::getData(){
  sensors_event_t orientationData , linearAccelData;
  imu.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  return orientationData;
}