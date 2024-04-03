#include "IMUDriver.h"

IMUDriver::IMUDriver()
:
imu(55) //IMU object
{
}
void IMUDriver::Init() {
    alive = imu.begin();
    imu.setExtCrystalUse(true);
}
sensors_event_t IMUDriver::getData(){
  sensors_event_t orientationData , linearAccelData;
  imu.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  return orientationData;
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void IMUDriver::displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  imu.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  while (!system)
  {
    Serial.println("calibrating...");
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.println(mag, DEC);
    delay(1000);
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}