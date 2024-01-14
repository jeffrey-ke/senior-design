#include <Adafruit_BNO055.h>
#include <Wire.h>

class IMUDriver{
  private:
    Adafruit_BNO055 imu;
  public:
    IMUDriver(){
      imu = Adafruit_BNO055(55, 0x28, &Wire); //IMU object
      if (!bno.begin())
      {
        PISerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
      }
    }
    sensor_event_t getData(){
      sensors_event_t orientationData , linearAccelData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      return orientationData;
    }

}