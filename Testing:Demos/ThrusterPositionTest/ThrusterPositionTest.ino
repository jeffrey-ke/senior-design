#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

byte servoPin1 = 4;
byte servoPin2 = 5;
Servo thruster1;
Servo thruster2;

uint16_t MIN_DELAY = 100; //Milliseconds between updates

uint16_t PRINT_DELAY = 500; //Milliseconds between printing to serial

double xPos = 0, yPos = 0, headingVel = 0;
double ACCEL_VEL_TRANSITION =  (double)(MIN_DELAY) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

//Should be address 28 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


void setup() {
  Serial.begin(9600); // Initialize serial communication

  thruster1.attach(servoPin1);
  thruster2.attach(servoPin2);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

	thruster1.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster2.writeMicroseconds(1500); // send "stop" signal to ESC.

	delay(7000); // delay to allow the ESC to recognize the stopped signal
}
int printCount = 0;
void loop() {
  
  unsigned long tStart = micros(); //get time
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  int sensorValue = analogRead(A1); 
  //Pentiometer outputs values from 0 to 1023, thruster takes in values from 1100 to 1900
  //where 1500 is stopped
  int value = map(sensorValue, 0, 1023, 1100, 1900);
  if(value>1450&&value<1550){ //make stopping it easier
    value = 1500;
  }

  //Print Block
  if (printCount * MIN_DELAY >= PRINT_DELAY) {
    //enough iterations have passed that we can print the latest data
    Serial.print("X Orientation: ");
    Serial.println(orientationData.orientation.x);
    Serial.print("Y Orientation: ");
    Serial.println(orientationData.orientation.y);
    Serial.print("Z Orientation: ");
    Serial.println(orientationData.orientation.z);
    Serial.print("Position: ");
    Serial.print(xPos);
    Serial.print(" , ");
    Serial.println(yPos);
    Serial.print("Speed: ");
    Serial.println(headingVel);
    Serial.println("-------");
    char buffer[8];
    if(value > 1500){
      Serial.print("Going forward at "); 
      double speed = ((double)(value-1500))/4.0;
      dtostrf(speed, 5, 0, buffer);
      Serial.print(buffer);
      Serial.println("% speed");
    }
    else if(value < 1500){
      Serial.print("Going backward at ");
      double speed = ((double)(1500-value))/4.0;
      dtostrf(speed, 5, 0, buffer);
      Serial.print(buffer);
      Serial.println("% speed");
    }    
    else{
      Serial.println("Thrusters stopped");
    }

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }
  thruster1.writeMicroseconds(value); // Send signal to ESC.
  thruster2.writeMicroseconds(value); // Send signal to ESC.
  //Instead of having set delay rate will include executione time
  while ((micros() - tStart) < (MIN_DELAY * 1000))
  {  }
}
