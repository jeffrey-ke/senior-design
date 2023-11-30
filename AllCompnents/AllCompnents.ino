/* This program is a basic rundown of the function of the Arduino
   It reads all of the relevant sensor data and sends it to the Pi
   over the Serial connection. The Pi then processes this data and 
   sends commands back to the Arduino.
*/
#include <Servo.h> //Maybe need to use servo timer 2 library https://forum.arduino.cc/t/error-multiple-definition-of-__vector_17-using-multiple-libraries/637963
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>
#include "ping1d.h"
#include "SoftwareSerial.h"

//Pinouts
#define PISerial Serial
#define GPSSerial Serial1
#define PingSerial Serial2
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 915.0 //Standard frequency for use in the US
#define servoPin1 7
#define servoPin2 6
#define servoPin3 9
#define servoPin4 8

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver

Servo thruster1;//Thruster objects
Servo thruster2;
Servo thruster3;
Servo thruster4;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //IMU object

static Ping1D ping { PingSerial }; //Ping Sensor Object

uint16_t MIN_DELAY = 100; //Milliseconds between updates
uint16_t PRINT_DELAY = 2000; //Milliseconds between printing to PISerial
uint16_t PRINT_COUNTER = 0;

void setup() {
  // Initialize serial communication with Pi
  while (!Serial);
  PISerial.begin(115200);
  
  //Intitialize Lora
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  PISerial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    PISerial.println("setFrequency failed");
    while (1);
  }
  PISerial.print("Set Freq to: "); PISerial.println(RF95_FREQ);

  rf95.setTxPower(23, false);

  //Initialize IMU
  if (!bno.begin())
  {
    PISerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //GPS Initialization
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  //Initialize Ping Sensor
  PingSerial.begin(9600);
  if(!ping.initialize()) {
        PISerial.println("\nPing device failed to initialize!");
        PISerial.println(PingSerial);
  }

  //Initialize thrusters
  thruster1.attach(servoPin1);
  thruster2.attach(servoPin2);

	thruster1.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster2.writeMicroseconds(1500); // send "stop" signal to ESC.

	delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  unsigned long tStart = micros(); //get time
  //Check command from PI
  while(PISerial.available() > 0) {
    String data = PISerial.readStringUntil('\n');
    PISerial.print("You sent me: ");
    PISerial.println(data);
    parseCommands(data);
  }

  //Get all sensor data and send it to PI
  if(PRINT_COUNTER >= PRINT_DELAY/MIN_DELAY){
  PRINT_COUNTER = 0;
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  PISerial.print("X Orientation: ");
  PISerial.println(orientationData.orientation.x);
  PISerial.print("Y Orientation: ");
  PISerial.println(orientationData.orientation.y);
  PISerial.print("Z Orientation: ");
  PISerial.println(orientationData.orientation.z);
  PISerial.println();

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    PISerial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  PISerial.print("Fix: "); Serial.print((int)GPS.fix);
  PISerial.print(" quality: "); Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    PISerial.print("Location: ");
    PISerial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    PISerial.print(", ");
    PISerial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    PISerial.print("Speed (knots): "); Serial.println(GPS.speed);
    PISerial.print("Angle: "); Serial.println(GPS.angle);
    PISerial.print("Altitude: "); Serial.println(GPS.altitude);
    PISerial.print("Satellites: "); Serial.println((int)GPS.satellites);
    PISerial.print("Antenna status: "); Serial.println((int)GPS.antenna);
  }
  
  if (ping.update()) {
        PISerial.print("Distance: ");
        PISerial.print(ping.distance());
        PISerial.print("\tConfidence: ");
        PISerial.println(ping.confidence());
  }

  //If lora message is recieved send it to PI
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.recv(buf, &len))
  {
     //RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got from Lora: ");
      Serial.println((char*)buf);
  }
  }

  //Instead of having set delay rate will include executione time
  while ((micros() - tStart) < (MIN_DELAY * 1000))
  {  }
}
/*This function takes the string sent over by the PI over the Serial connection
  and parses it into the distinct commands. Need to define some protocol for this 
  communication such as seperating a list of commands with commas
*/
void parseCommands(String commandList){
 int start, end = 0;
  do {
    end = commandList.indexOf(",", start);
    parseCommand(commandList.substring(start, end - start));
    start = end + 1;
  } while (end != -1);
}
void parseCommand(String command){
  int seperator = command.indexOf(":");
  if(command.substring(0,seperator) == "Thrusters"){
      //Parse and execute thruster command
  }
  else if(command.substring(0,seperator) == "Radio"){
      //Parse and execute radio command
  }
  else if(command.substring(0,seperator) == "EStop"){
      //Parse and execute EStop command
  }
  else {
    PISerial.print("Unable to parse command");
  } 
}
//Set value of a thruster
void thrusterCommand(int thrusterId, int value){
  switch(thrusterId){
    case 1:
      thruster1.writeMicroseconds(value);
      break;
    case 2:
      thruster2.writeMicroseconds(value);
      break;
    case 3:
      thruster3.writeMicroseconds(value);
      break;
    case 4:
      thruster4.writeMicroseconds(value);
      break;
  }
}
void radioCommand(uint8_t message[]){
  rf95.send(message, sizeof(message));
  rf95.waitPacketSent();
  PISerial.println("Sent a reply");
}

