#include "ThrusterDriver.h"

ThrusterDriver::ThrusterDriver(){
  DebugSerial.println("No Pin provided for thruster");
}
ThrusterDriver::ThrusterDriver(const int servoPin){
    thruster.attach(servoPin);

    thruster.writeMicroseconds(1500); // send "stop" signal to ESC.

    delay(7000); // delay to allow the ESC to recognize the stopped signal
    DebugSerial.print("Thruster Initialized on Pin ");
    DebugSerial.println(servoPin);
}
void ThrusterDriver::setVelocity(int vel){
    thruster.writeMicroseconds(vel);
    DebugSerial.print("Wrote ");
    DebugSerial.print(vel);
    DebugSerial.println(" to thruster");
}

