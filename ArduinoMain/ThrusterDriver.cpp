#include "Arduino.h"
#include "ThrusterDriver.h"

ThrusterDriver::ThrusterDriver(int servoPin){
    thruster.attach(servoPin);

    thruster.writeMicroseconds(1500); // send "stop" signal to ESC.

    delay(7000); // delay to allow the ESC to recognize the stopped signal
}
void ThrusterDriver::setVelocity(int vel){
    thruster.writeMicroseconds(vel);
}

