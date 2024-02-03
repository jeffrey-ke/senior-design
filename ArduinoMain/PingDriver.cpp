#include "PingDriver.h"

PingDriver::PingDriver(){
  PingSerial.begin(9600);
    if(!ping.initialize()) {
        DebugSerial.println("\nPing device failed to initialize!");
        DebugSerial.println(PingSerial);
    }
    else{
      DebugSerial.println("Ping initialized succesfully");
    }
}
int PingDriver::getData(){
    if (ping.update()) {
        return ping.distance();
        //ping.confidence();
    }
    else return -1;
}