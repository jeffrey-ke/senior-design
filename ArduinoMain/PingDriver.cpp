#include "Arduino.h”
#include "PingDriver.h"

PingDriver::PingDriver(){
    ping = new Ping1D(PingSerial);
    if(!ping.initialize()) {
        DebugSerial.println("\nPing device failed to initialize!");
        DebugSerial.println(PingSerial);
    }
}
int PingDriver::getData(){
    if (ping.update()) {
        return ping.distance());
        //ping.confidence();
    }
    else return -1;
}