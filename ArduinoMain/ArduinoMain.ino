#include "MasterComputeBridge.h"
#include "Constants.h"
uint16_t MIN_DELAY = 1000;
void setup() {
  // put your setup code here, to run once:
  DebugSerial.begin(115200);
  MasterComputeBridge bridge; 
  //bridge = MasterComputeBridge();
  DebugSerial.println("Initialization Done");
  
  while(true){
    unsigned long tStart = micros(); //get time
    String data = PISerial.readStringUntil('\n');
    DebugSerial.print("You sent me: ");
    DebugSerial.println(data);
    if(data.length()>0){
      bridge.giveCommand(data);
      PISerial.println(bridge.returnCommand());
    }
    bridge.giveCommand("G:");
    PISerial.println(bridge.returnCommand());
    bridge.giveCommand("I:");
    PISerial.println(bridge.returnCommand());
    bridge.giveCommand("P:");
    PISerial.println(bridge.returnCommand());
    while ((micros() - tStart) < (MIN_DELAY * 1000))
    {  }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
