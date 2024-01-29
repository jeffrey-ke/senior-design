#include "MasterComputeBridge.h"
#include "Constants.h"

void setup() {
  // put your setup code here, to run once:
  PISerial.begin(115200);
  MasterComputeBridge bridge; 
  //bridge = MasterComputeBridge();
  DebugSerial.println("Initialization Done");
  
  while(true){
    bridge.giveCommand("G:");
    PISerial.println(bridge.returnCommand());
    delay(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
