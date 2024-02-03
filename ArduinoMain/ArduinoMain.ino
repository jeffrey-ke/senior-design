#include "MasterComputeBridge.h"
#include "Constants.h"
uint16_t MIN_DELAY = 1000;
void setup() {
  // put your setup code here, to run once:
  PISerial.begin(115200);
  //DebugSerial.begin(9600);
  MasterComputeBridge bridge; 
  //bridge = MasterComputeBridge();
  PISerial.println("Initialization Done");
  
  while(true){
    unsigned long tStart = micros(); //get time
    String data = PISerial.readStringUntil('\n');  
    if(data.length()>0){
      DebugSerial.print("You sent me: ");
      DebugSerial.println(data);
      if(data=="K"){ return; }
      bridge.giveCommand(data);
      PISerial.println(bridge.returnCommand());
    }
    bridge.giveCommand("G:");
    PISerial.println(bridge.returnCommand());
   /* bridge.giveCommand("I:");
    PISerial.println(bridge.returnCommand());
    bridge.giveCommand("P:");
    PISerial.println(bridge.returnCommand());*/
    while ((micros() - tStart) < (MIN_DELAY * 1000))
    {  }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
