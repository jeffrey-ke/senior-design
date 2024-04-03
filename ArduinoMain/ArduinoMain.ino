#include "MasterComputeBridge.h"
#include "Constants.h"
void setup() {
  PISerial.begin(115200);
  //DebugSerial.begin(9600);
  PISerial.setTimeout(10); //avoid long delays to mistimed reads
  MasterComputeBridge bridge; 
  delay(5000); // delay to allow the ESC to recognize the stopped signals
  DebugSerial.println("Initialization Done");
  uint32_t timer = millis();
  
  while(true){
    //Must spin GPS at rate it is filled to clear serial buffer
    if (millis() - timer >= 100) {
      timer = millis();
      bridge.spinGPS();
    }
    if(PISerial.available()>0){
      String data = PISerial.readStringUntil('\n');
      DebugSerial.print("You sent me: ");
      DebugSerial.println(data);
      if(data=="K"){ return; }
      else if(data=="S:"){ 
        PISerial.println("S:1");
      }
      else{
        bridge.giveCommand(data);
        PISerial.println(bridge.returnCommand());
      }
    }
  }
}

void loop() {} //running all code in setup cause we can't use global variables 

