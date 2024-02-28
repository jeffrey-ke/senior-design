#include "MasterComputeBridge.h"
#include "Constants.h"
bool startup = false;
uint32_t timer = millis();
void setup() {
  PISerial.begin(115200);
  //DebugSerial.begin(9600);
  PISerial.setTimeout(10); //avoid long delays to mistimed reads
  MasterComputeBridge bridge; 
  DebugSerial.println("Initialization Done");
  startup = true;

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
      else if(data=="S"){ 
        if(startup){ PISerial.println("1"); }
        else{ PISerial.println("0"); }
      }
      else{
        bridge.giveCommand(data);
        PISerial.println(bridge.returnCommand());
      }
    }
  }
}

void loop() { //running all code in setup cause we can't use global variables  }
