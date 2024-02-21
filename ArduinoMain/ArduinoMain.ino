#include "MasterComputeBridge.h"
#include "Constants.h"
uint16_t MIN_DELAY = 1000;
bool startup = false;
uint32_t timer = millis();
void setup() {
  // put your setup code here, to run once:
  PISerial.begin(115200);
  DebugSerial.begin(9600);
  PISerial.setTimeout(100);
  MasterComputeBridge bridge; 
  PISerial.println("Initialization Done");
  startup = true;
  while(true){
    bridge.spinGPS();
    if (millis() - timer > 100) {
      String data = PISerial.readStringUntil('\n');
      timer = millis(); // reset the timer
      if(data.length()>0){
        DebugSerial.print("You sent me: ");
        DebugSerial.println(data);
        if(data=="K"){ return; }
        else if(data=="S"){ 
          if(startup){ PISerial.println("1"); }
          else{ PISerial.println("0"); }
        }
        else if(data=="yo"){
          PISerial.println("Hey whats up?");
        }
        else{
          bridge.giveCommand(data);
          PISerial.println(bridge.returnCommand());
        }
      }
      
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
