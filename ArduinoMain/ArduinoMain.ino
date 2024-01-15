#include "MasterComputeBridge.h"
#include "Constants.h"


void setup() {
  // put your setup code here, to run once:
  DebugSerial.begin(115200);
  MasterComputeBridge bridge; 
  //bridge = MasterComputeBridge();
  DebugSerial.println("Initialization Done");
  
  while(true){
    while(DebugSerial.available() > 0) {
      String data = DebugSerial.readStringUntil('\n');
      DebugSerial.print("You sent me: ");
      DebugSerial.println(data);
      bridge.giveCommand(data);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
