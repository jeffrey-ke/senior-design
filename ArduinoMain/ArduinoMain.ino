
#define IS_TEST 1
#define IS_ACTUAL 0

#if IS_ACTUAL
#include "MasterComputeBridge.h"
#include "Constants.h"
#include "WaypointController.h"


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
#elif IS_TEST
#include "_tests.h"
#include "_StateMachineTests.h"
void setup() {
// put your setup code here, to run once:
    Serial.begin(115200);
}

void loop() {
// put your main code here, to run repeatedly:
    Test::run();
}
#endif

