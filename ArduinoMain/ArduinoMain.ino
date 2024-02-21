
#define IS_TEST 1
#define IS_ACTUAL 0

#if IS_ACTUAL
#include "MasterComputeBridge.h"
#include "Constants.h"
#include "WaypointController.h"
#include "StateMachine.h"


StateMachine sm(1, 0, 0, 0,
                1, 0, 0, 0,
                5);
void setup() {
  // put your setup code here, to run once:
  PISerial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
    sm.DecideState();
    sm.ExecuteState();
    auto pwm = sm.GetCommandedPWM();
    PISerial.print("FL: ");
    PISerial.println(pwm.FL);
    PISerial.print("FR: ");
    PISerial.println(pwm.FR);
    PISerial.print("DL: ");
    PISerial.println(pwm.DL);
    PISerial.print("DR: ");
    PISerial.println(pwm.DR);
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

