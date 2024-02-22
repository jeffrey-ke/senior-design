
#define IS_TEST 0
#define IS_ACTUAL 0
#define FLIP_TEST 1

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
#elif FLIP_TEST
#include "OrientationController.h"
#include "Controller.h"
#include "IMUDriver.h"


void setup() {
    Serial.begin(9600);
    OrientationController o_con_(10.0, 0.0, 0.0);
    o_con_.SetDesiredToVertical();
    auto timer = millis();
    Serial.println("Starting test.");
    while (millis() - timer < 60000) {
        auto pwm = o_con_.CalculateControlEffort();
        Serial.print("Current orientation: ");
        Serial.println(o_con_.GetCurrentOrientation().z);
        Serial.print("Desired orientation: ");
        Serial.println(o_con_.GetDesiredOrientation());
        Serial.print("FL: ");
        Serial.println(pwm.FL);
        Serial.print("FR: ");
        Serial.println(pwm.FR);
        Serial.print("DL: ");
        Serial.println(pwm.DL);
        Serial.print("DR: ");
        Serial.println(pwm.DR);
        delay(1000);
    }
}

void loop() {

}

#endif

