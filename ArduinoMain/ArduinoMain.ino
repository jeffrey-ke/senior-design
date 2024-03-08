
#define UNIT_TEST 0
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
#elif UNIT_TEST
#include "_depth_unittests.h"
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
#include "IMUDriver.h"
#include "ThrusterDriver.h"
#include "BarometerDriver.h"
#include "MS5837Driver.h"
#include "DepthController.h"
#include "SerialParser.h"
#include "units.h"
#include "timer.h"

OrientationController o_con_(10, 0, 0);
DepthController depth_con_(4, 0.1, 0, 10);
IMUDriver imu_;
BarometerDriver baro_(5);
MS5837Driver depth_sensor_;


void FlipTest(milliseconds duration, double Kp, double Ki, double Kd);
void DiveTest(milliseconds duration, meters depth, double Kp, double Ki, double Kd);
bool PressureTest(milliseconds duration, mmHg maximum_deviation=1.0);
#define PRESSURE_GOOD true
#define PRESSURE_BAD false

void setup() {
    Serial.begin(115200);
    imu_.Init();
    baro_.Init();
    depth_sensor_.Init();
}

void loop() {
    while (true) {
        if (Serial.available()) {
            String line{Serial.readStringUntil('\n')};
            if (line == "Pi Ready") {
                Serial.println("Ready");
                break;
            }
        }
    }
    SerialParser parser;
    while (true) {
        TestParams params;
        if (Serial.available()) {
            String line{Serial.readStringUntil('\n')};
            params = parser.ParseLine(line);
        }
        switch (params.type)
        {
        case PRESSURE:
            PressureTest(params.duration, params.max_deviation);
        break;
        case DIVE:
            DiveTest(params.duration, params.target_depth, params.Kp, params.Ki, params.Kd);
        break;
        case FLIP:
            FlipTest(params.duration, params.Kp, params.Ki, params.Kd);
        break;
        }
        params.Reset();
    }
    
    return;
}

bool PressureTest(milliseconds duration, mmHg maximum_deviation) {
    Timer t(duration);
    auto starting_pressure = baro_.GetAvgPressure();
    while (!t.IsExpired()) {
        Serial.println(baro_.GetAvgPressure());
        if (fabs(baro_.GetAvgPressure() - starting_pressure) > maximum_deviation) {
            Serial.println("*********Critical pressure leak!!!*********");
            return PRESSURE_BAD;
        }
        delay(100);
    }
    return PRESSURE_GOOD;
}

void DiveTest(milliseconds duration, meters depth, double Kp, double Ki, double Kd) {
    meters surface_depth{0};
    depth_con_.SetDesiredDepth(depth);
    Timer t(duration);
    while (!t.IsExpired()) {
        auto pwm = depth_con_.CalculateControlEffort(depth_sensor_.GetDepth());
        Serial.print(depth_sensor_.GetDepth()); Serial.print(",");
        Serial.print(imu_.GetData().z); Serial.print(",");
        Serial.print(pwm.FL); Serial.print(",");
        Serial.print(pwm.FR); Serial.print(",");
        Serial.print(pwm.DL); Serial.print(",");
        Serial.print(pwm.DR); Serial.print("\n");
        delay(100);
    }
    depth_con_.SetDesiredDepth(surface_depth);
}

void FlipTest(milliseconds duration, double Kp, double Ki, double Kd) {
    o_con_.SetDesiredToVertical();
    Timer t(duration);
    while (!t.IsExpired()) {
        auto pwm = o_con_.CalculateControlEffort(imu_.GetData());
        Serial.print(imu_.GetData().z); Serial.print(",");
        Serial.print(pwm.FL); Serial.print(",");
        Serial.print(pwm.FR); Serial.print(",");
        Serial.print(pwm.DL); Serial.print(",");
        Serial.print(pwm.DR); Serial.print(",");
        delay(100);
    }

}



#endif

