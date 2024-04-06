
#define UNIT_TEST 0
#define IS_ACTUAL 0
#define FLIP_TEST 1

#if IS_ACTUAL
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

void loop() {} //running all code in setup cause we can't use global variables  }

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
#include "BarometerDriver.h"
#include "MS5837Driver.h"
#include "DepthController.h"
#include "SerialParser.h"
#include "units.h"
#include "timer.h"
#include <Servo.h>
#include "Constants.h"
#include "Msgs.h"
OrientationController o_con_(10, 0, 0);
DepthController depth_con_(4, 0.1, 0, 10);
IMUDriver imu_;
BarometerDriver baro_(5);
MS5837Driver depth_sensor_;
Servo FL, FR, DL, DR;


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
    FL.attach(FL_PIN);
    FR.attach(FR_PIN);
    DL.attach(DL_PIN);
    DR.attach(DR_PIN);
    StopThrusters();
    delay(7000);
    StopThrusters();
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
        StopThrusters();
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
        StopThrusters();
    }
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
    Serial.println("done");

    return PRESSURE_GOOD;
}

void DiveTest(milliseconds duration, meters depth, double Kp, double Ki, double Kd) {
    meters surface_depth{0};
    depth_con_.SetDesiredDepth(depth);
    depth_con_.SetGains(Kp, Ki, Kd);
    o_con_.SetDesiredToVertical();
    Timer t(duration);
     
    while (!t.IsExpired()) {
        auto pwm_orientation = o_con_.CalculateControlEffort(imu_.GetData());
        auto pwm_depth = (o_con_.IsVertical(imu_.GetData()))? depth_con_.CalculateControlEffort(depth_sensor_.GetDepth()) : Msg::PWM{};
        auto pwm = pwm_depth.SaturatePWM(pwm_depth, pwm_orientation);
        CommandThrusterAt(pwm);
        Serial.print(depth_sensor_.GetDepth()); Serial.print(",");
        Serial.print(imu_.GetData().z); Serial.print(",");
        Serial.print(pwm.FL); Serial.print(",");
        Serial.print(pwm.FR); Serial.print(",");
        Serial.print(pwm.DL); Serial.print(",");
        Serial.print(pwm.DR); Serial.print("\n");
        delay(100);
    }
    Serial.println("done");
}

void FlipTest(milliseconds duration, double Kp, double Ki, double Kd) {
    o_con_.SetDesiredToVertical();
    o_con_.SetGains(Kp, Ki, Kd);
    Timer t(duration);
    while (!t.IsExpired()) {
        auto pwm = o_con_.CalculateControlEffort(imu_.GetData());
        CommandThrusterAt(pwm);
        Serial.print(imu_.GetData().z); Serial.print(",");
        Serial.print(pwm.FL); Serial.print(",");
        Serial.print(pwm.FR); Serial.print(",");
        Serial.print(pwm.DL); Serial.print(",");
        Serial.print(pwm.DR); Serial.print("\n");
        delay(100);
    }
    Serial.println("done");
}

void CommandThrusterAt(const Msg::PWM& pwm) {
    FL.writeMicroseconds(pwm.FL);
    FR.writeMicroseconds(pwm.FR);
    DL.writeMicroseconds(pwm.DL);
    DR.writeMicroseconds(pwm.DR);
}

void StopThrusters() {
    FL.writeMicroseconds(1500);
    FR.writeMicroseconds(1500);
    DL.writeMicroseconds(1500);
    DR.writeMicroseconds(1500);
}
#endif


