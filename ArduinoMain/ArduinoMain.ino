
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
#include "_GPSDriver.h"
OrientationController o_con_;
DepthController depth_con_(4, 0.1, 0, 10);
IMUDriver imu_;
BarometerDriver baro_(5);
MS5837Driver depth_sensor_;
Servo FL, FR, DL, DR;
_GPSDriver gps_;


void FlipTest(milliseconds duration, double Kp, double Ki, double Kd);
void DiveTest(milliseconds duration, meters depth, double Kp, double Ki, double Kd);
bool PressureTest(milliseconds duration, mmHg maximum_deviation=1.0);
#define PRESSURE_GOOD true
#define PRESSURE_BAD false

void setup() {
    Serial.begin(115200);
    Serial.println("Hello");
    imu_.Init();
    baro_.Init();
    gps_.Init();
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
    Serial.println("Hello");
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
        case FUN:
            FlipUnflipTest(params.duration, params.Kp, params.Ki, params.Kd, params.duration2, params.forward_pwm);
        break;
        case WAYPOINT:
            WaypointTest(params.duration, params.Kp, params.Ki, params.Kd, params.distance_threshold, params.heading_threshold, params.goal_point);
        break;
        }
        params.Reset();
        StopThrusters();
    }
}

void WaypointTest(milliseconds duration, double kp_a, double ki_a, double kd_a, 
                    meters distance_threshold, degrees heading_threshold, Msg::GNSS goal_point) {
    Msg::GNSS home_coords = gps_.GetGNSS();
    o_con_.SetGains(kp_a, ki_a, kd_a, OrientationController::YAW);
    Timer timer_(duration);
    int waypoint_id{0};
    // profiler gets duration seconds to get to the waypoint and back.
    while (!timer_.IsExpired()) {
        Msg::GNSS current_loc = gps_.GetGNSS();
        degrees bearing = goal_point % current_loc;
        degrees heading = imu_.GetData().x;
        o_con_.SetDesiredOrientation(bearing, OrientationController::YAW);
        auto pwm = Msg::PWM{};
        if (goal_point - current_loc < distance_threshold) {
            pwm = Msg::pwm_FULL_OFF;
            CommandThrusterAt(pwm);
            goal_point = home_coords;
            waypoint_id = 1;
        }
        if (!o_con_.IsAngleAchieved(heading, 5, OrientationController::YAW)) {
            pwm = o_con_.CalculateControlEffort(Msg::RPY{heading, 0, 0});
            CommandThrusterAt(pwm);
        } 
        else if (goal_point - current_loc > distance_threshold){
            pwm = Msg::pwm_FULL_FORWARD;
            CommandThrusterAt(pwm);
        }
        char data_line[80];// time,lat,lon,bearing,heading,FL,FR,DL,DR
        snprintf(data_line, 
                78, 
                "%d,%d,%.7f,%.7f,%.2f,%.7f,%.7f,%.2f,%d,%d,%d,%d\n", 
                millis(),
                waypoint_id,
                goal_point.lat,
                goal_point.lon,
                bearing,
                current_loc.lat,
                current_loc.lon,
                heading,
                pwm.FL,
                pwm.FR,
                pwm.DL,
                pwm.DR
                );
        Serial.println(data_line);
        
    }
}

/**
 * Duration_forward should be at MOST 10000 ms
*/
void FlipUnflipTest(milliseconds duration_vertical, double Kp, double Ki, double Kd,  
                    milliseconds duration_forward, int pwm_forward) {
    o_con_.SetGains(Kp, Ki, Kd, OrientationController::PITCH);


    Serial.println("Flipping");
    o_con_.SetDesiredToVertical();
    Timer timer_vertical(duration_vertical);
    while (!timer_vertical.IsExpired()) {
        auto pwm = o_con_.CalculateControlEffort(imu_.GetData());
        CommandThrusterAt(pwm);
        Serial.print(imu_.GetData().z); Serial.print(",");
        Serial.print(pwm.FL); Serial.print(",");
        Serial.print(pwm.FR); Serial.print(",");
        Serial.print(pwm.DL); Serial.print(",");
        Serial.print(pwm.DR); Serial.print("\n");
        delay(100);
    }
// ============================================================
    Serial.println("Unflipping and moving forward");
    o_con_.SetDesiredToHorizontal();
    Timer timer_horizontal(duration_forward);
    while (!timer_horizontal.IsExpired()) {
        auto pwm = o_con_.CalculateControlEffort(imu_.GetData());
        pwm.FL = pwm_forward;
        pwm.FR = pwm_forward;
        CommandThrusterAt(pwm);
        Serial.print(imu_.GetData().z); Serial.print(",");
        Serial.print(pwm.FL); Serial.print(",");
        Serial.print(pwm.FR); Serial.print(",");
        Serial.print(pwm.DL); Serial.print(",");
        Serial.print(pwm.DR); Serial.print("\n");
        delay(100);
    }
// ============================================================
    Serial.println("Flipping again");
    o_con_.SetDesiredToVertical();
    timer_vertical.Reset();
    while (!timer_vertical.IsExpired()) {
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
        auto pwm_depth = (o_con_.IsVertical(imu_.GetData()), 5)? depth_con_.CalculateControlEffort(depth_sensor_.GetDepth()) : Msg::PWM{};
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
    o_con_.SetGains(Kp, Ki, Kd, OrientationController::PITCH);
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



