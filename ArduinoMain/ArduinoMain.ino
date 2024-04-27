
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

#define PRESSURE_GOOD true
#define PRESSURE_BAD false

void setup() {
    Serial.begin(115200);
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
            DiveTest(params.Kp, params.Ki, params.Kd, 
                    params.Kp2, params.Ki2, params.Kd2, 
                    params.duration, params.target_depth);
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
        degrees heading = (static_cast<int>(imu_.GetData().x) + 360) % 360;
        o_con_.SetDesiredOrientation(bearing, OrientationController::YAW);
        auto pwm = Msg::PWM{};
        if (goal_point - current_loc < distance_threshold) {
            pwm = Msg::pwm_FULL_OFF;
            CommandThrusterAt(pwm);
            goal_point = home_coords;
            waypoint_id = 1;
        }
        pwm = o_con_.CalculateControlEffort(Msg::RPY{heading, 0, 0}) + Msg::pwm_FULL_FORWARD;
        CommandThrusterAt(pwm);
        Serial.print(millis()); Serial.print(",");
        Serial.print(waypoint_id); Serial.print(",");
        Serial.print(goal_point.lat, 7); Serial.print(",");
        Serial.print(goal_point.lon, 7); Serial.print(",");
        Serial.print(goal_point - current_loc); Serial.print(",");
        Serial.print(bearing); Serial.print(",");
        Serial.print(current_loc.lat, 7); Serial.print(",");
        Serial.print(current_loc.lon, 7); Serial.print(",");
        Serial.print(heading); Serial.print(",");
        Serial.print(pwm.FL); Serial.print(",");
        Serial.print(pwm.FR); Serial.print(",");
        Serial.print(pwm.DL); Serial.print(",");
        Serial.println(pwm.DR);  
    }
    Serial.println("done");
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

void DiveTest(double Kp_dive, double Ki_dive, double Kd_dive,
            double Kp_o, double Ki_o, double Kd_o, 
            milliseconds duration, meters depth) {
    meters surface_depth{0};
    // Depth con gains should be LOW!!! Especially integral gain!!
    depth_con_.SetGains(Kp_dive, Ki_dive, Kd_dive);
    o_con_.SetGains(Kp_o, Ki_o, Kd_o, OrientationController::PITCH);
    depth_con_.SetDesiredDepth(depth);
    o_con_.SetDesiredToVertical();
    Timer t(duration);
    while (!t.IsExpired()) {
        auto pwm_orientation = o_con_.CalculateControlEffort(imu_.GetData());
        Msg::PWM pwm_depth;
        if (o_con_.IsVertical(imu_.GetData(), 10)) {
            pwm_depth = depth_con_.CalculateControlEffort(depth_sensor_.GetDepth());
        } else {
            depth_con_.ResetIntegratedError();
            pwm_depth = Msg::pwm_FULL_OFF;
        }
        auto pwm = pwm_depth + pwm_orientation;
        CommandThrusterAt(pwm);
        Serial.print(depth_sensor_.GetDepth()); Serial.print(",");
        Serial.print(imu_.GetData().x); Serial.print(",");
        Serial.print(imu_.GetData().y); Serial.print(",");
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
        Serial.print(imu_.GetData().x); Serial.print(",");
        Serial.print(imu_.GetData().y); Serial.print(",");
        Serial.print(imu_.GetData().z); Serial.print(",");
        Serial.print(depth_sensor_.GetDepth()); Serial.print(",");
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



