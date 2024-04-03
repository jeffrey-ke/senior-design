#include "OrientationController.h"

void OrientationController::SetDesiredToVertical() {
    con_.SetDesired(Msg::rpy_VERTICAL.z);
}

void OrientationController::SetDesiredToHorizontal() {
    con_.SetDesired(Msg::rpy_HORIZONTAL.z);
}

void OrientationController::SetDesiredOrientation(Msg::RPY des){
    con_.SetDesired(des.z);
};

Msg::PWM OrientationController::CalculateControlEffort(Msg::RPY current_orientation) {
    auto effort = static_cast<int>(con_.CalculateControlEffort(current_orientation.z) / 2); 
    return Msg::PWM{1500, 1500, 1500 + effort, 1500 + effort};
}

void OrientationController::SetGains(double kp, double ki, double kd) {
    con_.SetKp(kp);
    con_.SetKi(ki);
    con_.SetKd(kd);
}

bool OrientationController::IsVertical(Msg::RPY current_orientation, double margin) {
    return fabs(current_orientation.z - 90) < margin;
}