#include "DepthController.h"

void DepthController::SetDesiredDepth(meters depth) {
    if (depth > max_depth_){
        con_.SetDesired(max_depth_);
    } else 
        con_.SetDesired(depth);
}

void DepthController::SetGains(double kp, double ki, double kd) {
    con_.SetKp(kp);
    con_.SetKi(ki);
    con_.SetKd(kd);
}

Msg::PWM DepthController::CalculateControlEffort(meters current_depth) {
    auto effort = static_cast<int>(con_.CalculateControlEffort(current_depth) / 2); 
    return Msg::PWM{1500 + effort, 1500 + effort, 1500, 1500};
}

void DepthController::ResetIntegratedError() {
    con_.ResetIntegratedError();
}
