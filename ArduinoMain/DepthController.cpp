#include "DepthController.h"

void DepthController::SetDesiredDepth(meters depth) {
    if (depth > max_depth_){
        con_.SetDesired(max_depth_);
    } else 
        con_.SetDesired(depth);
}

Msg::PWM DepthController::CalculateControlEffort(meters current_depth) {
    auto effort = static_cast<int>(con_.CalculateControlEffort(current_depth) / 2); 
    return Msg::PWM{1500 + effort, 1500 + effort, 1500, 1500};
}