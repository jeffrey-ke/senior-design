#include "WaypointController.h"

double WaypointController::LINEAR_MARGIN_OF_ERROR = 1.0; //meters
double WaypointController::ANGULAR_MARGIN_OF_ERROR = 0.1; //radians

WaypointController::PWM WaypointController::CalculatePWM(Location current_loc, double current_heading) {
    auto l_pwm = CalculateLinearPWM(current_loc);
    auto a_pwm = CalculateAngularPWM(current_loc, current_heading);

    if (IsHeadingCorrectWithinMargin(current_loc, current_heading)) {
        return l_pwm;
    } else {
        return a_pwm;
    }
}

WaypointController::PWM WaypointController::CalculateLinearPWM(Location current_loc) {
    auto current_distance = CalculateDistanceBetween(current_loc, desired_);
    auto linear_effort_per_thruster =  static_cast<int>(linear_controller_.CalculateControlEffort(current_distance) / 2);
    return PWM{1500 + linear_effort_per_thruster, 1500 + linear_effort_per_thruster, 1500, 1500};
}

WaypointController::PWM WaypointController::CalculateAngularPWM(Location current_loc, double current_heading) {
    auto current_angular_difference = current_heading - CalculateAngleDifferenceBetween(desired_, current_loc);
    auto angular_effort_per_thruster = static_cast<int>(angular_controller_.CalculateControlEffort(current_angular_difference) / 2);
    return PWM{1500 - angular_effort_per_thruster, 1500 + angular_effort_per_thruster, 1500, 1500};
}

bool WaypointController::IsHeadingCorrectWithinMargin(Location current_loc, double current_heading) {
    return current_heading - CalculateAngleDifferenceBetween(desired_, current_loc) < ANGULAR_MARGIN_OF_ERROR;
}

bool WaypointController::IsLocationCorrectWithinMargin(Location current_loc) {
    return CalculateDistanceBetween(current_loc, desired_) < LINEAR_MARGIN_OF_ERROR;
}
