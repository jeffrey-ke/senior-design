#include "OrientationController.h"

void OrientationController::SetDesiredToVertical() {
    con_z_.SetDesired(Msg::rpy_VERTICAL.z);
}

void OrientationController::SetDesiredToHorizontal() {
    con_z_.SetDesired(Msg::rpy_HORIZONTAL.z);
}

void OrientationController::SetDesiredOrientation(degrees desired, AXES axis){
    switch (axis)
    {
    case AXES::X:
        con_x_.SetDesired(desired);
    break;
    case AXES::Y:
        con_y_.SetDesired(desired);
    break;
    case AXES::Z:
        con_z_.SetDesired(desired);
    break;
    }
};

Msg::PWM OrientationController::CalculateControlEffort(Msg::RPY current_orientation) {
    auto effort_z_ = static_cast<int>(con_z_.CalculateControlEffort(current_orientation.z) / 2);  //pitch
    auto effort_x_ = static_cast<int>(con_x_.CalculateControlEffort(current_orientation.x) / 2); // yaw
    auto effort_y_ = static_cast<int>(con_y_.CalculateControlEffort(current_orientation.y) / 2); // roll
    auto pwm_z = Msg::PWM{1500, 1500, 1500 + effort_z_, 1500 + effort_z_};
    auto pwm_x = Msg::PWM{1500 + effort_x_, 1500 - effort_x_, 1500, 1500};
    auto pwm_y = Msg::PWM{1500, 1500, 1500 + effort_y_, 1500 - effort_y_};
    return pwm_x + pwm_y + pwm_z;
}

void OrientationController::SetGains(double kp, double ki, double kd, AXES axis) {
    switch (axis)
    {
    case AXES::X:
        con_x_.SetKp(kp);
        con_x_.SetKi(ki);
        con_x_.SetKd(kd);
    break;
    case AXES::Y:
        con_y_.SetKp(kp);
        con_y_.SetKi(ki);
        con_y_.SetKd(kd);
    break;
    case AXES::Z:
        con_z_.SetKp(kp);
        con_z_.SetKi(ki);
        con_z_.SetKd(kd);
    break;
    }
}

bool OrientationController::IsVertical(Msg::RPY current_orientation, degrees margin) {
    return fabs(current_orientation.z - 90) < margin;
}

degrees OrientationController::GetDesiredOrientation(AXES axis) {
    return (axis == AXES::X) ? con_x_.GetDesired() : 
            ((axis == AXES::Y)? con_y_.GetDesired() : 
                                con_z_.GetDesired());
}

degrees OrientationController::GetAngleError(degrees desired, degrees current) {
    degrees err = desired - current;
    if (err > 180.0) {
        err -= 360;
    } else if (err < -180) {
        err += 360;
    }
    return err;
}

bool OrientationController::IsAngleAchieved(degrees current, degrees margin, AXES axis) {
    return (axis == AXES::X) ? fabs(GetAngleError(con_x_.GetDesired(), current)) < margin : 
           ((axis == AXES::Y)? fabs(GetAngleError(con_y_.GetDesired(), current)) < margin : 
                               fabs(GetAngleError(con_z_.GetDesired(), current)) < margin);
}