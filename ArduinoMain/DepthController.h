#ifndef DEPTH_CON_H
#define DEPTH_CON_H
#include "Controller.h"
#include "Msgs.h"
#include "units.h"
class DepthController {
    public: //constructor
        DepthController(double kp, double ki, double kd, meters max_depth): con_(kp, ki, kd), max_depth_(max_depth){};

    public: //methods
        void SetDesiredDepth(meters depth);
        void SetGains(double Kp, double Ki, double Kd);
        Msg::PWM CalculateControlEffort(meters current_depth);
        void ResetIntegratedError();

    public: //getters
        meters GetDesiredDepth() const {return con_.GetDesired();}
    private:
        Controller con_;
        meters max_depth_;
};
#endif