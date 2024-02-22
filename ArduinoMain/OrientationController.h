#ifndef O_CONTROLLER_H
#define O_CONTROLLER_H
#include "Controller.h"
#include "Msgs.h"
#include "IMUDriver.h"

// ONLY controls ___ (insert whatever axis we're controlling)
class OrientationController {
    public: // constructor
        OrientationController(double k_p, double k_i, double k_d, double k_ol=0.0)
        :
        con_(k_p, k_i, k_d, k_ol),
        imu_()
        {};

    public: //methods
        void SetDesiredToVertical() {con_.SetDesired(Msg::rpy_VERTICAL.z);}
        void SetDesiredToHorizontal() {con_.SetDesired(Msg::rpy_HORIZONTAL.z);}
        void SetDesiredOrientation(Msg::RPY des) {con_.SetDesired(des.z);};
        Msg::PWM CalculateControlEffort(Msg::RPY current_orientation) 
                                                                    {auto effort = static_cast<int>(con_.CalculateControlEffort(current_orientation.z) / 2); return Msg::PWM{1500, 1500, 1500 + effort, 1500 + effort};}
        Msg::PWM CalculateControlEffort() 
                                                                    {auto effort = con_.CalculateControlEffort(imu_.GetData().z); Serial.println(effort); return Msg::PWM{1500, 1500, 1500 + effort, 1500 + effort};} //assuming that positive control effort is 1500+ pwm

    public: //getters
        double GetDesiredOrientation() const {return con_.GetDesired();}
        Msg::RPY GetCurrentOrientation() {return imu_.GetData();}

    private: //members
        Controller con_;
        IMUDriver imu_;
};


#endif