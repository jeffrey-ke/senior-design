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
        con_(k_p, k_i, k_d, k_ol)
        {
        };

    public: //methods
        void SetDesiredToVertical();
        void SetDesiredToHorizontal();
        void SetDesiredOrientation(Msg::RPY des);
        Msg::PWM CalculateControlEffort(Msg::RPY current_orientation); 
        void SetGains(double kp, double ki, double kd);
        bool IsVertical(Msg::RPY current_orientation, double margin=5);
 
    public: //getters
        double GetDesiredOrientation() const {return con_.GetDesired();}

    private: //members
        Controller con_;
};


#endif