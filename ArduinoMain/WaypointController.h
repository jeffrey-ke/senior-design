#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include "Msgs.h"
#include "Controller.h"

class WaypointController {
    public: //aliases
        using Location = Msg::GNSS;
        using PWMCommand = Msg::PWM;
    
    public: //constants
        static double LINEAR_MARGIN_OF_ERROR;
        static double ANGULAR_MARGIN_OF_ERROR;

    public: //constructor
        WaypointController(double kp_l, double ki_l, double kd_l, double kol_l, 
                            double kp_a, double ki_a, double kd_a, double kol_a): linear_controller_(kp_l, ki_l, kd_l, kol_l),
                                                                                    angular_controller_(kp_a, ki_a, kd_a, kol_a){
            linear_controller_.SetDesired(Controller::ZERO); //we want zero linear error
            angular_controller_.SetDesired(Controller::ZERO); //we also want zero angular error 
        };

    public: //methods
        PWMCommand CalculatePWM(const Location& current_loc);
        void UpdateDesiredLocation(const Location& loc) {desired_ = loc;};
        bool IsHeadingCorrectWithinMargin(const Location& current_loc);
        bool IsLocationCorrectWithinMargin(const Location& current_loc);

    public: //Getters
        Location GetDesired() const {return desired_;};

    private: //members

        Controller linear_controller_;
        Controller angular_controller_;

        Location desired_;

    private: //helpers
        PWMCommand CalculateLinearPWM(const Location& current_loc);
        PWMCommand CalculateAngularPWM(const Location& current_loc);
        double CalculateDistanceBetween(const Location& l1, const Location& l2) {return l2 - l1;};
        double CalculateAngleDifferenceBetween(const Location& l1, const Location& l2) {return l2 % l1;};
};


#endif