#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include "Msgs.h"
#include "Controller.h"

class WaypointController {
    public: //aliases
        using Location = m_GNSS;
        using PWM = m_PWM;
    
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
        PWM CalculatePWM(Location current_loc);
        void UpdateDesiredLocation(Location loc) {desired_ = loc;};
        bool IsHeadingCorrectWithinMargin(Location current_loc);
        bool IsLocationCorrectWithinMargin(Location current_loc);

    public: //Getters
        Location GetDesired() const {return desired_;};

    private: //members

        Controller linear_controller_;
        Controller angular_controller_;

        Location desired_;

    private: //helpers
        PWM CalculateLinearPWM(Location current_loc);
        PWM CalculateAngularPWM(Location current_loc);
        double CalculateDistanceBetween(Location l1, Location l2) {return l2 - l1;};
        double CalculateAngleDifferenceBetween(Location l1, Location l2) {return l2 % l1;};
};


#endif