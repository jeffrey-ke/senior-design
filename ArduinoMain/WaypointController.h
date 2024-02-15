#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include "Msgs.h"
#include "Controller.h"

class WaypointController {
    public: //aliases
        using Location = m_GNSS;
        using PWM = m_PWM;

    public: //constructor
        WaypointController(double kp, double ki, double kd, double kol=0.0): distance_controller_(kp, ki, kd, kol){};

    public: //methods
        PWM CalculatePWM(Location current_loc);
        void UpdateDesiredLocation(Location loc) {desired_ = loc;};

    private: //members

        Controller distance_controller_;
        Controller heading_controller_;

        Location desired_;
        Location current_;
};


#endif