#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include "Msgs.h"

class WaypointController {

    public: //aliases
        using Location = m_GNSS;

    public: // Constructors
        WaypointController(double kp, double ki, double kd, double kol=0.0);
    
    public: // Setters
        void SetKp(double kp) {k_p_ = kp;};
        void SetKi(double ki) {k_i_ = ki;};
        void SetKd(double kd) {k_d_ = kd;};
        void SetKol(double kol) {k_ol_ = kol;};
        void SetTargetWaypoint(Location target) {target_wp_ = target;};
        void SetCurrentLocation(Location current) {current_loc_ = current;};
        void ResetIntegratedError() {integrated_error_ = 0.0;};

    public: //Getters
        double GetKp() {return k_p_;};
        double GetKi() {return k_i_;};
        double GetKd() {return k_d_;};
        double GetKol() {return k_ol_;};
        Location GetTargetWaypoint() {return target_wp_;};
        Location GetCurrentLocation() {return current_loc_;};

    private: // member vars
        double k_p_, k_i_, k_d_, k_ol_;
        Location target_wp_;
        Location current_loc_;

        double integrated_error_;
        double previous_error_;


    private: //helper functions
        double DistanceBetweenLocations(Location l1, Location l2);
        double BearingBetweenLocations(Location l1, Location l2);

};



#endif