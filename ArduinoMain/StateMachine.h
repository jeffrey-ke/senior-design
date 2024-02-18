#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "WaypointController.h"
#include "ThrusterDriver.h"
#include "_GPSDriver.h"
#include "Msgs.h"
#include <ArduinoQueue>

namespace state {
    enum State {WAYPOINT = 'W', PROFILING = 'P', RETURN = 'R', STANDBY = 'S', MANUAL = 'M'};
}

class StateMachine {

    public: //aliases
        using Waypoints = ArduinoQueue<Msg::GNSS>;

    public: // constructor
        StateMachine(double kp_l, double ki_l, double kd_l, double kol_l, 
                        double kp_a, double ki_a, double kd_a, double kol_a,
                        unsigned num_waypoints): wp_controller_(kp_l, ki_l, kd_l, kol_l, kp_a, ki_a, kd_a, kol_a),
                                                state_(state::STANDBY), 
                                                gps_(),
                                                waypoint_itinerary_(num_waypoints)  {};

    public: //methods   
        state::State DecideState();
        state::State ExecuteState();
        void Input(Msg::StateMachineInput input);

    public: //Setters
        void SetState(state::State s) {state_ = s;};

    public: //Getters   
        state::State GetState() const {return state_;};


    private: //members
        WaypointController wp_controller_;

        ThrusterDriver thruster_FL, thruster_FR, thruster_DL, thruster_DR;
        _GPSDriver gps_;
        state::State state_;
        ArduinoQueue<Msg::GNSS> waypoint_itinerary_;

    private: //helper functions
        bool QueryGNSS();
        bool QueryDepth();
        bool QueryVehicleHealth();
        bool QueryWaypointsRemaining();
        const Msg::GNSS& GetCurrentWaypoint() const;
        Msg::GNSS PopWaypoint() {if (IsWaypointItineraryNotEmpty()) return waypoint_itinerary_.dequeue();};
        Msg::GNSS AddWaypoint(const Msg::GNSS& wp) {if (IsWaypointItineraryNotFull()) waypoint_itinerary_.enqueue(wp)};
        bool IsWaypointItineraryNotFull() const {return !waypoint_itinerary_.isFull();}
        bool IsWaypointItineraryNotEmpty() const {return !waypoint_itinerary_.isEmpty();}

        

};


#endif