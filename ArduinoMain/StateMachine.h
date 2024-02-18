#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "WaypointController.h"
#include "ThrusterDriver.h"
#include "_GPSDriver.h"
#include "Msgs.h"
#include "Constants.h"
#include <ArduinoQueue>

namespace state {
    enum State {WAYPOINT = 'W', PROFILING = 'P', RETURN = 'R', STANDBY = 'S', MANUAL = 'M'};
}

class StateMachine {

    public: //aliases
        using Waypoints = ArduinoQueue<Msg::GNSS>;

    public: // constructor
        StateMachine();
        StateMachine(double kp_l, double ki_l, double kd_l, double kol_l, 
                        double kp_a, double ki_a, double kd_a, double kol_a,
                        unsigned num_waypoints): wp_controller_(kp_l, ki_l, kd_l, kol_l, kp_a, ki_a, kd_a, kol_a),
                                                state_(state::STANDBY), 
                                            thruster_FL(FL_PIN),
                                            thruster_FR(FR_PIN),
                                            thruster_DL(DL_PIN),
                                            thruster_DR(DR_PIN),
                                                gps_(),
                                                waypoint_itinerary_(num_waypoints)  {};

    public: //methods   
        state::State DecideState();
        state::State ExecuteState();
        void Input(Msg::StateMachineInput input) {HandleInput(input);};

    public: //Setters
        void SetState(state::State s) {state_ = s;};

    public: //Getters   
        state::State GetState() const {return state_;};


    private: //members
        WaypointController wp_controller_;
        Waypoints waypoint_itinerary_;

        ThrusterDriver thruster_FL, thruster_FR, thruster_DL, thruster_DR;
        _GPSDriver gps_;
        state::State state_;
        Msg::GNSS home_coordinates_;
        Msg::GNSS current_location_;

    private: //helper functions
        void HandleInput(const Msg::StateMachineInput& input);

        Msg::GNSS UpdateCurrentLocation()
                                {return current_location_;};

        void CommandThrusters(const Msg::PWM& cmd);

        void GotoWaypoint(const Msg::GNSS& wp);

        double IsProfileDone() 
                                {return true;}; //profile should also be done if the vehicle is in danger
        bool IsVehicleInDanger() 
                                {return false;};

        ////////////////////////////////////////////////////////////
        // waypoint_itinerary_ management //////////////////////////
        ////////////////////////////////////////////////////////////
        bool AtTheWaypoint() 
                                {return wp_controller_.IsLocationCorrectWithinMargin(current_location_);};
        bool AllWaypointsDone() 
                                {return waypoint_itinerary_.isEmpty();};
        Msg::GNSS GetCurrentWaypoint() const 
                                {return waypoint_itinerary_.getHead();};
        Msg::GNSS PopWaypoint() 
                                {if (IsWaypointItineraryNotEmpty()) return waypoint_itinerary_.dequeue();};
        void AddWaypoint(const Msg::GNSS& wp) 
                                {if (IsWaypointItineraryNotFull()) waypoint_itinerary_.enqueue(wp)};
        bool IsWaypointItineraryNotFull() const 
                                {return !waypoint_itinerary_.isFull();}
        bool IsWaypointItineraryNotEmpty() const 
                                {return !waypoint_itinerary_.isEmpty();}
        void ResetItinerary() 
                                {while (IsWaypointItineraryNotEmpty()) waypoint_itinerary_.dequeue(); }

    public: // test functions
        void test_SetCurrentLocation(const Msg::GNSS& loc) 
                                {current_location_ = loc;}
        

};


#endif