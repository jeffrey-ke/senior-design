#include "StateMachine.h"

void StateMachine::HandleInput(const Msg::StateMachineInput& input) {
    switch (state_)
    {
    /////////////////////////////////////////////////////////////////////////
    // STANDBY /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::STANDBY:
        if (input.type == Msg::StateMachineInput::MANUAL) {
            state_ = state::MANUAL;
        }
        else if (input.type == Msg::StateMachineInput::START) {
            state_ = (IsGPSReady())? state::WAYPOINT : state::STANDBY;
        }
        else if (input.type == Msg::StateMachineInput::NEW_WAYPOINT) {
            AddWaypoint(input.new_waypoint);
        }
    break;
    /////////////////////////////////////////////////////////////////////////
    // WAYPOINT /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::WAYPOINT:
        if (input.type == Msg::StateMachineInput::NEW_WAYPOINT) {
            AddWaypoint(input.new_waypoint);
        }
    break;

    default:
        break;
    }
}


state::State StateMachine::DecideState() {
    switch (state_)
    {
    /////////////////////////////////////////////////////////////////////////
    // STANDBY /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::STANDBY:
    break;

    /////////////////////////////////////////////////////////////////////////
    // WAYPOINT /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::WAYPOINT:
        if (AllWaypointsDone()) {
            state_ = state::RETURN;
        }
        else if (AtTheWaypoint()) {
            state_ = state::PROFILING;
        }
        else if (IsVehicleInDanger()) {
            state_ = state::RETURN;
        }
    break;
    /////////////////////////////////////////////////////////////////////////
    // PROFILING ////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::PROFILING:
        if (IsProfileDone()) {
            PopWaypoint();
            state_ = state::WAYPOINT;
        }
    break;
    /////////////////////////////////////////////////////////////////////////
    // RETURN ///////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::RETURN:
        if (AtTheWaypoint()) {
            state_ = state::STANDBY;
        }
    break;
    default:
    break;
    }
}

state::State StateMachine::ExecuteState() {
    switch (state_)
    {
    case state::STANDBY:
        // (1) save home coordinates
        // (2) display command menu
        //      (2.1) print out collected data
        //      (2.2) print out health of all systems
        home_coordinates_ = GetCurrentLocation();
    break;
    /////////////////////////////////////////////////////////////////////////
    // WAYPOINT /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::WAYPOINT:
        auto current_waypoint = GetCurrentWaypoint();
        current_location_ = GetCurrentLocation();
        GotoWaypoint(current_waypoint);
    break;
    /////////////////////////////////////////////////////////////////////////
    // PROFILING ////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::PROFILING:
        //gotodepth()
    break;
    /////////////////////////////////////////////////////////////////////////
    // RETURN ///////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    case state::RETURN:
        current_location_ = GetCurrentLocation();
        GotoWaypoint(home_coordinates_);
    break;
    default:
        break;
    }
}

void StateMachine::GotoWaypoint(const Msg::GNSS& wp) {
    wp_controller_.UpdateDesiredLocation(wp);
    auto pwm_command = wp_controller_.CalculatePWM(current_location_);
    CommandThrusters(pwm_command);
}

void StateMachine::CommandThrusters(const Msg::PWM& pwm_command) {
    thruster_FL.setVelocity(pwm_command.FL);
    thruster_FR.setVelocity(pwm_command.FR);
    thruster_DL.setVelocity(pwm_command.DL);
    thruster_DR.setVelocity(pwm_command.DR);
}