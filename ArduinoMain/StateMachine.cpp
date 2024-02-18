#include "StateMachine.h"

void StateMachine::HandleInput(Msg::StateMachineInput input) {
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
            state_ = state::WAYPOINT;
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
        /* code */
        break;
    
    default:
        break;
    }
}

void StateMachine::Input(Msg::StateMachineInput input) {
    if (input.type == Msg::StateMachineInput::NEW_WAYPOINT) {
        AddWaypoint(input.new_waypoint);
    }
}