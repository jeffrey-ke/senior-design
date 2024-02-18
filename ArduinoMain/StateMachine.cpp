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