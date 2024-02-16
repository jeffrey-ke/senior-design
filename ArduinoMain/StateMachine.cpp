#include "StateMachine.h"

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