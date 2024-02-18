#include <ArduinoUnit.h>

#include "StateMachine.h"
test(standby_saves_home_coordinates) {
    StateMachine sm(1, 0, 0, 0,
                    1, 0, 0, 0,
                    5);
    assertEqual(sm.GetState(), state::STANDBY);
}