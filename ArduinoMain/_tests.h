#include <ArduinoUnit.h>
#include "MasterComputeBridge.h"
#include "Constants.h"
#include "WaypointController.h"

test(basic_controller_test){
    Controller c(1, 0, 0);
    assertEqual(c.GetKp(), 1);

    c.SetDesired(10);
    assertEqual(c.GetDesired(), 10);

    auto eff = c.CalculateControlEffort(0);
    assertEqual(eff, 10 * c.GetKp());

    eff = c.CalculateControlEffort(5);
    assertEqual(eff, 5 * c.GetKp());

    eff = c.CalculateControlEffort(10);
    assertEqual(eff, 0);

    c.SetDesired(0);
    assertEqual(c.GetDesired(), 0);

    eff = c.CalculateControlEffort(10);
    assertEqual(eff, -10 * c.GetKp());
}

test(wp_controller) {
    WaypointController wp_c(1, 0, 0, 0
                            1, 0, 0, 0);
    wp_c.UpdateDesiredLocation()
}


void setup() {
// put your setup code here, to run once:
    Serial.begin(115200);
}

void loop() {
// put your main code here, to run repeatedly:
    Test::run();
}