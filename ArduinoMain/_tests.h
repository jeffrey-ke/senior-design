
#include <ArduinoUnit.h>
#include "WaypointController.h"
#include "Msgs.h"
#include "_helper_tests.h"

test(bearing_calculation) {
    Msg::GNSS l1{0.6518668445984239, -2.128217453995158, 0.0}, l2{0.6515253011171012, -2.1293523019813896, 0.0};
    assertNear(l1 % l2, 1.20858232936434, 0.20);
}

test(distance_calculation) {
    Msg::GNSS l1{0.6518668445984239, -2.128217453995158, 0.0}, l2{0.6515253011171012, -2.1293523019813896, 0.0};
    assertNear(l1 - l2, 6.146e3, 1.0);
}

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
    WaypointController wp_c(1, 0, 0, 0,
                            1, 0, 0, 0);
    Msg::GNSS desired_loc{10, 10, 0};
    wp_c.UpdateDesiredLocation(desired_loc);
    assertLocationsEqual(desired_loc, wp_c.GetDesired());

    auto eff = wp_c.CalculatePWM(Msg::GNSS{10, 10, 0});
    assertPWMEqual(eff, Msg::pwm_FULL_OFF);

    eff = wp_c.CalculatePWM(Msg::GNSS{10, 10, 0.1});
    assertPWMEqual(eff, Msg::pwm_FULL_OFF);

    eff = wp_c.CalculatePWM(Msg::GNSS{10, 10, 69.0}); //regardless of heading, pwm should be zero.
    assertPWMEqual(eff, Msg::pwm_FULL_OFF);
}


