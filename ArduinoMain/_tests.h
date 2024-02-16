#include <ArduinoUnit.h>
#include "MasterComputeBridge.h"
#include "Constants.h"
#include "WaypointController.h"
#include "Msgs.h"

void assertLocationsEqual(const m_GNSS& l1, const m_GNSS& l2) {
    assertEqual(l1.lat, l2.lat);
    assertEqual(l1.lon, l2.lon);
    assertEqual(l1.heading, l2.heading);
}

void assertPWMEqual(const m_PWM& p1, const m_PWM& p2) {
    assertEqual(p1.FL, p2.FL);
    assertEqual(p1.FR, p2.FR);
    assertEqual(p1.DL, p2.DL);
    assertEqual(p1.DR, p2.DR);
}


test(bearing_calculation) {
    m_GNSS l1{37.349219, -121.937878, 0.0}, l2{37.329650, -122.002900, 0.0};
    assertEqual(l1 % l2, 1.20858232936434);
}

test(distance_calculation) {
    m_GNSS l1{37.349219, -121.937878, 0.0}, l2{37.329650, -122.002900, 0.0};
    assertEqual(l1 - l2, 6.146e3);
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
    m_GNSS desired_loc{10, 10, 0};
    wp_c.UpdateDesiredLocation(desired_loc);
    assertLocationsEqual(desired_loc, wp_c.GetDesired());

    auto eff = wp_c.CalculatePWM(m_GNSS{10, 10, 0});
    assertPWMEqual(eff, pwm_FULL_OFF);

    eff = wp_c.CalculatePWM(m_GNSS{10, 10, 0.1});
    assertPWMEqual(eff, m_PWM{});
}


void setup() {
// put your setup code here, to run once:
    Serial.begin(115200);
}

void loop() {
// put your main code here, to run repeatedly:
    Test::run();
}