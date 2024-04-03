#ifndef HELPER
#define HELPER
#include "Msgs.h"
#include <ArduinoUnit.h>
void assertLocationsEqual(const Msg::GNSS& l1, const Msg::GNSS& l2) {
    assertEqual(l1.lat, l2.lat);
    assertEqual(l1.lon, l2.lon);
    assertEqual(l1.heading, l2.heading);
}

void assertPWMEqual(const Msg::PWM& p1, const Msg::PWM& p2) {
    assertEqual(p1.FL, p2.FL);
    assertEqual(p1.FR, p2.FR);
    assertEqual(p1.DL, p2.DL);
    assertEqual(p1.DR, p2.DR);
}

#endif
