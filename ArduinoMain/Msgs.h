#ifndef MSGS_H
#define MSGS_H
#include <math.h>

typedef struct m_GNSS {
    static const double RADIUS_EARTH = 6378.1370e3;

    double lat;
    double lon;
    double heading;

    double operator-(const m_GNSS& other) {
        auto delta_lat = other.lat - lat;
        auto delta_lon = other.lon - lon;

        auto a = pow(sin(delta_lat/2), 2) + cos(lat) * cos(other.lat) * pow(sin(delta_lon/2), 2);
        auto c = atan2(sqrt(a), sqrt(1 - a));

        return RADIUS_EARTH * c;
    }

    double operator%(const m_GNSS& other) {
        auto y = sin(other.lon - lon) * sin(other.lat);
        auto x = cos(lat) * sin(other.lat) - sin(lat) * cos(other.lat) * cos(other.lon - lon);
        return atan2(y, x);
    }
} m_GNSS;

typedef struct m_depth {
    double depth;
} m_depth;

typedef struct m_PWM {
    int FL, FR, DL, DR;

    m_PWM operator+(const m_PWM& rhs) {
        return m_PWM{FL + rhs.FL, FR + rhs.FR, DL + rhs.DL, DR + rhs.DR};
    }

    m_PWM operator-(const m_PWM& rhs) {
        return m_PWM{FL - rhs.FL, FR - rhs.FR, DL - rhs.DL, DR - rhs.DR};
    }
} m_PWM;

#endif