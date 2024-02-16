#ifndef MSGS_H
#define MSGS_H
#include <math.h>

typedef struct m_GNSS {
    constexpr static double RADIUS_EARTH = 6371e3;

    double lat;
    double lon;
    double heading;

    double operator-(const m_GNSS& other) {
        auto delta_lat = lat - other.lat;
        auto delta_lon = lon - other.lon;

        auto a = pow(sin(delta_lat/2), 2.0) + cos(other.lat) * cos(lat) * pow(sin(delta_lon/2), 2.0);
        auto c = 2 * atan2(sqrt(a), sqrt(1 - a));

        return RADIUS_EARTH * c;
    }

    double operator%(const m_GNSS& other) {
        auto y = sin(lon - other.lon) * sin(lat);
        auto x = cos(other.lat) * sin(lat) - sin(other.lat) * cos(lat) * cos(lon - other.lon);
        return atan2(y, x);
    }
} m_GNSS;

typedef struct m_depth {
    double depth;
} m_depth;

typedef struct m_PWM {
    int FL, FR, DL, DR;

    m_PWM(): FL(1500), FR(1500), DL(1500), DR(1500){}
    m_PWM(int FL_, int FR_, int DL_, int DR_) {
        FL = (FL_ > 1900) ? 1900 : ((FL_ < 1100) ? 1100 : FL_);
        FR = (FR_ > 1900) ? 1900 : ((FR_ < 1100) ? 1100 : FR_);
        DL = (DL_ > 1900) ? 1900 : ((DL_ < 1100) ? 1100 : DL_);
        DR = (DR_ > 1900) ? 1900 : ((DR_ < 1100) ? 1100 : DR_);
    }

    m_PWM operator+(const int& of) const {
        return m_PWM{FL + of, FR + of, DL + of, DR + of};
    }

    m_PWM operator-(const int& of) const {
        return m_PWM{FL - of, FR - of, DL - of, DR - of};
    }
    
    m_PWM operator-(const m_PWM& rhs) const {
        return m_PWM{FL - rhs.FL + 1500, FR - rhs.FR + 1500, DL - rhs.DL + 1500, DR - rhs.DR + 1500};
    }//FL - 1500 - (FL - 1500)

    m_PWM operator+(const m_PWM& rhs) const {
        return m_PWM{FL + rhs.FL - 1500, FR + rhs.FR - 1500, DL + rhs.DL - 1500, DR + rhs.DR - 1500};
    } // FL - 1500 + FL - 1500

    
} m_PWM;


const static m_PWM pwm_FULL_OFF{1500, 1500, 1500, 1500};
const static m_PWM pwm_FULL_FORWARD{1900, 1900, 1500, 1500};
const static m_PWM pwm_FULL_BACKWARD{1100, 1100, 1500, 1500};

#endif