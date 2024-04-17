#ifndef MSGS_H
#define MSGS_H
#include <math.h>
#include "units.h"


namespace Msg {


    typedef struct GNSS {
        constexpr static double RADIUS_EARTH = 6371e3;

        degrees lat;
        degrees lon;
        degrees heading;

        radians RADIANS(degrees deg) {
            return deg * M_PI / 180.0;
        }

        degrees DEGREES(radians rad) {
            return rad * 180 / M_PI;
        }

        meters operator-(const GNSS& other) {
            auto delta_lat = RADIANS(lat) - RADIANS(other.lat);
            auto delta_lon = RADIANS(lon) - RADIANS(other.lon);

            auto a = pow(sin(delta_lat/2), 2.0) + cos(RADIANS(other.lat)) * cos(RADIANS(lat)) * pow(sin(delta_lon/2), 2.0);
            auto c = 2 * atan2(sqrt(a), sqrt(1 - a));

            return RADIUS_EARTH * c;
        }

        degrees operator%(const GNSS& other) {
            auto y = sin(RADIANS(lon) - RADIANS(other.lon)) * cos(RADIANS(lat));
            auto x = cos(RADIANS(other.lat)) * sin(RADIANS(lat)) - sin(RADIANS(other.lat)) * cos(RADIANS(lat)) * cos(RADIANS(lon - other.lon));
            return static_cast<int>((DEGREES(atan2(y, x))) + 360) % 360;
        }
    } GNSS;

    typedef struct Depth {
        double depth;
    } Depth;

    typedef struct RPY {
        degrees x;
        degrees y;
        degrees z;
    } RPY;


    typedef struct PWM {
        int FL, FR, DL, DR;

        PWM(): FL(1500), FR(1500), DL(1500), DR(1500){}
        PWM(int FL_, int FR_, int DL_, int DR_) {

            FL = (FL_ > 1900) ? 1900 : ((FL_ < 1100) ? 1100 : FL_);
            FR = (FR_ > 1900) ? 1900 : ((FR_ < 1100) ? 1100 : FR_);
            DL = (DL_ > 1900) ? 1900 : ((DL_ < 1100) ? 1100 : DL_);
            DR = (DR_ > 1900) ? 1900 : ((DR_ < 1100) ? 1100 : DR_);
        }

        PWM operator+(const int& of) const {
            return PWM{FL + of, FR + of, DL + of, DR + of};
        }

        PWM operator-(const int& of) const {
            return PWM{FL - of, FR - of, DL - of, DR - of};
        }
        
        PWM operator-(const PWM& rhs) const {
            return PWM{FL - rhs.FL + 1500, FR - rhs.FR + 1500, DL - rhs.DL + 1500, DR - rhs.DR + 1500};
        }//FL - 1500 - (FL - 1500)

        PWM operator+(const PWM& rhs) const {
            return PWM{FL + rhs.FL - 1500, FR + rhs.FR - 1500, DL + rhs.DL - 1500, DR + rhs.DR - 1500};

        } // (FL - 1500) + (FL - 1500) + 1500

        PWM SaturatePWM(const PWM& Forward, const PWM& Down) {
            double offset_For = Forward.FL - 1500;
            double offset_Dow = Down.DL - 1500;
            
            double ratio_For = fabs(offset_For / 400.0);
            double ratio_Dow = fabs(offset_Dow / 400.0);

            auto total_ratio = ratio_For + ratio_Dow;
            if (total_ratio > 0.75) {
                offset_For *= 0.75 / total_ratio;
                offset_Dow *= 0.75 / total_ratio;
                return {offset_For + 1500, offset_For + 1500, offset_Dow + 1500, offset_Dow + 1500};

            } else {
                return Forward + Down;
            }
        }


        
    } PWM;

    typedef struct StateMachineInput {
        enum InputType {NEW_WAYPOINT, MANUAL, START};
        InputType type;
        GNSS new_waypoint;
    } StateMachineInput;

    const static PWM pwm_FULL_OFF{1500, 1500, 1500, 1500};
    const static PWM pwm_FULL_FORWARD{1900, 1900, 1500, 1500};
    const static PWM pwm_FULL_BACKWARD{1100, 1100, 1500, 1500};

    const static GNSS gnss_INVALID{-99.0, -99.0, -99.0};

    const static RPY rpy_VERTICAL{0.0, 0.0, 90.0};
    const static RPY rpy_HORIZONTAL{0.0, 0.0, 0.0};


    }

#endif