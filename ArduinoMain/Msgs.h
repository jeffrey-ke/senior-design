#ifndef MSGS_H
#define MSGS_H
#include <math.h>

namespace Msg {


    typedef struct GNSS {
        constexpr static double RADIUS_EARTH = 6371e3;

        double lat;
        double lon;
        double heading;

        double operator-(const GNSS& other) {
            auto delta_lat = lat - other.lat;
            auto delta_lon = lon - other.lon;

            auto a = pow(sin(delta_lat/2), 2.0) + cos(other.lat) * cos(lat) * pow(sin(delta_lon/2), 2.0);
            auto c = 2 * atan2(sqrt(a), sqrt(1 - a));

            return RADIUS_EARTH * c;
        }

        double operator%(const GNSS& other) {
            auto y = sin(lon - other.lon) * sin(lat);
            auto x = cos(other.lat) * sin(lat) - sin(other.lat) * cos(lat) * cos(lon - other.lon);
            return atan2(y, x);
        }
    } GNSS;

    typedef struct Depth {
        double depth;
    } Depth;

    typedef struct RPY {
        double x;
        double y;
        double z;
    } RPY;

    typedef struct PWM {
        int FL, FR, DL, DR;

        PWM(): FL(1500), FR(1500), DL(1500), DR(1500){}
        PWM(int FL_, int FR_, int DL_, int DR_) {
            FL = (FL_ > 1800) ? 1800 : ((FL_ < 1200) ? 1200 : FL_);
            FR = (FR_ > 1800) ? 1800 : ((FR_ < 1200) ? 1200 : FR_);
            DL = (DL_ > 1800) ? 1800 : ((DL_ < 1200) ? 1200 : DL_);
            DR = (DR_ > 1800) ? 1800 : ((DR_ < 1200) ? 1200 : DR_);
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