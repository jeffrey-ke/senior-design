#ifndef SERIALPARSER_H
#define SERIALPARSER_H

#include "units.h"
#include "Msgs.h"
#include "string.h"

enum TestType {PRESSURE, DIVE, FLIP, FUN, WAYPOINT, NONE};

typedef struct TestParams {
    TestType type;
    double Kp, Kp2, Kp3;
    double Ki, Ki2, Ki3;
    double Kd, Kd2, Kd3;
    milliseconds duration;
    milliseconds duration2;
    meters distance_threshold;
    degrees heading_threshold;
    Msg::GNSS goal_point;
    meters target_depth;
    mmHg max_deviation;
    pwm forward_pwm;
    TestParams(): type(NONE){};
    TestParams(TestType type_, double Kp_, double Ki_, double Kd_, milliseconds duration_, meters target_depth_, mmHg max_dev_, milliseconds duration2_, pwm for_pwm):
                type(type_),
                Kp(Kp_),
                Ki(Ki_),
                Kd(Kd_),
                duration(duration_),
                target_depth(target_depth_),
                max_deviation(max_dev_),
                duration2(duration2_),
                forward_pwm(for_pwm) {}

    void Reset() {type = NONE;}
} TestParams;

class SerialParser {
    public: //constructor
        SerialParser(){};
    public: //methods
        TestParams ParseLine(class String &line);

    private: //helpers
        bool CStringsEqual(const char* str1, const char* str2) const;
};
#endif