#ifndef SERIALPARSER_H
#define SERIALPARSER_H

#include "units.h"
#include "string.h"

enum TestType {PRESSURE, DIVE, FLIP, NONE};
typedef struct TestParams {
    TestType type;
    double Kp;
    double Ki;
    double Kd;
    milliseconds duration;
    meters target_depth;
    mmHg max_deviation;
    TestParams(): type(NONE){};
    TestParams(TestType type_, double Kp_, double Ki_, double Kd_, milliseconds duration_, meters target_depth_, mmHg max_dev_):
                type(type_),
                Kp(Kp_),
                Ki(Ki_),
                Kd(Kd_),
                duration(duration_),
                target_depth(target_depth_),
                max_deviation(max_dev_) {}

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