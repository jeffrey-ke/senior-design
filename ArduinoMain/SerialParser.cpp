#include "SerialParser.h"
#include <Arduino.h>
    /*
        Dive: "DIVE,P,I,D,DURATION,TARGET_DEPTH"
        Flip: "FLIP,P,I,D,DURATION"
        Pressure: "PRESSURE,DURATION,DEVIATION"
        
    */
TestParams SerialParser::ParseLine(class String &line) {
    char* line_cstr = line.c_str();
    char *type = strtok(line_cstr, ",");
    Serial.println(type);
    if (CStringsEqual(type, "DIVE")) {
        auto Kp = strtod(strtok(NULL, ","), NULL);
        auto Ki = strtod(strtok(NULL, ","), NULL);
        auto Kd = strtod(strtok(NULL, ","), NULL);
        auto duration = strtod(strtok(NULL, ","), NULL);
        auto target_depth = strtod(strtok(NULL, ","), NULL);
        return TestParams(DIVE, Kp, Ki, Kd, duration, target_depth, 0.0);

    } else if (CStringsEqual(type, "FLIP")) {
        auto Kp = strtod(strtok(NULL, ","), NULL);
        auto Ki = strtod(strtok(NULL, ","), NULL);
        auto Kd = strtod(strtok(NULL, ","), NULL);
        auto duration = strtod(strtok(NULL, ","), NULL);
        return TestParams(FLIP, Kp, Ki, Kd, duration, 0.0, 0.0);

    } else if (CStringsEqual(type, "PRESSURE")) {
        auto duration = strtod(strtok(NULL, ","), NULL);
        auto deviation = strtod(strtok(NULL, ","), NULL);
        return TestParams(PRESSURE, 0, 0, 0, duration, 0.0, deviation);

    }
    return TestParams(NONE, 0, 0, 0, 0, 0, 0);
}

bool SerialParser::CStringsEqual(const char* str1, const char* str2) const {
    return strcmp(str1, str2) == 0;
}