#include "SerialParser.h"
#include <Arduino.h>
    /*
        Dive: "DIVE,P,I,D,DURATION,TARGET_DEPTH"
        Flip: "FLIP,P,I,D,DURATION"
        Pressure: "PRESSURE,DURATION,DEVIATION"
        Waypoint: "WAYPOINT,P,I,D,DURATION,DISTANCE_THRESHOLD,HEADING_THRESHOLD,GOAL_LAT,GOAL_LON"
    */
TestParams SerialParser::ParseLine(class String &line) {
    char* line_cstr = line.c_str();
    char *type = strtok(line_cstr, ",");
    if (CStringsEqual(type, "DIVE")) {
        auto Kp = strtod(strtok(NULL, ","), NULL);
        auto Ki = strtod(strtok(NULL, ","), NULL);
        auto Kd = strtod(strtok(NULL, ","), NULL);
        auto duration = strtod(strtok(NULL, ","), NULL);
        auto target_depth = strtod(strtok(NULL, ","), NULL);
        return TestParams(DIVE, Kp, Ki, Kd, duration, target_depth, 0.0, 0, 0);

    } else if (CStringsEqual(type, "FLIP")) {
        auto Kp = strtod(strtok(NULL, ","), NULL);
        auto Ki = strtod(strtok(NULL, ","), NULL);
        auto Kd = strtod(strtok(NULL, ","), NULL);
        auto duration = strtod(strtok(NULL, ","), NULL);
        return TestParams(FLIP, Kp, Ki, Kd, duration, 0.0, 0.0, 0, 0);

    } else if (CStringsEqual(type, "PRESSURE")) {
        auto duration = strtod(strtok(NULL, ","), NULL);
        auto deviation = strtod(strtok(NULL, ","), NULL);
        return TestParams(PRESSURE, 0, 0, 0, duration, 0.0, deviation, 0, 0);

    } else if (CStringsEqual(type, "FUN")) {
        auto Kp = strtod(strtok(NULL, ","), NULL);
        auto Ki = strtod(strtok(NULL, ","), NULL);
        auto Kd = strtod(strtok(NULL, ","), NULL);
        auto duration_vertical = strtod(strtok(NULL, ","), NULL);
        auto duration_horizontal = strtod(strtok(NULL, ","), NULL);
        auto pwm_forward = static_cast<int>(strtol(strtok(NULL, ","), NULL, 10));
        return TestParams(FUN, Kp, Ki, Kd, duration_vertical, 0, 0, duration_horizontal, pwm_forward);
    }
    else if (CStringsEqual(type, "WAYPOINT")) {
        auto Kp = strtod(strtok(NULL, ","), NULL);
        auto Ki = strtod(strtok(NULL, ","), NULL);
        auto Kd = strtod(strtok(NULL, ","), NULL);
        auto duration = strtod(strtok(NULL, ","), NULL);
        auto distance_threshold = strtod(strtok(NULL, ","), NULL);
        auto heading_threshold = strtod(strtok(NULL, ","), NULL);
        auto goal_lat = strtod(strtok(NULL, ","), NULL);
        auto goal_lon = strtod(strtok(NULL, ","), NULL);
        TestParams p;
        p.Kp = Kp;
        p.Ki = Ki;
        p.Kd = Kd;
        p.duration = duration;
        p.distance_threshold = distance_threshold;
        p.heading_threshold = heading_threshold;
        p.goal_point = Msg::GNSS{goal_lat, goal_lon, 0};
        return p; 
    }
    return TestParams(NONE, 0, 0, 0, 0, 0, 0, 0, 0);
}

bool SerialParser::CStringsEqual(const char* str1, const char* str2) const {
    return strcmp(str1, str2) == 0;
}