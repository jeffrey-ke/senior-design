#ifndef TIMER_H
#define TIMER_H
#include "units.h"
typedef struct Timer {
    Timer(milliseconds durtn): duration_(durtn), start_time_(millis()){}
    void Reset() {start_time_ = millis();}
    bool IsExpired() const {return millis() - start_time_ > duration_;}
    milliseconds start_time_;
    milliseconds duration_;
} Timer;

#endif