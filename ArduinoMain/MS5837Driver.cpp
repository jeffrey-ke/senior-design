#include "MS5837Driver.h"

void MS5837Driver::Init() {
    Wire.begin();
    alive_ = sensor_.init();
    sensor_.setModel(MS5837::MS5837_30BA);
    sensor_.setFluidDensity(SEAWATER_DENSITY); // kg/m^3 (freshwater, 1029 for seawater)
}

meters MS5837Driver::GetDepth() {
    if (timer_.IsExpired()) {
        sensor_.read();
        timer_.Reset();
        depth_ = sensor_.depth();
    }
    return depth_;
}