#include "BarometerDriver.h"

BarometerDriver::Init() {
    is_alive_ = baro_.begin();
    baro_.setSeaPressure(1013.26);
}

BarometerDriver::mmHg BarometerDriver::GetAvgPressure() {
    AddMeasurement();
    mmHg sum{0.0};
    for (int i = 0; i < buf_.itemCount(); ++i) {
        auto measurement = buf_.dequeue();
        sum += measurement;
        buf_.enqueue(measurement);
    }
    return sum / buf_.itemCount();
}

void BarometerDriver::AddMeasurement() {
    if (buf_.isFull()) 
        buf_.dequeue();
    hPa pressure = baro_.getPressure();
    mmHg pressure_in_mmHg = pressure * 0.750062;
    buf_.enqueue(pressure_in_mmHg);
}
