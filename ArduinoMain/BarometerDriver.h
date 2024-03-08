#ifndef BARO_H
#define BARO_H
#include <Adafruit_MPL3115A2.h>
#include <ArduinoQueue.h>
#include "units.h"


class BarometerDriver
{
    public: // constructor
        BarometerDriver(unsigned buf_size): buf_(buf_size), is_alive_(false) {};

    public: //methods
        mmHg GetAvgPressure();
        void Init();


    private: //helpers
        void AddMeasurement();

    private: //private members
        Adafruit_MPL3115A2 baro_;
        ArduinoQueue<mmHg> buf_;
        bool is_alive_;

    /* data */
};





#endif