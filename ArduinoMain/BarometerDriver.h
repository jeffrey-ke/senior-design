#ifndef BARO_H
#define BARO_H
#include <Adafruit_MPL3115A2.h>
#include <ArduinoQueue.h>


class BarometerDriver
{

    public: //aliases
        using hPa = double;
        using mmHg = double;
    public: // constructor
        BarometerDriver(unsigned buf_size): buf_(buf_size), is_alive_(false) {};

    public: //methods
        inline mmHg GetAvgPressure();
        inline void Init();


    private: //helpers
        inline void AddMeasurement();

    private: //private members
        Adafruit_MPL3115A2 baro_;
        ArduinoQueue<mmHg> buf_;
        bool is_alive_;

    /* data */
};





#endif