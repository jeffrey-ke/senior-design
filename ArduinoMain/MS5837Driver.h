#ifndef MS_H
#define MS_H
#include <Wire.h>
#include "MS5837.h"
#include "units.h"
#include "timer.h"

class MS5837Driver {
    #define SEAWATER_DENSITY 1029
    #define FRESHWATER_DENSITY 997

    public: //constructor
        MS5837Driver(): depth_(0), alive_(false), timer_(1000){};
    public: //methods
        void Init();
        meters GetDepth();

    private: 
        meters depth_;
        MS5837 sensor_;
        bool alive_;
        Timer timer_;


};

#endif