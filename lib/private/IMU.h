#ifndef IMU_h
#define IMU_h

#include "config.h"
#include <arduino.h>

class IMU {
    private:
        int _pin;
        byte  sign;
        byte  value;
        float rot = 0;

    public:
        IMU(int pin);
        void  setup();
        float read();
        float offset = 314;
};

#endif
