#ifndef LidarPlusI2C_h
#define LidarPlusI2C_h

#include "config.h"
#include <Arduino.h>
#include <Metro.h>
#include <TFLI2C.h>
#include <Wire.h>

class LidarPlusI2C {
    public:
        LidarPlusI2C();
        void  setup(TwoWire &LIDAR_Bus, int Offset = 0);
        float read();

        int16_t tfDist;

    private:
        TFLI2C   tf;
        int16_t  tfOffset;
        uint16_t tfFPS  = FPS_100;
        uint16_t tfAddr = TFL_DEF_ADR;

        Metro i2cMetro =
            Metro(10); // check every 10ms to match the frame rate (100 FPS)
};
#endif

// #define tfFPS FPS_250 //FPS_100/125/250
// #define tfAddr TFL_DEF_ADR
