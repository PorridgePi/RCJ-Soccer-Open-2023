#ifndef LidarPlusI2C_h
#define LidarPlusI2C_h

#include <Arduino.h>
#include <TFLI2C.h>
#include <Wire.h>
#include <elapsedMillis.h>
//#include "config.h"


class LidarPlusI2C {
    public:
        LidarPlusI2C();
        void setup(int def_addr, TwoWire &LIDAR_BUS, int set_addr = 0x00); 
        float read();
        
        int16_t tfDist;
            
    private:
        TFLI2C tf;
        int16_t tfOffset;
        uint16_t tfFPS = FPS_250;
        int _addr;

        elapsedMillis timer;
};
#endif


//#define tfFPS FPS_250 //FPS_100/125/250
//#define tfAddr TFL_DEF_ADR
