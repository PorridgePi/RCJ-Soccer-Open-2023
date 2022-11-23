#ifndef LidarPlus_h
#define LidarPlus_h

#include <Arduino.h>
#include <TFMPlus.h>
#include "config.h"
#include <Metro.h>

class LidarPlus {
    public:
        LidarPlus();
        void setup(HardwareSerial &LIDAR_SERIAL); float read();
        int16_t tfDist;
            
    private:
        TFMPlus tf;
        Metro serialMetro = Metro(10); //check every 10ms to match the frame rate (100 FPS)

};
#endif
