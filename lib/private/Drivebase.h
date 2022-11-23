#ifndef DRIVEBASE_h
#define DRIVEBASE_h

#include "Motors.h"
#include "config.h"
#include <arduino.h>

class Drivebase {
    private:
        Motor &ma;
        Motor &mb;
        Motor &mc;
        Motor &md;
        float  path;

    public:
        Drivebase(Motor &A, Motor &B, Motor &C, Motor &D);
        void drive(double spd, float dir, float rot);
        void activeAIM(); // assistiveAIM()
};

#endif
