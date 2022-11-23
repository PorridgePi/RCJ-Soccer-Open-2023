#ifndef LOCALISATION_h
#define LOCALISATION_h

#include <arduino.h>

#include "Lidar.h"
#include "config.h"
#include <elapsedMillis.h>

class Localisation {
    public:
        Localisation(int LEFT_ADDR, int RIGHT_ADDR, int FRONT_ADDR, int BACK_ADDR);
        void setup();

        bool update(float rot);
        void calcCoord(float rot);

        bool boundaryCheck(); //, goalCheck();
        bool moveTo(float x, float y, bool checkConfidence = true, int rad = 4);
        void aim();

        void debug();
        void confidence();
        void boundaryScale(float spd, float dir);

        float dir, spd, dist;
        float coord_x, coord_y;

    private:
        float front, back, left, right;

        float x_confidence, y_confidence;
        int   xr_offset = 5, xl_offset = 5, yf_offset = 8, yb_offset = 5;

        float x_constrain_min = -1, x_constrain_max = 1;
        float y_constrain_min = -1, y_constrain_max = 1;

        float spd_x, spd_y;

        bool stopBot = false;

        LidarPlusI2C ld_left;
        LidarPlusI2C ld_right;
        LidarPlusI2C ld_back;
        LidarPlusI2C ld_front;

        int _LEFT_ADDR, _RIGHT_ADDR, _BACK_ADDR, _FRONT_ADDR;
};
#endif
