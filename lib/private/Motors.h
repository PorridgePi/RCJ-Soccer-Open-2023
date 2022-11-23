#ifndef MOTORS_h
#define MOTORS_h

#include <arduino.h>

class Motor {
    private:
        int _pin1;
        int _pin2;

    public:
        Motor(int pin1, int pin2);
        void move(double spd);
};

#endif
