#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    public:
        Motor(int pin0, int pin1) :
            _pin0(pin0), _pin1(pin1) {
            pinMode(_pin0, OUTPUT);
            pinMode(_pin1, OUTPUT);
        }

        void setSpeed(float speed) {
            _speed = constrain(speed, -1, 1);
            if (_speed >= 0) {
                analogWrite(_pin0, 255);
                analogWrite(_pin1, 255 - abs(_speed) * 255);
            } else {
                analogWrite(_pin0, 255 - (abs(_speed) * 255));
                analogWrite(_pin1, 255);
            }
        }

    private:
        const int _pin0;
        const int _pin1;
        float     _speed;
};

#endif
