#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    public:
        Motor(int pin0, int pin1, float maxSpeed) :
            _pin0(pin0), _pin1(pin1), _maxSpeed(maxSpeed) {
            pinMode(_pin0, OUTPUT);
            pinMode(_pin1, OUTPUT);
        }

        void setSpeed(float speed) {
            _speed = constrain(speed, -_maxSpeed, _maxSpeed);
            if (_speed >= 0) {
                analogWrite(_pin0, _speed * 255);
                analogWrite(_pin1, 0);
            } else {
                analogWrite(_pin0, 0);
                analogWrite(_pin1, -_speed * 255);
            }
        }

    private:
        const int _pin0;
        const int _pin1;
        float     _speed;
        float     _maxSpeed;
};

#endif
