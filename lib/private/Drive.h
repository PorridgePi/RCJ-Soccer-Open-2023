#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Definitions.h>

class Drive {
    public:
        Drive(int motorAngle) : _motorAngle(motorAngle) {}

        int drive() {
            constrain(_speed, -1, 1);
            float x = sinf(RAD(_direction));
            float y = cosf(RAD(_direction));
            float translationCommand = _speed * 255 * (cosf(RAD(_motorAngle)) * x - sinf(RAD(_motorAngle)) * y);

            return translationCommand;
        }

        void setSpeed(float speed) {
            _speed = speed;
        }

        void setDirection(int direction) {
            _direction = direction;
        }

    private:
        const int _motorAngle;
        float _speed;
        int _direction;
};

#endif
