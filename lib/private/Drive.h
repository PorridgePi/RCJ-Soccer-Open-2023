#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Definitions.h>

signed char sign(float A) {
    return (A < 0) ? -1 : ((A > 0) ? 1 : 0);
};

float deltaAngle(float current, float target) { // Gets the smallest difference in 2 angle, including its direction
    if (abs(current - target) < 180) {
        return target - current;
    } else {
        return target - current - 360;
    }
};

class Drive {
    public:
        Drive(int ANGL) :
            _motorAngle(ANGL) {
        }

        int drive() {
            constrain(_speed, -1, 1);
            float x = sinf(RAD(_direction));
            float y = cosf(RAD(_direction));
            float translationCommand = _speed * 255 * (cosf(RAD(_motorAngle)) * x - sinf(RAD(_motorAngle)) * y);

            // analogWrite(_pin0, constrain(command, 0, 255));
            // analogWrite(_pin1, abs(constrain(command, -255, 0)));

            // JUST FOR DEBUGGING: remember to change function to method
            return translationCommand;
        }

        void setSpeed(float speed) {
            _speed = speed;
        }

        void setDirection(int direction) {
            _direction = direction;
        }

    private:
        const int   _motorAngle;
        float _speed;
        int _direction;
};

#endif
