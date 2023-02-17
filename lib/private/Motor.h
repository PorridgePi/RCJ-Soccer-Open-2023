#ifndef KICKER_H
#define KICKER_H

#include "Arduino.h"

const float pi = 3.14159265358979323846f; // probably will reduce the precision
const float rad = pi / 180;
// float CurrentOrientation = 30; // updated to the IMU readings every frame;
// Angle of the bot wrt the field float TargetOrientation = 0;
float currentOrientation = 0;
float targetOrientation  = 0;

struct {
    float magnitude = 10;
    float direction = 0;
} Velocity; // Current Position and Orientation; updated by Localisation

// struct {float X = 0; float Y = 0; float Orientation = 0;} targetPosori; // Target Position and Orientation

signed char sign(float A) {
    return (A < 0) ? -1 : ((A > 0) ? 1 : 0);
};

float deltaAngle(float current,
                 float target) { // Gets the smallest difference in 2 angle,
                                 // including its direction
    if (abs(current - target) < 180) {
        return target - current;
    } else {
        return target - current - 360;
    }
};

class Motor {
    private:
    const int   _angle;
    const float _multiplier;
    const int   _pin0;
    const int   _pin1;

    public:
    float drive() { // float velocityX, float velocityY, float
                    // currentOrientation, float targetOrientation) {
        float directionInRadians = (Velocity.direction) * rad;
        float x = Velocity.magnitude * sinf(directionInRadians);
        float y = Velocity.magnitude * cosf(directionInRadians);
        float motorOrientationInRadians = _angle * rad;
        float translationCommand        = (cosf(motorOrientationInRadians) * x - sinf(motorOrientationInRadians) * y);
        float rotationCommand =
            sign(deltaAngle(currentOrientation, targetOrientation));
        float command = _multiplier * (translationCommand + rotationCommand);
        digitalWrite(_pin0, constrain(command, 0, 255));
        digitalWrite(_pin1, abs(constrain(command, -255, 0)));

        // JUST FOR DEBUGGING: remember to change function to method
        return command;
    };
    Motor(float ANGL, float MULT, int PIN0, int PIN1) :
        _angle(ANGL), _multiplier(MULT), _pin0(PIN0), _pin1(PIN1) {
        pinMode(_pin0, OUTPUT);
        pinMode(_pin1, OUTPUT);
    } // SETS _angle = ANGL, _multiplier = MULT, etc.
};

#endif
