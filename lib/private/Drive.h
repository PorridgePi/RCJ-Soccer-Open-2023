#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Definitions.h>

// float CurrentOrientation = 30; // updated to the IMU readings every frame;
// Angle of the bot wrt the field float TargetOrientation = 0;
float currentOrientation = 0;
float targetOrientation  = 0;

struct Velocity { // Current Position and Orientation; updated by Localisation
    float speed = 10;
    float direction = 0;
};

Velocity velocity;

// struct {float X = 0; float Y = 0; float Orientation = 0;} targetPosori; // Target Position and Orientation

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

        float drive() {
            float directionInRadians = RAD(velocity.direction);
            float x                  = velocity.speed * sinf(directionInRadians);
            float y                  = velocity.speed * cosf(directionInRadians);
            float translationCommand = (cosf(RAD(_motorAngle)) * x - sinf(RAD(_motorAngle)) * y);
            float rotationCommand =
                sign(deltaAngle(currentOrientation, targetOrientation));
            float command = translationCommand + rotationCommand;

            // analogWrite(_pin0, constrain(command, 0, 255));
            // analogWrite(_pin1, abs(constrain(command, -255, 0)));

            // JUST FOR DEBUGGING: remember to change function to method
            return command;
        }
    
    private:
        const int   _motorAngle;
};

#endif
