#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Definitions.h>
#include <Motor.h>

class Drive {
    public:
        Drive(Motor &motorFR, Motor &motorBR, Motor &motorBL, Motor &motorFL) : _motorFR(motorFR), _motorBR(motorBR), _motorBL(motorBL), _motorFL(motorFL) {}

        void setDrive(float speed, int angle, float rotationRate) {
            rotationRate = constrain(rotationRate, -1, 1);
            speed = constrain(speed, -1, 1) * (1-abs(rotationRate));
            // when rotationRate == 0, bot moves straight
            // when rotationRate == 1, bot rotates clockwise on the spot
            
            float speedX, speedY;
            speedX = speed * cosf(RAD(angle + 45));
            speedY = speed * sinf(RAD(angle + 45));

            float speedFL, speedFR, speedBL, speedBR;
            speedFR = speedX;
            speedBR = speedY;
            speedFL = -speedY;
            speedBL = -speedX;


            //Check if any of the wheel's speed exceeds ± 1,

            //if yes, find the wheel's speed that exceeded the most,
            float exceeded = 0;
            exceeded = max(abs(speedBL+rotationRate)-1, exceeded);
            exceeded = max(abs(speedFL+rotationRate)-1, exceeded);
            exceeded = max(abs(speedBR+rotationRate)-1, exceeded);
            exceeded = max(abs(speedFR+rotationRate)-1, exceeded);

            


            //divide all the movement components by the amount needed such that it does not exceed ±1

            /*
            if (angle >= 45 && angle < 225) { // positive x (FR, BL)
                if (rotationRate >= 0) { // clockwise
                    speedBL = speedBL * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    speedFR = speedFR * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            } else { // negative x (FR, BL)
                if (rotationRate >= 0) { // clockwise
                    speedFR = speedFR * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    speedBL = speedBL * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            }

            if (angle >= 135 && angle < 315) { // negative y (FL, BR)
                if (rotationRate >= 0) { // clockwise
                    speedFL = speedFL * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    speedBR = speedBR * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            } else { // positive y (FL, BR)
                if (rotationRate >= 0) { // clockwise
                    speedBR = speedBR * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    speedFL = speedFL * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            }*/

            _motorFL.setSpeed(speedFL+rotationRate);
            _motorFR.setSpeed(speedFR+rotationRate);
            _motorBL.setSpeed(speedBL+rotationRate);
            _motorBR.setSpeed(speedBR+rotationRate);
        };
        
    private:
        Motor _motorFL, _motorFR, _motorBL, _motorBR;
};

#endif
