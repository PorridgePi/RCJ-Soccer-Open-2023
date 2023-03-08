#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Definitions.h>
#include <Motor.h>

class Drive {
    public:
        Drive(Motor &motorFR, Motor &motorBR, Motor &motorBL, Motor &motorFL) : _motorFR(motorFR), _motorBR(motorBR), _motorBL(motorBL), _motorFL(motorFL) {}

        void setDrive(float speed, int angle, float rotationRate) {
            speed = constrain(speed, -1, 1);
            rotationRate = constrain(rotationRate, -1, 1);
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


            Serial.print("Before:\tspeedFR: ");
            Serial.print(speedFR);
            Serial.print("\tspeedBR: ");
            Serial.print(speedBR);
            Serial.print("\tspeedBL: ");
            Serial.print(speedBL);
            Serial.print("\tspeedFL: ");
            Serial.print(speedFL);

            if (angle >= 45 && angle < 225) { // positive x (FR, BL)
                if (rotationRate >= 0) { // clockwise
                    Serial.print("\tBL+");
                    speedBL = speedBL * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    Serial.print("\tFR-");
                    speedFR = speedFR * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            } else { // negative x (FR, BL)
                if (rotationRate >= 0) { // clockwise
                    Serial.print("\tFR+");
                    speedFR = speedFR * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    Serial.print("\tBL-");
                    speedBL = speedBL * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            }

            if (angle >= 135 && angle < 315) { // negative y (FL, BR)
                if (rotationRate >= 0) { // clockwise
                    Serial.print("\tFL+");
                    speedFL = speedFL * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    Serial.print("\tBR-");
                    speedBR = speedBR * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            } else { // positive y (FL, BR)
                if (rotationRate >= 0) { // clockwise
                    Serial.print("\tBR+");
                    speedBR = speedBR * (1 - 2 * rotationRate); // if rotationRate == 1, reverse direction
                } else { // counterclockwise
                    Serial.print("\tFL-");
                    speedFL = speedFL * (1 + 2 * rotationRate); // if rotationRate == -1, reverse direction
                }
            }

            Serial.print("\tAfter:\tspeedFR: ");
            Serial.print(speedFR);
            Serial.print("\tspeedBR: ");
            Serial.print(speedBR);
            Serial.print("\tspeedBL: ");
            Serial.print(speedBL);
            Serial.print("\tspeedFL: ");
            Serial.print(speedFL);
            Serial.println();

            _motorFL.setSpeed(speedFL);
            _motorFR.setSpeed(speedFR);
            _motorBL.setSpeed(speedBL);
            _motorBR.setSpeed(speedBR);
        }

    private:
        Motor _motorFL, _motorFR, _motorBL, _motorBR;
};

#endif
