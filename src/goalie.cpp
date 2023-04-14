#define IS_GOALIE
#include <main.cpp>

bool isGoalie = true;

volatile float volatileBallAngle;
volatile float ballAngleRate;

PID interceptPid(0.1, 0, 0.1, 16667);

void loop() {
    float dt = (micros() - t) / 1000;
    t        = micros(); // loop time
    speed    = SPEED;
    //// ** DATA UPDATE & PROCESSING ** ////
    updateData();
    y = 243 - backDist;

    //// ** STRATEGY ** ////
    deconstructSpeed();
    if (isGoalie) {
        /*
        static bool atSecondPoint = false;
        DPRINT(atSecondPoint);
        if (ballAngle == -1) {
            if (y < 135) {
                atSecondPoint = true;
            } else {
                atSecondPoint = false;
            }
            if (!atSecondPoint) {
                if (x < 91) {
                    moveTo(43, 130, 3);
                } else {
                    moveTo(200, 130, 3);
                }

            } else {
                moveTo(91, 183, 3);
            }
        */
        if (ballAngle != -1 && abs(ANGLE_360_TO_180(ballAngle)) < 115) {
            // ball in front
            // atSecondPoint = false;
            float incomingBallAngle;
            if (ballAngle < 115) {
                incomingBallAngle = 180 - ownGoalAngle + ballAngle;
            } else {
                incomingBallAngle = ballAngle - ownGoalAngle - 180;
            }
            DPRINT(ballAngle);
            DPRINT(ownGoalAngle);
            DPRINT(incomingBallAngle);
            /*
                            if (incomingBallAngle > 0) { // move right
                                moveAngle = 90;
                                // moveAngle = constrain(1 / 21701390000000.0f * powf((x - 91.0f), 9.0f) + 90, 0.0f, 180.0f);
                            } else { // move lft
                                moveAngle = 270;
                                // moveAngle = constrain(1 / 21701390000000.0f * powf((x - 91.0f), 9.0f) + 90, 0.0f, 180.0f) + 180.0f;
                            }
            */

            speedX = interceptPid.compute(0, -incomingBallAngle); //powf(abs(incomingBallAngle) / 180.0f, 1.0f) * copysign(1, incomingBallAngle);
            // DPRINT(speedX);

            digitalWrite(PIN_LED, HIGH);
        } else {
            // moveTo(91, 186, 3);
            speedX = -(leftDist - rightDist) * 0.01f;
            // DPRINT(rawBackDist);

            digitalWrite(PIN_LED, LOW);

            /*
            else { // ball behind, ignore
                        if (y < 135) {
                //atSecondPoint = true;
            } else {
               // atSecondPoint = false;
            }
            if (!atSecondPoint) {
                if (x < 91) {
                    moveTo(43, 130, 3);
                } else {
                    moveTo(200, 130, 3);
                }
            } else {
                moveTo(91, 183, 3);
            }

                digitalWrite(PIN_LED, LOW);
            }
            */
        }

        static float distanceY;
        const int    borderToleranceY = 15;
        int          aimY             = 40; // 37;
        distanceY                     = aimY - rawBackDist;
        // DPRINT(rawBackDist)
        // DPRINT(distanceY);
        //  multiplierY = powf(constrain(1.4f * abs(distanceY) / 121.5f, -1, 1), 1.0f) * copysign(1, distanceY); // tuning - change the 1.0f multiplier and 1.0f power
        //   speedY = constrain(abs(speedY), 0, abs(multiplierY)) * copysign(1, speedY) * copysign(1, multiplierY); // abs for magnitude, copysign for direction
        //   speedY = multiplierY;
        speedY = distanceY / 20;
        constructSpeed();
        speed = constrain(speed, -0.5, 0.5);
        /*
                deconstructSpeed();
                static float multiplierY;
                static float distanceY;
                int          maxY;
                const int    borderToleranceY = 15;
                maxY                          = 243 - BORDER_DISTANCE - 25 - borderToleranceY;
                int aimY                      = 183;
                distanceY                     = y - aimY;
                DPRINT(distanceY);
                // multiplierY = powf(constrain(1.4f * abs(distanceY) / 121.5f, -1, 1), 1.0f) * copysign(1, distanceY); // tuning - change the 1.0f multiplier and 1.0f power
                //DPRINT(multiplierY);
                // speedY = constrain(abs(speedY), 0, abs(multiplierY)) * copysign(1, speedY) * copysign(1, multiplierY); // abs for magnitude, copysign for direction
                // speedY = multiplierY;
                speedY = distanceY / 50;
                DPRINT(speedY);
                constructSpeed();
                */
    } else { // striker
    }

    //// ** LOCALISATION ** ////
    if (moveAngle == -1) { // stop if moveAngle is -1
        speed = 0;
    }

    // Staying within bounds
    stayWithinBounds();
    // Staying within bounds (failsafe) using TEMTs
    // if (wasOnLine == true) { // failsafe: if on line, move to the center
    //     moveTo(91, 122, 2);
    //     speed = 0.3;
    // }

    // confidence();

    //// ** MOVEMENT ** ////
    rotateCommand = constrain(-2 * ANGLE_360_TO_180(botHeading)/180, -1, 1);
    if (abs(ANGLE_360_TO_180(botHeading) > 2) && speed < 0.5) {
        speed = 0.2;
    }
    //driveBase.setDrive(speed, moveAngle, rotateCommand);
    driveBase.setDrive(speed, moveAngle, rotateCommand);
    averageLastSpeed = (averageLastSpeed * 99 + speed) / 100;

    //// * DEBUG * ////
    // DPRINT(isBallInFront);
    // DPRINT(isBallCaptured);
    // DPRINT(moveAngle);
    // // DPRINT(goalAngle);
    // // DPRINT(isOnLine);
    // DPRINT(ballAngle);
    // DPRINT(ownGoalAngle);

    // DPRINT(goalAngle);
    // // DPRINT(ballDistance);
    // DPRINT(botHeading);
    // DPRINT(frontDst);
    // DPRINT(backDist)i;
    // DPRINT(leftDist);
    // DPRINT(rightDist);
    // DPRINT(x);
    // DPRINT(y);
    // DPRINT(moveAngle);
    // DPRINT(speed);
    // DPRINT(pixyXC);
    // DPRINT(pixyYC);

    // DPRINT(lidarFront.read());
    // DPRINT(lidarBack.read());
    // DPRINT(lidarLeft.read());
    // DPRINT(lidarRight.read());

    // if (DEBUG_LED) blinkLED();
    Serial.println();
}

void loop1() {
    pixy.ccc.getBlocks(); // get blocks from Pixy

    numBlocks = pixy.ccc.numBlocks;    // number of blocks detected
    memset(blocks, 0, sizeof(blocks)); // clear blocks array
    for (int i = 0; i < numBlocks; i++) {
        // copy blocks to array so that they can be accessed in main loop
        memcpy(&blocks[i], &pixy.ccc.blocks[i], sizeof(Block));
    }

    //categoriseBlock();

    // DPRINT(ballAngle);
}
