#define IS_GOALIE
#include <main.cpp>

bool isGoalie = true;

void loop() {
    float dt = (micros() - t) / 1000;
    t = micros(); // loop time
    speed = SPEED;
    //// ** DATA UPDATE & PROCESSING ** ////
    updateData();

    //// ** STRATEGY ** ////
    if (isGoalie) {
        DPRINT(y);
        if (x > (91 - 40 + 5) && x < (91 + 40 + 5)) {
            y = 243 - backDist;
        }
        DPRINT(y);
        DPRINT(backDist);
        DPRINT(confY);
        // DPRINT(moveAngle);
        // EPRINT('a');
            deconstructSpeed();
            if (abs(243 - BORDER_DISTANCE - 25 - 10 - y) > 10) {
                speedY = -(243 - BORDER_DISTANCE - 25 - 10 - y) * 0.05;
                DPRINT(speedY);
            }
            speedY;
        if (ballAngle == -1 || abs(ANGLE_360_TO_180(ballAngle)) > 135) {
            moveTo(91, 243 - BORDER_DISTANCE - 25 - 10, 5);
        } else {
            float incomingBallAngle = (180 - abs(ownGoalAngle - ballAngle)) * copysign(1, ownGoalAngle - ballAngle);
            DPRINT(incomingBallAngle);

            moveTo(x + 20 * incomingBallAngle/90, 243 - BORDER_DISTANCE - 25 - 10, 5);
            // speed = SPEED; // overwrite moveTo speed

            DPRINT(moveAngle);
            if (incomingBallAngle > 0) { // move right
                moveAngle = constrain(1 / 21701390000000.0f * powf((x - 91.0f), 9.0f) + 90, 0.0f, 180.0f);
            } else { // move left
                moveAngle = constrain(1 / 21701390000000.0f * powf((x - 91.0f), 9.0f) + 90, 0.0f, 180.0f) + 180.0f;
            }
            if (abs(incomingBallAngle) < 5) {
                speed = 0.1;
            }


            speedY;

            // speed = 0;
        }

        DPRINT(ballAngle);
        DPRINT(ballDistance);
        DPRINT(ownGoalAngle);
        DPRINT(moveAngle);
        DPRINT(botHeading);

        DPRINT(x);
        DPRINT(y);
    } else { // striker

    }
    //// ** LOCALISATION ** ////
    if (moveAngle == -1) { // stop if moveAngle is -1
        speed = 0;
    }

    // Staying within bounds
    stayWithinBounds();
    // Staying within bounds (failsafe) using TEMTs
    // if (isOnLine == true) { // failsafe: if on line, move to the center
    //     moveTo(91, 122, 2);
        
    //     speed = 0.3;
    // }

    confidence();

    //// ** MOVEMENT ** ////
    rotateCommand = constrain(pid.compute(0, ANGLE_360_TO_180(botHeading)), -1, 1);
    if (abs(rotateCommand) > 0.01 && speed < 0.2) {
        speed = 0.2;
    }
    driveBase.setDrive(speed, moveAngle, rotateCommand);
    averageLastSpeed = (averageLastSpeed * 99 + speed) / 100;

    //// * DEBUG * ////
    // DPRINT(isBallInFront);
    // DPRINT(isBallCaptured);
    // DPRINT(moveAngle);
    // // DPRINT(goalAngle);
    // // DPRINT(isOnLine);
    // DPRINT(ballAngle);

    // DPRINT(goalAngle);
    // // DPRINT(ballDistance);
    // DPRINT(botHeading);
    DPRINT(frontDist);
    DPRINT(backDist);
    DPRINT(leftDist);
    DPRINT(rightDist);
    // DPRINT(x);
    // DPRINT(y);

    // DPRINT(lidarFront.read());
    // DPRINT(lidarBack.read());
    // DPRINT(lidarLeft.read());
    // DPRINT(lidarRight.read());

    // Loop time
    if (DEBUG_LOOP_TIME) Serial.print((float)(micros()-t)/1000);
    if (DEBUG_PRINT || DEBUG_LOOP_TIME) Serial.println();

    if (DEBUG_LED) blinkLED();
}
