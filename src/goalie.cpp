#define IS_GOALIE
#include <main.cpp>

bool isGoalie = true;

volatile float volatileBallAngle;
volatile float ballAngleRate;

PID interceptPid(0.3, 0, 0.2, 16667);

void loop() {
    float dt = (micros() - t) / 1000;
    t        = micros(); // loop time
    speed    = SPEED;
    //// ** DATA UPDATE & PROCESSING ** ////
    updateData();
    y = 243 - backDist;

    static int lastBallAngle = ballAngle;
    static unsigned long lastBallChange = millis();
    static unsigned long lastBallNotSeen = millis();

    DPRINT(isGoalie);

    if (isGoalie && ballAngle != -1 && abs(lastBallAngle - ballAngle) < 3) {
        EPRINT("aaaa");
        if (millis() - lastBallChange > 3000) {
            isGoalie = false;
        }
    } else {
        lastBallAngle = ballAngle;
        // isGoalie = true;
        lastBallChange = millis();
    }

    if (!isGoalie && ballAngle == -1) {
        if (millis() - lastBallNotSeen > 1000) {
            isGoalie = true;
        }
    } else {
        lastBallNotSeen = millis();
    }

    //// ** STRATEGY ** ////
    deconstructSpeed();
    if (isGoalie) {
        if (ballAngle != -1 && abs(ANGLE_360_TO_180(ballAngle)) < 115) {
            // ball in front
            float incomingBallAngle;
            if (ballAngle < 115) { // 0 - 115
                incomingBallAngle = 180 - ownGoalAngle + ballAngle;
            } else { // 245 - 360
                incomingBallAngle = ballAngle - ownGoalAngle - 180;
            }

            speedX = powf(constrain(abs(incomingBallAngle) / 90.0f, 0, 3), 1.0f) * copysign(1, incomingBallAngle);
            if (abs(incomingBallAngle) < 5) {
                speedX = 0;
            }

            digitalWrite(PIN_LED, HIGH);
        } else {
            // moveTo(91, 186, 3);
            const float maxSpeedX = 0.3f;
            speedX = constrain(-(leftDist - rightDist) * 0.01f, -maxSpeedX, maxSpeedX);

            digitalWrite(PIN_LED, LOW);

        }

        static float distanceY;
        const int    borderToleranceY = 15;
        int          aimY             = 40; // 37;
        distanceY                     = aimY - rawBackDist;
        DPRINT(distanceY);
        const float maxSpeedY = 0.3;
        speedY = maxSpeedY * powf(constrain(abs(distanceY) / 20.0f, 0.0f, 1.0f), 1) * copysign(1, distanceY);
        DPRINT(speedY);
        constructSpeed();

        speed = constrain(speed, -1, 1);
        DPRINT(speed);
        DPRINT(moveAngle);
        if (speed < 0.15) {
            speed = 0;
        }
    } else { // striker
        if (ballAngle == -1) { // no ball detected, move to centre
            moveTo(91, 122, 3);
        } else { // ball detected
            if (isBallInFront || isBallCaptured) { // ball in front
                if (ballDistance > MIN_BALL_DIST_THRESHOLD + 10 && (millis() - lastAimMillis > 1000)) { // ball far away, move towards ball
                    moveToBallInFront();
                } else { // ball close enough, aim ISSUE
                    // speed = 0;
                    aim();
                }
            } else { // ball not in front, move towards it
                moveTrackBall();
            }
        }

        //// ** LOCALISATION ** ////
        if (moveAngle == -1) { // stop if moveAngle is -1
            speed = 0;
        }

        // Staying within bounds
        stayWithinBounds();
        // Staying within bounds (failsafe) using TEMTs
        if (wasOnLine == true) { // failsafe: if on line, move to the center
            moveTo(91, 122, 2);
            speed = max(0.3, SPEED/2);
        }

        confidence();

    }

    //// ** LOCALISATION ** ////
    if (moveAngle == -1) { // stop if moveAngle is -1
        speed = 0;
    }

    // Staying within bounds
    // stayWithinBounds();
    // Staying within bounds (failsafe) using TEMTs
    if (isOnLine == true) { // failsafe: if on line, move to the center
        moveTo(91, 122, 2);
        speed = 0.3;
    }

    if (!isGoalie) confidence();

    //// ** MOVEMENT ** ////
    rotateCommand = constrain(-3 * ANGLE_360_TO_180(botHeading)/180, -1, 1);
    rotateCommand = constrain((0.1 + abs(rotateCommand)) * copysign(1, rotateCommand), -1, 1);
    if (abs(botHeading) > 3 && abs(speed) < 0.2) { // if trying torotate but below the minimum speed?
        speed = 0.2f;
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

    // categoriseBlock();

    // DPRINT(ballAngle);
}
