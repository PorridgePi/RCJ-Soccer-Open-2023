#include <Arduino.h>
#include <CommonUtils.h>
#include <Definitions.h>
#include <DriveYK.h>
#include <IMU.h>
#include <MechaQMC5883.h>
#include <Lidar.h>
#include <PID.h>
#include <Wire.h>
#include <Kicker.h>

//// ** CONFIG ** ////
#define DEBUG_LED true
#define IS_CALIBRATING false
#define DEBUG_PRINT true
#define DEBUG_LOOP_TIME true

#ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
#define pixyXC 137 // x-coordinate of center of Pixy2 camera
#define pixyYC 111 // y-coordinate of center of Pixy2 camera
#else                   // BOT 2
#define pixyXC 140 // x-coordinate of center of Pixy2 camera
#define pixyYC 108 // y-coordinate of center of Pixy2 camera
#endif

#define IN_FRONT_FOV 7 // angle in front of robot to search for ball

#define SIGNATURE_BALL        1 // signature of ball
#define SIGNATURE_BLUE_GOAL   2 // signature of blue goal
#define SIGNATURE_YELLOW_GOAL 3 // signature of yellow goal

#define MAX_BALL_DIST_THRESHOLD 400 // max distance to ball before out of range
#define MIN_BALL_DIST_THRESHOLD 180 // min distance to ball
// #define BALL_FUNCTION_THRESHOLD 0.000323979729302f

#define LIGHT_GATE_DIFFERENCE_THRESHOLD 50

#define CONFX_MIN 0.6f
#define CONFY_MIN 0.5f
#define CONFX_POWER 1.9f
#define CONFY_POWER 2.0f

#define BORDER_DISTANCE 12 // 12 // set to 25 for testing on ri field

#define BALL_FUNCTION_THRESHOLD 0.0004f // constant for distance to cm conversion

#define SPEED_TO_TURN_RATE_RATIO 1

#ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
#define SPEED .8 // speed of robot (0.0 to 1.0)
#else                   // BOT 2
#define SPEED .8 // speed of robot (0.0 to 1.0)
#endif

#ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
#define SUM_X 181.0f
#define SUM_Y 225.0f
// in goal sum_y is 222
// outside of goal sum_y is 180
#else                   // BOT 2
#define SUM_X 178.0f
#define SUM_Y 230.0f
#endif

//// ** DECLARATIONS ** ////
// Pixy2 Camera
#include <Pixy2UART.h> // include official Pixy2 library
Pixy2UART pixy;        // Pixy object from official Pixy2 library

volatile int numBlocks;        // total number of blocks deteced by Pixy2, volatile for multicore access
Block        blocks[50];       // array of all blocks detected by Pixy2
Block        ballBlocks[10];   // array of ball blocks
Block        yellowBlocks[10]; // array of yellow goal blocks
Block        blueBlocks[10];   // array of blue goal blocks


// Ball
float ballAngle;    // angle of ball relative to robot (0 to 360 degrees)
int ballDistance; // distance to ball (arbitrary units due to non-linear relationship caused by mirror distortion)
int emptyLightGateThreshold;

// Goals
float goalAngle;
float ownGoalAngle;
int goalDistance;
bool isGoalYellow; // colour of goal to score on (YELLOW or BLUE)
bool isGoalDetermined = false;

// Bottom Plate
bool isOnLine; // true (1) if robot is on line (i.e. any bottom plate TEMT6000 exceeds threshold), false (0) if not
bool isBallInFront;
bool isBallCaptured;

size_t timeAtLastRise;
size_t timeBetweenSubsequentRises;
float lineAngle;

unsigned long t;
float dt;

bool wasOnLine = false;
float confXPower = CONFX_POWER;

static float averageLastSpeed = 0;

// Movement
#ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
Motor motorFR(PIN_BOT_B_LPWM, PIN_BOT_B_RPWM);
Motor motorBR(PIN_BOT_A_RPWM, PIN_BOT_A_LPWM);
Motor motorBL(PIN_TOP_B_RPWM, PIN_TOP_B_LPWM);
Motor motorFL(PIN_TOP_A_RPWM, PIN_TOP_A_LPWM);
#else                   // BOT 2
Motor motorFR(PIN_BOT_B_LPWM, PIN_BOT_B_RPWM);
Motor motorBR(PIN_BOT_A_LPWM, PIN_BOT_A_RPWM);
Motor motorBL(PIN_TOP_B_RPWM, PIN_TOP_B_LPWM);
Motor motorFL(PIN_TOP_A_LPWM, PIN_TOP_A_RPWM);
#endif
Drive driveBase(motorFR, motorBR, motorBL, motorFL); // drive base controlling all 4 motors
float speed = SPEED;
float speedX, speedY, moveAngle;

// LiDAR
#ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
Lidar lidarFront(Wire, 0x12, +6); // front LiDAR
Lidar lidarRight(Wire, 0x13, +4); // right LiDAR
Lidar lidarBack(Wire, 0x11, +6);  // back LiDAR
Lidar lidarLeft(Wire, 0x10, +2);  // left LiDAR
#else                   // BOT 2
Lidar lidarFront(Wire, 0x11, +8); // front LiDAR
Lidar lidarRight(Wire, 0x13, +4); // right LiDAR
Lidar lidarBack(Wire, 0x12, +4);  // back LiDAR
Lidar lidarLeft(Wire, 0x10, +3);  // left LiDAR
#endif

float frontDist, backDist, rightDist, leftDist, rawBackDist; // distance to obstacles

int x, y; // coordinate of robot relative to field

// IMU
// IMU imu(Wire1, 0x1E); // IMU providing heading
#ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
MechaQMC5883 imu(Wire1, -190, -236, 1.03225739008, 53.8968998045);
#else                   // BOT 2
IMU imu(Wire1, 64.5, -22, 0.983015342964, 49.1136970449);
#endif

volatile float botHeading; // heading of robot (0 to 360 degrees)
float rotateCommand; // for compass correction (-180 to 180 degrees)
float goalRotateAngle;

#ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
// Kicker kicker(PIN_RELAY);
#endif

//// ** FUNCTIONS ** ////
// continuous async blink LED to indicate program is running and Pico has not hang
void blinkLED(int interval = 50) {
    static unsigned long lastMillis = 0;     // last time LED was toggled
    static bool          ledState   = false; // true = LED on, false = LED off
    if (millis() - lastMillis >= interval) {
        lastMillis = millis();
        if (ledState) {
            digitalWrite(PIN_LED, LOW);
            ledState = false;
        } else {
            digitalWrite(PIN_LED, HIGH);
            ledState = true;
        }
    }
}

// Localisation
// move to coordinates (x, y) with tolerance in cm
void moveTo(int targetX, int targetY, int tolerance) {
    float targetAngle = DEG(atan2(targetY - y, targetX - x)) + 90;
    float dist        = hypot(targetX - x, targetY - y); // distance to target coordinates

    if (dist > tolerance) { // not reached target, move
        moveAngle = LIM_ANGLE(targetAngle);
        float maxSpeed = SPEED * constrain(powf((dist / 150.0f), 0.3f), 0, 1);
        speed = constrain(speed, -maxSpeed, maxSpeed);
    } else { // reached target, stop
        speed = 0;
    }
}

// update robot coordinates using LiDAR
void updatePosition() {
    float angle = botHeading <= 180 ? botHeading : 360 - botHeading; // correct for robot rotation (0 to 180 degrees)
    if (angle > 30 || angle < -30) {
        angle = 0;
    }
    leftDist    = constrain(lidarLeft.read() * cosf(RAD(angle)), 0, 255);
    rightDist   = constrain(lidarRight.read() * cosf(RAD(angle)), 0, 255);
    x = constrain((leftDist + 182 - rightDist) / 2, 0, 255);
    int yCorrection = 20; // ACTUAL FIELD TODO NOTE IMPORTANT

    // yCorrection = 12 - 7.4;
    if (x >= 61 && x <= 121) {
        yCorrection = 20; // ACTUAL FIELD TODO NOTE IMPORTANT
    } else {
        yCorrection = 0; // ACTUAL FIELD TODO NOTE IMPORTANT
    }
    
    rawBackDist = lidarBack.read() * cosf(RAD(angle));
    frontDist   = constrain(lidarFront.read() * cosf(RAD(angle)) + yCorrection, 0, 255);
    backDist    = constrain(rawBackDist + yCorrection, 0, 255);
    y = constrain((frontDist + 243 - backDist) / 2, 0, 255);
}

void deconstructSpeed() {
    speedX = speed * sin(RAD(moveAngle));
    speedY = speed * cos(RAD(moveAngle));
}

void constructSpeed() {
    speed = sqrt(speedX * speedX + speedY * speedY);
    moveAngle = LIM_ANGLE(DEG(atan2(speedX, speedY)));
}

void stayWithinBounds() {
    const int borderToleranceX = 9;
    const int borderToleranceY = 9;

    deconstructSpeed();

    static float multiplierX;
    static float distanceX;
    int maxXL, maxXR, maxX;

    // consider right border
    if (x < (91 - 40) && y < (BORDER_DISTANCE + 25) || y > (243 - (BORDER_DISTANCE + 25))) { // bot to the left of goal
        maxXR = 91 - 40;
    } else {
        maxXR = 182 - BORDER_DISTANCE - borderToleranceX;
    }

    // consider left border
    if (x > (91 + 40) && y < (BORDER_DISTANCE + 25) || y > (243 - (BORDER_DISTANCE + 25))) { // bot to right of goal
        maxXL = 91 + 40;
    } else {
        maxXL = BORDER_DISTANCE + borderToleranceX;
    }

    // const int angleRange = 0;
    // if (abs(ANGLE_360_TO_180(moveAngle)) <= angleRange || abs(ANGLE_360_TO_180(moveAngle)) >= (180 - angleRange)) {
    //     if (maxXR < maxXL) { // consider right border
    //         // EPRINT("ri ");
    //         maxX = maxXR;
    //         distanceX = max(0, maxX - x);
    //     } else { // consider left border
    //         // EPRINT("le ");
    //         maxX = maxXL;
    //         distanceX = x - maxX;
    //     }
    // } else {
        if (moveAngle > 0 && moveAngle <= 180) { // moving right
            // EPRINT("ri ");
            maxX = maxXR;
            distanceX = maxX - x;
        } else if (moveAngle > 180 && moveAngle < 360) {
            // EPRINT("le ");
            maxX = maxXL;
            distanceX = x - maxX;
        }
    // }

    // increase confidence power if near border
    const int minDistanceX = 12;
    if (distanceX < minDistanceX) {
        confXPower = CONFX_POWER + powf((float) (minDistanceX - distanceX) / minDistanceX, 0.5) * 2;
    } else {
        confXPower = CONFX_POWER;
    }

    // DPRINT(confXPower);

    multiplierX = constrain(1 * distanceX / 91, -1, 1); // TODO IMPORTANT INCREASE 1.7 ON ACTUAL FIELD 
    speedX = constrain(abs(speedX), 0, abs(multiplierX)) * copysign(1, speedX) * copysign(1, multiplierX); // abs for magnitude, copysign for direction

    static float multiplierY;
    static float distanceY;
    int maxY;
    if (moveAngle >= 90 && moveAngle < 270) { // moving down
        // EPRINT("do ");
        maxY = min(243 - BORDER_DISTANCE, powf((9.0f / 160.0f * (x - 91.0f)), 4) + 243 - (25 + BORDER_DISTANCE)) - borderToleranceY; // 38 is wall to goal border
        distanceY = maxY - y;
        multiplierY = powf(constrain(2.0f * abs(distanceY) / 121.5f, -1, 1), 0.8f) * copysign(1, distanceY); // tuning - change the 1.0f multiplier and 1.0f power
    } else if (moveAngle >= 270 || moveAngle < 90) { // moving up
        // EPRINT("up");
        maxY = max(BORDER_DISTANCE, - powf((9.0f / 160.0f * (x - 91.0f)), 4) + (25 + BORDER_DISTANCE)) + borderToleranceY - 20;
        multiplierY = powf(constrain(1.4f * abs(distanceY) / 121.5f, -1, 1), 1.0f) * copysign(1, distanceY); // tuning - change the 1.0f multiplier and 1.0f power
        distanceY = y - maxY;
    }

    speedY = constrain(abs(speedY), 0, abs(multiplierY)) * copysign(1, speedY) * copysign(1, multiplierY); // abs for magnitude, copysign for direction

    // if (copysign(1, distanceX) == -1) {
    //     speedX = copysign(1, speedX) * max(1.0f, abs(speedX) * 2);
    // }

    // speedX *= min(1.0f, powf((1 - averageLastSpeed), 2));
    // EPRINT(min(1.0f, powf((1 - averageLastSpeed), 2)));
    // speedY *= min(1.0f, powf((1 - averageLastSpeed), 1));

    constructSpeed();

    // DPRINT(speedX);
    // DPRINT(speedY);
    // DPRINT(maxX);
    // DPRINT(maxY);
    // DPRINT(multiplierX);
    // DPRINT(multiplierY);
    // DPRINT(speedX);
    // DPRINT(speedY);
    // DPRINT(speed);
    // DPRINT(moveAngle);
    // DPRINT(distanceX);
    // DPRINT(distanceY);
}

float confX, confY;

void confidence() {
    float sumX = leftDist + rightDist;
    float sumY = frontDist + backDist;

    // CALIBRATE ZERO ERROR
    if (IS_CALIBRATING) {
        static float avgSumX = sumX;
        static float avgSumY = sumY;
        avgSumX = (avgSumX * 99 + sumX) / 100;
        avgSumY = (avgSumY * 99 + sumY) / 100;

        Serial.print("avgSumX");
        Serial.print("\t");
        Serial.print(avgSumX);
        Serial.print("\t");
        Serial.print("avgSumY");
        Serial.print("\t");
        Serial.print(avgSumY);
        Serial.print("\t");

        Serial.print("sumX");
        Serial.print("\t");
        Serial.print(sumX);
        Serial.print("\t");
        Serial.print("sumY");
        Serial.print("\t");
        Serial.print(sumY);
        Serial.print("\t");
        Serial.print("confX");
        Serial.print("\t");
        Serial.print(confX);
        Serial.print("\t");
        Serial.print("confY");
        Serial.print("\t");
        Serial.print(confY);
        Serial.print("\t");
    }

    confX = constrain(sumX / SUM_X, 0, 1);
    confY = constrain(sumY / SUM_Y, 0, 1);

    // DPRINT(confX);
    // DPRINT(confY);

    if (confX > CONFX_MIN) {
        confX = powf((confX - CONFX_MIN)/(1 - CONFX_MIN), confXPower);
    } else {
        confX = 0;
    }

    if (confY > CONFY_MIN) {
        confY = powf((confY - CONFY_MIN)/(1 - CONFY_MIN), CONFY_POWER);
    } else {
        confY = 0;
    }

    // int maxDist = BORDER_DISTANCE + 9; // include radius of robot
    // if (confX > 0 && (leftDist + rightDist) < 2 * maxDist) {
    //     confX = 0;
    // }
    // if (confY > 0 && (frontDist + backDist) < 2 * maxDist) {
    //     confY = 0;
    // }

    deconstructSpeed();
    float ogSpeed = speed;
    float ogSpeedX = speedX;
    float ogSpeedY = speedY;
    // const float MIN_SPEED = 0;
    // float maxXSpeed = MIN_SPEED + (SPEED - MIN_SPEED) * pow(confX, POWER);
    // float maxYSpeed = MIN_SPEED + (SPEED - MIN_SPEED) * pow(confY, POWER);
    speedX = constrain(speedX, -confX, confX);
    speedY = constrain(speedY, -confY, confY);
    constructSpeed();

    // DPRINT(maxXSpeed);
    // DPRINT(maxYSpeed);
    // DPRINT(confX);
    // DPRINT(confY);

    if (!wasOnLine && speed > 0.2 && confX < 0.5 && confY > 0.8 && abs(ogSpeedY) < 0.2) { // if originally trying to move in x direction but now not moving
        speedY = copysign(0.2, ogSpeedY);
        constructSpeed();
    } else if (!wasOnLine && speed > 0.2 && confY < 0.5 && confX > 0.8 && abs(ogSpeedX) < 0.2) { // if originally trying to move in y direction but now not moving
        speedX = copysign(0.2, ogSpeedX);
        constructSpeed();
    }

    // forced compass correction
    if (abs(ANGLE_360_TO_180(botHeading)) > 5 && speed < 0.2) {
        speed = max(0.3, SPEED / 3);
    }

    if (wasOnLine == true && isOnLine == false && confX > 0.3 && confY > 0.3) {
        wasOnLine = false;
    }
}

// Ball Capture Zone
// sets isBallCaptured to true (1) if ball is captured, false (0) if not
void readBallCap() {
    isBallCaptured = analogRead(PIN_BALL_CAP_ANALOG) < (emptyLightGateThreshold - LIGHT_GATE_DIFFERENCE_THRESHOLD);
}

int calibrateLightGate() {
    delay(500);
    int sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(PIN_BALL_CAP_ANALOG);
    }
    return sum / 100;
}

// Camera
// gets distance to ball (arbitrary units), returns -1 if no ball detected
int getBallDistance() {
    Block ballBlock = ballBlocks[0]; // get first ball block ONLY (Potential improvement?)
    if (ballBlock.m_signature == SIGNATURE_BALL) {
        int xDiff = ballBlock.m_x - pixyXC;
        int yDiff = ballBlock.m_y - pixyYC;

        // max distance = 5 * 84 = 420 since max change is around 84 pixels
        return constrain(5 * hypot(xDiff, yDiff), 0, 3000);
    } else {
        return -1; // return -1 if no ball detected
    }
}

int getGoalDistance() {
    int dist = -1;
    float maxSize = 0;
    for (int i = 0; i < numBlocks; i++) {
        Block block = (isGoalYellow == true ? yellowBlocks[i] : blueBlocks[i]);
        if (block.m_signature == (isGoalYellow == true ? SIGNATURE_YELLOW_GOAL : SIGNATURE_BLUE_GOAL)) {
            float area = block.m_height * block.m_width;
            if (area > maxSize) {
                int xDiff = block.m_x - pixyXC;
                int yDiff = block.m_y - pixyYC;
                maxSize = area;
                dist = 5 * hypot(xDiff, yDiff);
            }
        }
    }
    
    return dist;
}

// gets angle of ball relative to robot (0 to 360 degrees), returns -1 if no ball detected
int getBallAngle() {
    Block ballBlock = ballBlocks[0]; // uses first ball block ONLY (Potential improvement?)
    if (ballBlock.m_signature == SIGNATURE_BALL) {
        int xDiff = ballBlock.m_x - pixyXC;
        int yDiff = ballBlock.m_y - pixyYC;

        float angle = atan2(yDiff, xDiff) * 180 / PI + 90;
        return 360 - LIM_ANGLE(angle);
    } else {
        return -1; // return -1 if no ball detected
    }
}

float getGoalAngle(int numBlocks, bool isGoalYellow) {
    float angle;
    float maxSize = 0;
    for (int i = 0; i < numBlocks; i++) {
        Block block = (isGoalYellow == true ? yellowBlocks[i] : blueBlocks[i]);
        if (block.m_signature == (isGoalYellow == true ? SIGNATURE_YELLOW_GOAL : SIGNATURE_BLUE_GOAL)) {
            float area = block.m_height * block.m_width;
            if (area > maxSize) {
                int xDiff = block.m_x - pixyXC;
                int yDiff = block.m_y - pixyYC;
                maxSize = area;
                angle = atan2(yDiff, xDiff) * 180 / PI + 90;
            }
        }
    }
    return 360 - LIM_ANGLE(angle);
}

// categorises blocks into ball, yellow goal, and blue goal, and updates ball angle and distance
void categoriseBlock() {
    int numBall = 0, numYellow = 0, numBlue = 0; // to be used as index for arrays
    for (int i = 0; i < numBlocks; i++) {
        Block block = blocks[i];
        if (block.m_signature == SIGNATURE_BALL) {
            ballBlocks[numBall] = block;
            numBall++;
        } else if (block.m_signature == SIGNATURE_YELLOW_GOAL) {
            yellowBlocks[numYellow] = block;
            numYellow++;
        } else if (block.m_signature == SIGNATURE_BLUE_GOAL) {
            blueBlocks[numBlue] = block;
            numBlue++;
        }
    }

    // Update ball angle and distance
    static unsigned long ballLastMillis = millis(); // last time ball was detected
    static unsigned long ballLastInFrontMillis = millis(); //last time ball was in front
    if (millis() - ballLastInFrontMillis > 0) { // timeout disabled
        isBallInFront = false;
    }
    if (numBall > 0) {
        ballAngle    = getBallAngle();
        ballDistance = getBallDistance();
        ballLastMillis = millis();
        if ((ballAngle < IN_FRONT_FOV || ballAngle > (360 - IN_FRONT_FOV))) {
            isBallInFront = true;
            ballLastInFrontMillis = millis();
        }
    } else { // no ball detected
        // if no ball detected for 1 second, reset ball angle and distance
        // time is to prevent false reset (i.e. due to lag or blind spot)
        if (millis() - ballLastMillis > 500) {
            ballAngle    = -1;
            ballDistance = -1;
        }
    }

    // Update goal angle and distance
    static unsigned long goalLastMillis = millis(); // last time goal was detected
    if ((isGoalYellow == true ? numYellow : numBlue) > 0) {
        goalDistance = getGoalDistance();
        goalDistance = powf(((goalDistance-200.0f)/27.0f), 2);
        goalAngle = getGoalAngle((isGoalYellow == true ? numYellow : numBlue), isGoalYellow);
    } else { // no goal detected
        // if no goal detected for 1 second, reset goal angle and distance
        // time is to prevent false reset (i.e. due to lag or blind spot)
        if (millis() - goalLastMillis > 1000) {
            goalAngle = -1;
            goalDistance = -1;
            goalLastMillis = millis();
        }
    }
    static unsigned long ownGoalLastMillis = millis(); // last time goal was detected
    if ((!isGoalYellow == true ? numYellow : numBlue) > 0) {
        ownGoalAngle = getGoalAngle((!isGoalYellow == true ? numYellow : numBlue), !isGoalYellow);
    } else { // no goal detected
        // if no goal detected for 1 second, reset goal angle and distance
        // time is to prevent false reset (i.e. due to lag or blind spot)
        if (millis() - ownGoalLastMillis > 1000) {
            goalAngle = -1;
            ownGoalLastMillis = millis();
        }
    }
}

float ballTrack() {
    // Move perpendicular to ball if near, move straight if far

    if (ballAngle == -1 || ballDistance == -1) {
        return -1;
    }

    float ballDistInCm = BALL_FUNCTION_THRESHOLD * ballDistance * ballDistance; // converts ball distance to cm

    float angle = 0;

    // if (ballAngle < 13 || ballAngle > 347) { // ball is in front (TODO: ball cap light gate integration?)
    //     if (ballDistance < 200) { // ball is near
    //         return -1; 
    //     }
    //     angle = 0; // if far, move forward
    // } else

    // negative number with float power will return inf!
    if (ballAngle >= 0 && ballAngle < 20) { // front right
        angle = ballAngle + 90 * (1 - pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 0.7));
    } else if (ballAngle >= 20 && ballAngle < 45) { // front right
        angle = ballAngle + 90 * (1 - pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 0.9));
    } else if (ballAngle >= 45 && ballAngle < 90) { // front right
        angle = ballAngle + 90 * (1 - pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1.0));
    } else if (ballAngle >= 90 && ballAngle < 180) {
        angle = ballAngle + 90 * (1 - pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1.0));
    } else if (ballAngle >= 180 && ballAngle < 270) {
        angle = ballAngle - 90 * (1 - (pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1.0)));
    } else if (ballAngle >= 270 && ballAngle < 315) { // front left
        angle = ballAngle - 90 * (1 - (pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1.0)));
    } else if (ballAngle >= 315 && ballAngle < 340) { // front left
        angle = ballAngle - 90 * (1 - (pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 0.9)));
    } else if (ballAngle >= 340 && ballAngle < 360) { // front left
        angle = ballAngle - 90 * (1 - (pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 0.7)));
    }

    // DPRINT(ballDistInCm);
    // DPRINT(angle);

    return LIM_ANGLE(angle);
}

void setGoalColour() {
    float blueAngle, yellowAngle;
    float blueMaxSize = 0, yellowMaxSize = 0;
    for (int i = 0; i < 10; i++) {
        Block block = blueBlocks[i];
        if (block.m_signature == SIGNATURE_BLUE_GOAL) {
            float area = block.m_height * block.m_width;
            if (area > blueMaxSize) {
                int xDiff = block.m_x - pixyXC;
                int yDiff = block.m_y - pixyYC;
                blueMaxSize = area;
                blueAngle = atan2(yDiff, xDiff) * 180 / PI + 90;
            }
        }
    }
    for (int i = 0; i < 10; i++) {
        Block block = yellowBlocks[i];
        if (block.m_signature == SIGNATURE_YELLOW_GOAL) {
            float area = block.m_height * block.m_width;
            if (area > yellowMaxSize) {
                int xDiff = block.m_x - pixyXC;
                int yDiff = block.m_y - pixyYC;
                yellowMaxSize = area;
                yellowAngle = atan2(yDiff, xDiff) * 180 / PI + 90;
            }
        }
    }
    blueAngle = ANGLE_360_TO_180(LIM_ANGLE(blueAngle));
    yellowAngle = ANGLE_360_TO_180(LIM_ANGLE(yellowAngle));

    if (abs(blueAngle) < 90 && abs(yellowAngle) > 90) { // blue goal in front, yellow goal behind
        // EPRINT("BLUE");
        isGoalYellow = false;
        isGoalDetermined = true;
    } else if (abs(yellowAngle) < 90 && abs(blueAngle) > 90) { // yellow goal in front, blue goal behind
        // EPRINT("YELLOW");
        isGoalYellow = true;
        isGoalDetermined = true;
    } else {
        isGoalDetermined = false;
    }
}

// setup devices (LiDARs, IMU, bottom plate) - called in either core 0 or 1
void setupDevices() {
    // I2C for LiDAR
    Wire.setSCL(PIN_WIRE0_LUNA_SCL);
    Wire.setSDA(PIN_WIRE0_LUNA_SDA);
    Wire.setClock(400000);
    Wire.setTimeout(1); // set timeout to 1 ms
    Wire.begin();

    #define LIDAR_FPS 250
    lidarFront.setFPS(LIDAR_FPS);
    lidarBack.setFPS(LIDAR_FPS);
    lidarLeft.setFPS(LIDAR_FPS);
    lidarRight.setFPS(LIDAR_FPS);

    // I2C for IMU
    Wire1.setSCL(PIN_WIRE1_GY_SCL);
    Wire1.setSDA(PIN_WIRE1_GY_SDA);
    Wire1.setTimeout(1); // set timeout to 1 ms
    Wire1.begin();
    // imu.setCalibration(159, 32, 516, 530, -53);
    imu.init();
    imu.tare(); // set current angle as heading 0

    // Pin for bottom plate
    pinMode(PIN_BOTPLATE_D1, INPUT);

    pinMode(PIN_BALL_CAP_ANALOG, INPUT);
    emptyLightGateThreshold = calibrateLightGate();
}

// update data (LiDARs, IMU, bottom plate) - called in either core 0 or 1
void updateData() {
    // Update LiDARs not as frequently, using millis() to prevent lag
    static unsigned long lastMillis = 0;       // last time LiDARs were updated
    if (millis() - lastMillis >= 1000 / 300) { // 300 Hz
        updatePosition();
        lastMillis = millis();
    }
    botHeading  = imu.readAngle();                                   // from 0 to 360
    isOnLine    = digitalRead(PIN_BOTPLATE_D1);
    readBallCap();
    categoriseBlock();

    if (digitalRead(PIN_ORI_RESET) == HIGH) { // reset IMU if button pressed
        imu.tare();
        isGoalDetermined = false;
    }

    if (!isGoalDetermined) {
        setGoalColour();
    }

    if (isOnLine == true) { // failsafe: if was on line, move to the center
        wasOnLine = true;
    }
}

static float prevMoveAngle = 0;
static float prevSpeed = -1;
static unsigned long lastAimMillis = millis();

void moveToBallInFront() {
    float distanceScale = constrain(powf(max(0, (float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / (float) (MAX_BALL_DIST_THRESHOLD - MIN_BALL_DIST_THRESHOLD)), 2.0f), 0, 1);
    float speedConstrain = constrain(distanceScale, max(0.2, SPEED / 4), SPEED);
    speed = constrain(speed, -speedConstrain, speedConstrain);
    moveAngle = 0;
    prevMoveAngle = 0;
    prevSpeed = -1;
}

void aim() {
    static unsigned long lastKickerMillis = millis();

    lastAimMillis = millis();
    // if (goalDistance > 60) {
    //     moveAngle = goalAngle * 1.01;
    // } else {
    moveAngle = goalAngle;
    // }
    
    // comment out below to disable localisation based aiming
    // moveTo(91, 12, 2);
    
    DPRINT(moveAngle);
    // if (abs(moveAngle) < 20) {
    
    //     moveAngle *= 1.5;
    // }
    // moveAngle = constrain(prevMoveAngle + goalAngle * dt / 10, 0, goalAngle);

    // prevMoveAngle = moveAngle;
    // speed = constrain(powf(prevSpeed, 0.99f) + dt * (min(6, (1.0f-((goalDistance-40)/80.0f))*6.0f)) / 1000, 0, SPEED);
    // DPRINT(goalDistance);
    
    if (prevSpeed <= 0.1f) {
        // prevSpeed = max(0.1f, powf(constrain(1.0f-goalDistance/100.0f, 0, 1), 1.5f) - 0.4f);
        prevSpeed = 0.2f;
    }
    DPRINT(prevSpeed);
    // float goalDistVelocity = max(0.05f, powf(constrain(1.0f-goalDistance/100.0f, 0, 1), 1.0f));
    // speed = constrain(powf(prevSpeed, 0.99f) + dt * goalDistVelocity / 10000, 0, SPEED);
    
    speed = constrain(prevSpeed + dt * 12 / 10000, 0, SPEED);
    prevSpeed = speed;
    DPRINT(speed);

    // DPRINT(goalDistance);
    // DPRINT(goalDistVelocity);

    // EPRINT((min(6, (1.0f-((goalDistance-40)/100.0f))*6.0f)));
    // EPRINT(powf(prevSpeed, 0.5f));

    // if (goalAngle != -1) {  // goal present
    //     rotateCommand = constrain(ANGLE_360_TO_180(goalAngle)/30, -1, 1);
    //     moveAngle = rotateCommand * 30;
    // } else { // cannot see goal - use lidars?
    //     moveAngle = 0;
    // }
    // DPRINT(rotateCommand);

    #ifndef IS_SECOND_BOT   // BOT 1 WITH KICKER
    if (
        isBallCaptured &&
        (millis() - lastKickerMillis) > 2000 &&
        y < 100 && confY > 0.7 &&
        x > (91 - 20) && x < (91 + 20) && confX > 0.7
        // abs(ANGLE_360_TO_180(goalAngle)) < 30
        // speed > SPEED * 0.5
        ){
        // kicker.kick();
        lastKickerMillis = millis();
    }
    #endif
    // rotateCommand = constrain(log(abs(-ANGLE_360_TO_180(goalAngle)+1))*-1*copysign(1,goalAngle), -speed/10, speed/10);
}

void moveTrackBall() {
    // lastAimMillis = 0;
    moveAngle = ballTrack();
    float distanceScale = constrain(powf(max(0, (float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / (float) (MAX_BALL_DIST_THRESHOLD - MIN_BALL_DIST_THRESHOLD)), 2.0f), 0, 1);
    // DPRINT(distanceScale);
    float angleScale = constrain(powf((ballAngle < 180.0f ? ballAngle : 360.0f - ballAngle) / 180.0f, 0.9f) , 0, 1);
    // DPRINT(angleScale);
    float ballDistanceSpeed = constrain(distanceScale * powf((1 - angleScale), 4.0f) + angleScale, max(0.2, SPEED / 4), SPEED);
    speed = constrain(speed, -ballDistanceSpeed, ballDistanceSpeed);
}

//// ** MAIN ** ////
// core 0 setup
void setup() {
    // UART
    Serial.begin(9600);

    setupDevices();

    // LED
    pinMode(PIN_LED, OUTPUT);
}

// core 1 setup
void setup1() {
    pixy.init();
}

// PID
PID pid(0.01, 0, 0.02, 5000);

#ifndef IS_GOALIE
// core 0 loop
void loop() {
    dt = (micros() - t) / 1000;
    t = micros(); // loop time
    speed = SPEED;

    //// ** DATA UPDATE & PROCESSING ** ////
    updateData();

    //// ** STRATEGY ** ////
    // rotateCommand = constrain((LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)/540, -1, 1); // from -180 to 180
    // rotateCommand = constrain(abs(rotateCommand), 0.03, 1) * copysign(1, rotateCommand);
    // rotateCommand = constrain(pid.compute(0, (LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)), -1, 1);
    // Compass Correction

    #ifndef IS_SECOND_BOT  // BOT 1 WITH KICKER
    rotateCommand = -constrain((LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)/45, -1, 1);
    #else                  // BOT 2 WITHOUT KICKER
    rotateCommand = constrain(pid.compute(0, ANGLE_360_TO_180(LIM_ANGLE(botHeading))), -1, 1);
    #endif

    if (ballAngle == -1) { // no ball detected, move to centre
        #ifndef IS_SECOND_BOT
        moveTo(75, 122, 3);
        #else
        moveTo(94, 180, 3);
        #endif
    } else { // ball detected
        if (isBallInFront || isBallCaptured) { // ball in front
            if (!isBallCaptured && ballDistance > MIN_BALL_DIST_THRESHOLD + 20) { // ball far away, move towards ball //  && (millis() - lastAimMillis > 1000)
                moveToBallInFront();
            } else { // ball close enough, aim ISSUE
                // speed = 0;
                aim();
            }
        } else { // ball not in front, move towards it
            moveTrackBall();
            // prevSpeed = 0;
        }
    }

    //// ** LOCALISATION ** ////
    if (moveAngle == -1) { // stop if moveAngle is -1
        speed = 0;
    }

    // Staying within bounds
    stayWithinBounds();
    // Staying within bounds (failsafe) using TEMTs
    if (isOnLine == true) { // failsafe: if on line, move to the center
        #ifndef IS_SECOND_BOT
        moveTo(88, 122, 3);
        #else
        moveTo(94, 122, 2);
        #endif
        speed = max(0.3, SPEED/2);
    }

    confidence();

    //// ** MOVEMENT ** ////
    // driveBase.setDrive(speed, moveAngle, rotateCommand); //Speed multiplied to accomodate for differences in speed with the wheels
    driveBase.setDrive(speed, moveAngle, rotateCommand);

    averageLastSpeed = (averageLastSpeed * 99 + speed) / 100;
    // if (prevSpeed > 0) {
    //     prevSpeed = speed;
    // }

    //// ** DEBUG ** ////
    // DPRINT(moveAngle);
    // DPRINT(speed);
    DPRINT(botHeading);
    // DPRINT(rotateCommand);

    // DPRINT(isBallInFront);
    DPRINT(isBallCaptured);

    // DPRINT(isOnLine);
    // DPRINT(wasOnLine);

    DPRINT(confX);
    DPRINT(confY);

    DPRINT(ballAngle);
    DPRINT(goalAngle);
    DPRINT(ballDistance);
    DPRINT(goalDistance);
    DPRINT(ownGoalAngle);

    // DPRINT(frontDist);
    // DPRINT(backDist);
    // DPRINT(leftDist);
    // DPRINT(rightDist);
    DPRINT(x);
    DPRINT(y);

    if (IS_CALIBRATING) {
        Serial.print("Front: ");
        Serial.print(lidarFront.readRaw());
        Serial.print("\t");
        Serial.print(lidarFront.read());
        Serial.print("\t");
        Serial.print("Back: ");
        Serial.print(lidarBack.readRaw());
        Serial.print("\t");
        Serial.print(lidarBack.read());
        Serial.print("\t");
        Serial.print("Left: ");
        Serial.print(lidarLeft.readRaw());
        Serial.print("\t");
        Serial.print(lidarLeft.read());
        Serial.print("\t");
        Serial.print("Right: ");
        Serial.print(lidarRight.readRaw());
        Serial.print("\t");
        Serial.print(lidarRight.read());
        Serial.print("\t");
        Serial.print("Ball: ");
        Serial.print(ballAngle);
        Serial.print("\tBall Distance: ");
        Serial.print(ballDistance);
    }

    // Loop time
    if (!IS_CALIBRATING && DEBUG_LOOP_TIME) Serial.print((float)(micros()-t)/1000);
    if (!IS_CALIBRATING && DEBUG_PRINT || DEBUG_LOOP_TIME) Serial.println();

    if (DEBUG_LED) blinkLED();
}

// core 1 loop
void loop1() {
    pixy.ccc.getBlocks(); // get blocks from Pixy

    numBlocks = pixy.ccc.numBlocks;    // number of blocks detected
    memset(blocks, 0, sizeof(blocks)); // clear blocks array
    for (int i = 0; i < numBlocks; i++) {
        // copy blocks to array so that they can be accessed in main loop
        memcpy(&blocks[i], &pixy.ccc.blocks[i], sizeof(Block));
    }


}

#endif
