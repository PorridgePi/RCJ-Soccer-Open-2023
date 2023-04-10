#include <Arduino.h>
#include <CommonUtils.h>
#include <Definitions.h>
#include <Drive.h>
// #include <IMU.h>
#include <MechaQMC5883.h>
#include <Lidar.h>
#include <PID.h>
#include <Wire.h>
#include <Kicker.h>

//// ** CONFIG ** ////
#define USE_MULTICORE         // if defined, use second core for data update (NOTE: Overwritten by USE_OFFICIAL_PIXY_LIB)
#define USE_OFFICIAL_PIXY_LIB // if defined, use official Pixy2 library (NOTE: Overwrites USE_MULTICORE)

#define DEBUG_PRINT true

#define GOAL_YELLOW false // colour of goal to score on (YELLOW or BLUE)

#define pixyXC 142 // x-coordinate of center of Pixy2 camera
#define pixyYC 112 // y-coordinate of center of Pixy2 camera

#define IN_FRONT_FOV 7 // angle in front of robot to search for ball

#define SIGNATURE_BALL        1 // signature of ball
#define SIGNATURE_BLUE_GOAL   2 // signature of blue goal
#define SIGNATURE_YELLOW_GOAL 3 // signature of yellow goal

#define MAX_BALL_DIST_THRESHOLD 420 // max distance to ball before out of range
#define MIN_BALL_DIST_THRESHOLD 200 // min distance to ball
// #define BALL_FUNCTION_THRESHOLD 0.000323979729302f
#define BALL_FUNCTION_THRESHOLD 0.0004f // constant for distance to cm conversion

#define SPEED_TO_TURN_RATE_RATIO 1

#define SPEED     0.5 // speed of robot (0.0 to 1.0)

//// ** DECLARATIONS ** ////
// Pixy2 Camera
#ifdef USE_OFFICIAL_PIXY_LIB
#include <Pixy2UART.h> // include official Pixy2 library
Pixy2UART pixy;        // Pixy object from official Pixy2 library

volatile int numBlocks;        // total number of blocks deteced by Pixy2, volatile for multicore access
Block        blocks[50];       // array of all blocks detected by Pixy2
Block        ballBlocks[10];   // array of ball blocks
Block        yellowBlocks[10]; // array of yellow goal blocks
Block        blueBlocks[10];   // array of blue goal blocks
#else
#include <Camera.h> // include personal Pixy2 library
Camera Pixy(PIN_CAM_RX, PIN_CAM_TX_MISO, pixyXC, pixyYC); // Camera object from personal Pixy2 library
#endif

// Ball
int ballAngle;    // angle of ball relative to robot (0 to 360 degrees)
int ballDistance; // distance to ball (arbitrary units due to non-linear relationship caused by mirror distortion)
int emptyLightGateThreshold;

// Goals
float goalAngle;

// Bottom Plate
bool isOnLine; // true (1) if robot is on line (i.e. any bottom plate TEMT6000 exceeds threshold), false (0) if not
bool isBallInFront;
bool isBallCaptured;

size_t timeAtLastRise;
size_t timeBetweenSubsequentRises;
float lineAngle;

long long t;

// PID
PID pid(0.0075, 0, 0.125, 2000);

// Movement
Motor motorFR(PIN_BOT_B_LPWM, PIN_BOT_B_RPWM);
Motor motorBR(PIN_BOT_A_RPWM, PIN_BOT_A_LPWM);
Motor motorBL(PIN_TOP_B_RPWM, PIN_TOP_B_LPWM);
Motor motorFL(PIN_TOP_A_RPWM, PIN_TOP_A_LPWM);
Drive driveBase(motorFR, motorBR, motorBL, motorFL); // drive base controlling all 4 motors
float speed = SPEED;
float speedX, speedY, moveAngle;

// LiDAR
Lidar lidarFront(Wire, 0x12, -5); // front LiDAR
Lidar lidarRight(Wire, 0x13, +4); // right LiDAR
Lidar lidarBack(Wire, 0x11, +5);  // back LiDAR
Lidar lidarLeft(Wire, 0x10, +4);  // left LiDAR
#if defined(USE_MULTICORE) && !defined(USE_OFFICIAL_PIXY_LIB)
volatile float frontDist, backDist, rightDist, leftDist; // volatile for multicore access
#else
float frontDist, backDist, rightDist, leftDist; // distance to obstacles
#endif
int x, y; // coordinate of robot relative to field

// IMU
// IMU imu(Wire1, 0x1E); // IMU providing heading
MechaQMC5883 imu(Wire1, -244, -305, 1.05993520658, 56.2635641705);
#ifdef USE_MULTICORE
volatile float botHeading; //  heading of robot (0 to 360 degrees), volatile for multicore access
#else
float botHeading; // heading of robot (0 to 360 degrees)
#endif
float rotateCommand; // for compass correction (-180 to 180 degrees)
float goalRotateAngle;

// Kicker
// Kicker kicker(PIN_RELAY);

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
int moveTo(int targetX, int targetY, int tolerance) {
    float targetAngle = DEG(atan2(targetY - y, targetX - x)) + 90;
    float dist        = hypot(targetX - x, targetY - y); // distance to target coordinates

    if (dist > tolerance) { // not reached target, move
        return LIM_ANGLE(targetAngle);
    } else { // reached target, stop
        return -1;
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

    frontDist   = constrain(lidarFront.read() * cosf(RAD(angle)) + yCorrection, 0, 255);
    backDist    = constrain(lidarBack.read() * cosf(RAD(angle)) + yCorrection, 0, 255);
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
    const int borderDist = 25;//12; //set to 25 for testing on ri field
    const int borderTolerance = 11;
    deconstructSpeed();

    static float multiplierX;
    static float distanceX;
    int maxXL, maxXR, maxX;

    // consider right border
    if (x < (91 - 40) && y < (borderDist + 25) || y > (243 - (borderDist + 25))) { // bot to the left of goal
        maxXR = 91 - 40;
    } else {
        maxXR = 182 - borderDist - borderTolerance;
    }

    // consider left border
    if (x > (91 + 40) && y < (borderDist + 25) || y > (243 - (borderDist + 25))) { // bot to right of goal
        maxXL = 91 + 40;
    } else {
        maxXL = borderDist + borderTolerance;
    }

    DPRINT(maxXL);
    DPRINT(maxXR);

    const int angleRange = 10;
    if (moveAngle <= angleRange || moveAngle >= (360 - angleRange) || (moveAngle >= (180 - angleRange) && moveAngle <= (180 + angleRange))) {
        if (maxXR < maxXL) { // consider right border
            EPRINT("ri ");
            maxX = maxXR;
            distanceX = max(0, maxX - x);
        } else { // consider left border
            EPRINT("le ");
            maxX = maxXL;
            distanceX = max(0, x - maxX);
        }
    } else {
        if (moveAngle > 30 && moveAngle <= 180) { // moving right
            EPRINT("ri ");
            maxX = maxXR;
            distanceX = max(0, maxX - x);
        } else if (moveAngle > 180 && moveAngle < 330) {
            EPRINT("le ");
            maxX = maxXL;
            distanceX = max(0, x - maxX);
        }
    }
    multiplierX = constrain(1.7 * distanceX / 91, 0, 1); // TODO IMPORTANT INCREASE 1.7 ON ACTUAL FIELD 
    speedX = constrain(speedX, -SPEED * multiplierX, SPEED * multiplierX);

    static float multiplierY;
    static float distanceY;
    int maxY;
    if (moveAngle >= 90 && moveAngle < 270) { // moving down
        EPRINT("do ");
        maxY = max(borderDist, powf((3.0f / 80.0f * (x - 91.0f)), 4) + 243 - (23 + borderDist + borderTolerance)); // 38 is wall to goal border
        distanceY = max(0, maxY - y);
    } else if (moveAngle >= 270 || moveAngle < 90) { // moving up
        EPRINT("up");
        maxY = max(borderDist, - powf((3.0f / 80.0f * (x - 91.0f)), 4) + (13 + borderDist + borderTolerance));
        distanceY = max(0, y - maxY);
    }
    multiplierY = constrain(3 * distanceY/121.5, 0, 1);
    speedY = constrain(speedY, -SPEED * multiplierY, SPEED * multiplierY);

    constructSpeed();

    // DPRINT(maxX);
    // DPRINT(maxY);
    DPRINT(multiplierX);
    DPRINT(multiplierY); 
    // DPRINT(distanceX);
    // DPRINT(distanceY);
}

void confidence() {
    float sumX = leftDist + rightDist;
    float sumY = frontDist + backDist;

    float confX = constrain(powf(sumX / (182 + 2), 2), 0, 1);
    float confY = constrain(powf(sumY / (243 - 12), 2), 0, 1);

    deconstructSpeed();
    const int POWER = 10;
    const float MIN_SPEED = 0;
    float maxXSpeed = MIN_SPEED + (SPEED - MIN_SPEED) * pow(confX, POWER);
    float maxYSpeed = MIN_SPEED + (SPEED - MIN_SPEED) * pow(confY, POWER);
    speedX = constrain(speedX, -maxXSpeed, maxXSpeed);
    speedY = constrain(speedY, -maxYSpeed, maxYSpeed);
    constructSpeed();

    // DPRINT("mxs: "); DPRINT(maxXSpeed);
    // DPRINT("mys: "); DPRINT(maxYSpeed);
    // DPRINT(sumX);
    // DPRINT(sumY);
    // DPRINT(confX);
    // DPRINT(confY);
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
#ifdef USE_OFFICIAL_PIXY_LIB
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

// gets angle of ball relative to robot (0 to 360 degrees), returns -1 if no ball detected
int getBallAngle() {
    Block ballBlock = ballBlocks[0]; // uses first ball block ONLY (Potential improvement?)
    if (ballBlock.m_signature == SIGNATURE_BALL) {
        int xDiff = ballBlock.m_x - pixyXC;
        int yDiff = ballBlock.m_y - pixyYC;

        int angle = atan2(yDiff, xDiff) * 180 / PI + 90;
        return 360 - LIM_ANGLE(angle);
    } else {
        return -1; // return -1 if no ball detected
    }
}

float getGoalAngle(int numBlocks) {
    float angle;
    float maxSize = 0;
    for (int i = 0; i < numBlocks; i++) {
        Block block = (GOAL_YELLOW == true ? yellowBlocks[i] : blueBlocks[i]);
        if (block.m_signature == (GOAL_YELLOW == true ? SIGNATURE_YELLOW_GOAL : SIGNATURE_BLUE_GOAL)) {
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
    if ((GOAL_YELLOW == true ? numYellow : numBlue) > 0) {
        goalAngle = getGoalAngle((GOAL_YELLOW == true ? numYellow : numBlue));
    } else { // no goal detected
        // if no goal detected for 1 second, reset goal angle and distance
        // time is to prevent false reset (i.e. due to lag or blind spot)
        if (millis() - goalLastMillis > 1000) {
            goalAngle = -1;
            goalLastMillis = millis();
        }
    }
}
#else
void updateBallData() {
    Pixy.isNewDataPresent(); // checks if new data is present and parses it
    ballAngle    = Pixy.getBallAngle();
    ballDistance = Pixy.getBallDistance();
}
#endif

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
    if (ballAngle >= 0 && ballAngle < 45) { // front right
        angle = ballAngle + 90 * (1 - pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1));
    } else if (ballAngle >= 45 && ballAngle < 90) { // front right
        angle = ballAngle + 90 * (1 - pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1));
    } else if (ballAngle >= 90 && ballAngle < 180) {
        angle = ballAngle + 90 * (1 - pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 0.7));
    } else if (ballAngle >= 180 && ballAngle < 270) {
        angle = ballAngle - 90 * (1 - (pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 0.7)));
    } else if (ballAngle >= 270 && ballAngle < 315) { // front left
        angle = ballAngle - 90 * (1 - (pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1)));
    } else if (ballAngle >= 315 && ballAngle < 360) { // front left
        angle = ballAngle - 90 * (1 - (pow((float) max((ballDistance - MIN_BALL_DIST_THRESHOLD), 0) / MAX_BALL_DIST_THRESHOLD, 1)));
    }

    // DPRINT(ballDistInCm);
    // DPRINT("angle: ");
    // DPRINT(angle);

    return LIM_ANGLE(angle);
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
    Wire1.setSCL(PIN_WIRE1_SCL);
    Wire1.setSDA(PIN_WIRE1_SDA);
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

    if (digitalRead(PIN_ORI_RESET) == HIGH) { // reset IMU if button pressed
        imu.tare();
    }
}

//// ** MAIN ** ////
// core 0 setup
void setup() {
    // UART
    Serial.begin(9600);

    #ifndef USE_OFFICIAL_PIXY_LIB
    Pixy.begin(19200);
    #endif
    #if !defined(USE_MULTICORE) || defined(USE_OFFICIAL_PIXY_LIB)
    setupDevices();
    #endif

    // LED
    pinMode(PIN_LED, OUTPUT);
}

// core 1 setup
void setup1() {
    #if defined(USE_MULTICORE) && !defined(USE_OFFICIAL_PIXY_LIB)
    setupDevices();
    #endif
    #ifdef USE_OFFICIAL_PIXY_LIB
    pixy.init();
    #endif
}

// core 0 loop
void loop() {
    float dt = (micros() - t) / 1000;
    t = micros(); // loop time
    speed = SPEED;

    //// ** DATA UPDATE & PROCESSING ** ////
    #if !defined(USE_MULTICORE) || defined(USE_OFFICIAL_PIXY_LIB)
    updateData();
    #endif

    #ifdef USE_OFFICIAL_PIXY_LIB
    categoriseBlock();
    #else
    updateBallData();
    #endif

    readBallCap();

    bool isAiming = false;
    static unsigned long lastKickerMillis = millis();

    //// ** STRATEGY ** ////
    //rotateCommand = constrain((LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)/540, -1, 1); // from -180 to 180
    //rotateCommand = constrain(abs(rotateCommand), 0.03, 1) * copysign(1, rotateCommand);
    rotateCommand = constrain(pid.compute(0, -(LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)), -1, 1);
    if (ballAngle == -1) { // no ball detected, move to centre
        // moveAngle = moveTo(91, 122, 2);
        speed = 0; // stop
    } else { // ball detected
        if (isBallInFront || isBallCaptured) { // ball in front
            moveAngle = ballAngle;
            if (ballDistance > MIN_BALL_DIST_THRESHOLD + 30) {
                float speedConstrain = constrain(SPEED * constrain(powf(max(0, (float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / (float) (MAX_BALL_DIST_THRESHOLD - MIN_BALL_DIST_THRESHOLD)), 0.8f), 0, 1), max(0.2, SPEED / 3), SPEED);
                speed = constrain(speed, -speedConstrain, speedConstrain);
            } else {
                // speed = min(speed, 0.3); // move towards ball at slower speed
                speed = 0.5;
                moveAngle = goalAngle;
                // rotateCommand = constrain(-ANGLE_360_TO_180(goalAngle)/1080, -1, 1);
                // DPRINT(rotateCommand);
                // rotateCommand = constrain(log(abs(-ANGLE_360_TO_180(goalAngle)+1))*-1*copysign(1,goalAngle), -speed/10, speed/10);
            }

            // if (isBallCaptured) { // ball captured

            // } else {
                // speed = min(speed, 0.3); // move towards ball at slower speed
                // moveAngle = ballAngle;
            // }
        } else { // ball not in front, move towards it
            moveAngle = ballTrack();
            float ballDistanceSpeed = SPEED * 0.5 * (constrain(powf((ballAngle < 180.0f ? ballAngle : 360.0f - ballAngle) / 180.0f, 0.3) , 0, 1) + constrain(powf(max(0, (float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / (float) (MAX_BALL_DIST_THRESHOLD - MIN_BALL_DIST_THRESHOLD)), 0.8f), 0, 1));

            speed = constrain(speed, -ballDistanceSpeed, ballDistanceSpeed);
        }
    }

    // if (isAiming == true) {
    //     moveAngle = goalAngle;
    //     goalRotateAngle = (goalAngle < 180 ? goalAngle : goalAngle - 360);
    // } else {
    //     goalRotateAngle = 0;
    // }

    // if (ballAngle != -1) { // if ball present, rotate robot to face the goal
    //     goalRotateAngle = (goalAngle < 180 ? goalAngle : goalAngle - 360);
    // }

    // rotateCommand = constrain((LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)/540, -1, 1); // from -180 to 180
    // moveAngle = ballTrack(); // -1 if no ball detected
    // if (ballAngle != -1 && (ballAngle >= 350 || ballAngle <= 10)) {
        // moveAngle = 0;
    // }
    // moveAngle = ballAngle; // direct movement straight to ball
    // moveAngle = moveTo(91, 122, 2); // -1 if reached target

    //// ** LOCALISATION ** ////
    if (moveAngle == -1) { // stop if moveAngle is -1
        speed = 0;
    }

    // Staying within bounds 
    stayWithinBounds();
    // Staying within bounds (failsafe) using TEMTs
    if (isOnLine == true) { // failsafe: if on line, move to the center
        moveAngle = moveTo(91, 122, 2);
        speed = 1;
    }

    // confidence();

    //// ** MOVEMENT ** ////
    // driveBase.setDrive(speed, moveAngle, rotateCommand); //Speed multiplied to accomodate for differences in speed with the wheels
    driveBase.setDrive(speed, moveAngle, rotateCommand, 0);

    //// ** DEBUG ** ////
    // DPRINT(isBallInFront);
    // DPRINT(isBallCaptured);
    DPRINT(moveAngle);
    // DPRINT(goalAngle);
    // DPRINT(isOnLine);
    DPRINT(ballAngle);

    DPRINT(goalAngle);
    // DPRINT(ballDistance);
    // DPRINT(botHeading);
    // DPRINT(frontDist);
    // DPRINT(backDist);
    // DPRINT(leftDist);
    // DPRINT(rightDist);
    DPRINT(x);
    DPRINT(y);

    // Loop time
    EPRINT((float)(micros()-t)/1000);
    if (DEBUG_PRINT) Serial.println();

    blinkLED();
}

// core 1 loop
void loop1() {
    #ifdef USE_OFFICIAL_PIXY_LIB
    pixy.ccc.getBlocks(); // get blocks from Pixy

    numBlocks = pixy.ccc.numBlocks;    // number of blocks detected
    memset(blocks, 0, sizeof(blocks)); // clear blocks array
    for (int i = 0; i < numBlocks; i++) {
        // copy blocks to array so that they can be accessed in main loop
        memcpy(&blocks[i], &pixy.ccc.blocks[i], sizeof(Block));
    }
    #else
    Pixy.readData();
    #endif

    #if defined(USE_MULTICORE) && !defined(USE_OFFICIAL_PIXY_LIB)
    updateData();
    #endif
}
