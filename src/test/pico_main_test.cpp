#include <Arduino.h>
#include <CommonUtils.h>
#include <Definitions.h>
#include <Drive.h>
#include <IMU.h>
#include <Lidar.h>
#include <PID.h>
#include <Wire.h>
#include <Kicker.h>

//// ** CONFIG ** ////
#define USE_MULTICORE         // if defined, use second core for data update (NOTE: Overwritten by USE_OFFICIAL_PIXY_LIB)
#define USE_OFFICIAL_PIXY_LIB // if defined, use official Pixy2 library (NOTE: Overwrites USE_MULTICORE)

#define GOAL_YELLOW false // colour of goal to score on (YELLOW or BLUE)

#define pixyXC 139 // x-coordinate of center of Pixy2 camera
#define pixyYC 104 // y-coordinate of center of Pixy2 camera

#define SIGNATURE_BALL        1 // signature of ball
#define SIGNATURE_BLUE_GOAL   2 // signature of blue goal
#define SIGNATURE_YELLOW_GOAL 3 // signature of yellow goal

#define MAX_BALL_DIST_THRESHOLD 380 // max distance to ball before out of range
#define MIN_BALL_DIST_THRESHOLD 160 // min distance to ball
// #define BALL_FUNCTION_THRESHOLD 0.000323979729302f
#define BALL_FUNCTION_THRESHOLD 0.0004f // constant for distance to cm conversion

#define MAX_SPEED 0.5 // max speed of robot (0.0 to 1.0), failsafe and constrains motor drivers directly
#define SPEED     0.3 // speed of robot (0.0 to 1.0)

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
Camera Pixy(PIXY_RX, PIXY_TX, pixyXC, pixyYC); // Camera object from personal Pixy2 library
#endif

// Ball
int ballAngle;    // angle of ball relative to robot (0 to 360 degrees)
int ballDistance; // distance to ball (arbitrary units due to non-linear relationship caused by mirror distortion)
int emptyLightGateThreshold;

// Goals
float goalAngle;

// Bottom Plate
bool isOnLine; // true (1) if robot is on line (i.e. any bottom plate TEMT6000 exceeds threshold), false (0) if not

// PID
PID pid(0.5, 0, 30, 1000);

// Movement
Motor motorFR(21, 20);                    // top left JST, top right motor
Motor motorBR(26, 22);                    // bottom left JST, bottom right motor
Motor motorBL(3, 7);                      // bottom right JST, bottom left motor
Motor motorFL(11, 9);                     // top right JST, top left motor
Drive driveBase(motorFR, motorBR, motorBL, motorFL); // drive base controlling all 4 motors
float speed = SPEED;
float speedX, speedY, moveAngle;

// LiDAR
Lidar lidarFront(0x12, -5); // front LiDAR
Lidar lidarRight(0x13, +4); // right LiDAR
Lidar lidarBack(0x11, +5);  // back LiDAR
Lidar lidarLeft(0x10, +4);  // left LiDAR
#if defined(USE_MULTICORE) && !defined(USE_OFFICIAL_PIXY_LIB)
volatile float frontDist, backDist, rightDist, leftDist; // volatile for multicore access
#else
float frontDist, backDist, rightDist, leftDist; // distance to obstacles
#endif
int x, y; // coordinate of robot relative to field

// IMU
IMU imu(0x1E); // IMU providing heading
#ifdef USE_MULTICORE
volatile float botHeading; //  heading of robot (0 to 360 degrees), volatile for multicore access
#else
float botHeading; // heading of robot (0 to 360 degrees)
#endif
float rotateAngle; // for compass correction (-180 to 180 degrees)
float goalRotateAngle;

// Kicker
Kicker kicker(2);

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

void deconstructSpeed() {
    speedX = speed * sin(RAD(moveAngle));
    speedY = speed * cos(RAD(moveAngle));
}

void constructSpeed() {
    speed = sqrt(speedX * speedX + speedY * speedY);
    moveAngle = LIM_ANGLE(DEG(atan2(speedX, speedY)));
}

// void stayWithinBounds() {
//     const int borderDist = 12;
//     const int borderTolerance = 3;
//     if (x > (0 + (borderDist + borderTolerance))) {
//     }
// }

int confidence() {
    float sumX = leftDist + rightDist;
    float sumY = frontDist + backDist;

    float confX = constrain(sumX/(182+4), 0, 1);
    float confY = constrain(sumY/(243-12), 0, 1);

    deconstructSpeed();
    speedX = speedX * pow(confX, 1);
    speedY = speedY * pow(confY, 1);
    constructSpeed();

    return 0;
}

// true (1) if ball is captured, false (0) if not
bool isBallCaptured() {
    if (analogRead(LIGHT_GATE_PIN) < emptyLightGateThreshold - LIGHT_GATE_DIFFERENCE_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}


int calibrateLightGate() {
    delay(500);
    int sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(LIGHT_GATE_PIN);
    }
    return sum / 100;
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
    if (numBall > 0) {
        ballAngle    = getBallAngle();
        ballDistance = getBallDistance();
    } else { // no ball detected
        // if no ball detected for 1 second, reset ball angle and distance
        // time is to prevent false reset (i.e. due to lag or blind spot)
        if (millis() - ballLastMillis > 2000) {
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

    float ballDistInCm = BALL_FUNCTION_THRESHOLD * ballDistance * ballDistance; // converts ball distance to cm

    float angle = 0;

    if (ballAngle < 13 || ballAngle > 347) { // ball is in front (TODO: ball cap light gate integration?)
        if (ballDistance < 200) { // ball is near
            return -1; 
        }
        angle = 0; // if far, move forward
    } else if (ballAngle >= 0 && ballAngle < 90) { // front right
        angle = ballAngle + 90 * (1 - pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 1));
    } else if (ballAngle >= 90 && ballAngle < 180) {
        angle = ballAngle + 90 * (1 - pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8));
    } else if (ballAngle >= 180 && ballAngle < 270) {
        angle = ballAngle - 90 * (1 - (pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8)));
    } else if (ballAngle >= 270 && ballAngle < 360) { // front left
        angle = ballAngle - 90 * (1 - (pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 1)));
    }

    // Serial.print(ballDistInCm); Serial.print("\t");
    // Serial.print("angle: ");
    // Serial.print(angle); Serial.print("\t");

    return LIM_ANGLE(angle);
}

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
    frontDist   = lidarFront.read() * cosf(RAD(angle));
    backDist    = lidarBack.read() * cosf(RAD(angle));
    leftDist    = lidarLeft.read() * cosf(RAD(angle));
    rightDist   = lidarRight.read() * cosf(RAD(angle));

    x = (leftDist + 182 - rightDist) / 2;
    y = (frontDist + 243 - backDist) / 2;
}

// setup devices (LiDARs, IMU, bottom plate) - called in either core 0 or 1
void setupDevices() {
    // I2C for LiDAR
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.setTimeout(1); // set timeout to 1 ms
    Wire.begin();

    // I2C for IMU
    imu.setCalibration(159, 32, 516, 530, -53);
    imu.init();
    imu.tare(); // set current angle as heading 0

    // Pin for bottom plate
    pinMode(BOTTOM_PLATE_PIN, INPUT);
    pinMode(LIGHT_GATE_PIN, INPUT);
    emptyLightGateThreshold = calibrateLightGate();
}

// update data (LiDARs, IMU, bottom plate) - called in either core 0 or 1
void updateData() {
    // Update LiDARs not as frequently, using millis() to prevent lag
    static unsigned long lastMillis = 0;       // last time LiDARs were updated
    if (millis() - lastMillis >= 1000 / 250) { // 250 Hz
        updatePosition();
        lastMillis = millis();
    }
    botHeading  = imu.readAngle();                                   // from 0 to 360
    isOnLine    = digitalRead(BOTTOM_PLATE_PIN);                     // 1 if on line, 0 if not on line
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
    unsigned long long now = micros(); // loop time
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

    bool isAiming = false;
    static unsigned long lastKickerMillis = millis();

    //// ** STRATEGY ** ////
    if (isBallCaptured()) {
        isAiming = true;
        if (millis() - lastKickerMillis > 500) {
            if (goalAngle < 10 || goalAngle > 350) {
                kicker.kick();
                lastKickerMillis = millis();
            }
        }
    } else {
        if (ballAngle == -1) { // no ball detected
            moveAngle = moveTo(91, 122, 2); // move to center
        } else { // ball detected
            moveAngle = ballTrack(); // move perpendicular to ball
            if (moveAngle == -1) { // if ball is in front of robot
                isAiming = true;
            }
        }
    }

    // if (isAiming == true) {
    //     moveAngle = goalAngle;
    //     goalRotateAngle = (goalAngle < 180 ? goalAngle : goalAngle - 360);
    // } else {
    //     goalRotateAngle = 0;
    // }

    if (ballAngle != -1) { // if ball present, rotate robot to face the goal
        goalRotateAngle = (goalAngle < 180 ? goalAngle : goalAngle - 360);
    }

    // moveAngle = ballTrack(); // -1 if no ball detected
    // moveAngle = ballAngle; // direct movement straight to ball
    // moveAngle = moveTo(91, 122, 2); // -1 if reached target

    Serial.print(moveAngle); Serial.print("\t");
    Serial.print(ballAngle); Serial.print("\t");
    
    if (moveAngle == -1) { // stop if moveAngle is -1
        speed = 0;
    }
    
    confidence();

    // Staying within bounds
    if (isOnLine == true) { // failsafe: if on line, move to the center
        moveAngle = moveTo(91, 122, 2);
    }

    rotateAngle = LIM_ANGLE(botHeading - goalRotateAngle) <= 180 ? LIM_ANGLE(botHeading - goalRotateAngle) : LIM_ANGLE(botHeading -  goalRotateAngle) - 360; // from -180 to 180

    //// ** MOVEMENT ** ////
    driveBase.setDrive(speed, moveAngle, constrain(rotateAngle / 540, -1, 1));


    //// ** DEBUG ** ////
    // Serial.print(goalAngle); Serial.print("\t");
    // Serial.print(isOnLine); Serial.print("\t");
    // Serial.print(ballAngle); Serial.print("\t");
    // Serial.print(ballDistance); Serial.print("\t");
    // Serial.print(botHeading); Serial.print("\t");
    // Serial.print(frontDist); Serial.print("\t");
    // Serial.print(backDist); Serial.print("\t");
    // Serial.print(leftDist); Serial.print("\t");
    // Serial.print(rightDist); Serial.print("\t");
    // Serial.print(x); Serial.print("\t");
    // Serial.print(y); Serial.print("\t");

    // Loop time
    Serial.print((float)(micros()-now)/1000); Serial.print("\t");
    Serial.println();

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
