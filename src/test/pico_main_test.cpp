#include <Arduino.h>
#include <Drive.h>
#include <IMU.h>
#include <Lidar.h>
#include <PID.h>
#include <Wire.h>

#define USE_MULTICORE // if defined, use second core for data update (NOTE: Overwritten by USE_OFFICIAL_PIXY_LIB)
#define USE_OFFICIAL_PIXY_LIB

#define pixyXC 139
#define pixyYC 104

#ifndef USE_OFFICIAL_PIXY_LIB
#include <Camera.h>
Camera Pixy(PIXY_RX, PIXY_TX, pixyXC, pixyYC);
#else
#include <Pixy2UART.h>
Pixy2UART pixy;
#endif

#define MAX_BALL_DIST_THRESHOLD 380
#define MIN_BALL_DIST_THRESHOLD 160
// #define BALL_FUNCTION_THRESHOLD 0.000323979729302f
#define BALL_FUNCTION_THRESHOLD 0.0004f

bool isOnLine;

int ballAngle;
int ballDistance;

#define MAX_SPEED 0.5
#define SPEED 0.3

PID pid(0.5, 0, 30, 1000);

Motor motorFR(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motorBR(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motorBL(3, 7, MAX_SPEED);   // bottom right JST, bottom left motor
Motor motorFL(11, 9, MAX_SPEED);  // top right JST, top left motor

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

#if defined(USE_MULTICORE) && !defined(USE_OFFICIAL_PIXY_LIB)
volatile float frontDist, backDist, rightDist, leftDist;
#else
float frontDist, backDist, rightDist, leftDist;
#endif

int x, y;
Lidar lidarFront(0x12, -5);
Lidar lidarRight(0x13, +4);
Lidar lidarBack(0x11, +5);
Lidar lidarLeft(0x10, +4);

float targetSpeed, rotationRate;

#ifdef DEBUG_CORE
volatile float botHeading;
#else
float botHeading;
#endif

float rotateAngle;
IMU imu(0x1E);

void blinkLED(int interval = 50) {
    static unsigned long lastMillis = 0;
    static bool ledState = false;
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
volatile int numBlocks;
Block blocks[50];

Block ballBlocks[10];
Block yellowBlocks[10];
Block blueBlocks[10];

int getBallDistance() {
    Block ballBlock = ballBlocks[0];
    if (ballBlock.m_signature == 1) {
        int xDiff = ballBlock.m_x - pixyXC;
        int yDiff = ballBlock.m_y - pixyYC;

        // max distance = 5 * 84 = 420 since max change is around 84 pixels
        return constrain(5 * hypot(xDiff, yDiff), 0, 3000);
    } else {
        return -1;
    }
}

int getBallAngle() {
    Block ballBlock = ballBlocks[0];
    if (ballBlock.m_signature == 1) {
        int xDiff = ballBlock.m_x - pixyXC;
        int yDiff = ballBlock.m_y - pixyYC;

        int angle = atan2(yDiff, xDiff) * 180 / PI + 90;
        if (angle < 0) angle += 360; // make sure angle is positive
        angle = 360 - angle; // invert angle

        return angle;
    } else {
        return -1;
    }
}

void categoriseBlock() {
    int numBall = 0, numYellow = 0, numBlue = 0;
    for (int i = 0; i < numBlocks; i++) {
        Block block = blocks[i];
        if (block.m_signature == 1) {
            ballBlocks[numBall] = block;
            numBall++;
        } else if (block.m_signature == 2) {
            yellowBlocks[numYellow] = block;
            numYellow++;
        } else if (block.m_signature == 3) {
            blueBlocks[numBlue] = block;
            numBlue++;
        }
    }


    static unsigned long ballLastMillis = millis();
    if (numBall > 0) {
        ballAngle = getBallAngle();
        ballDistance = getBallDistance();
    } else {
        if (millis() - ballLastMillis > 1000) {
            ballAngle = -1;
            ballDistance = -1;
        }
    }

    // for (int i = 0; i < numBlocks; i++) {
    //     Block block = blocks[i];
    //     Serial.print(" block ");
    //     Serial.print(i);
    //     Serial.print(": ");

    //     Serial.print("sig: ");
    //     Serial.print(block.m_signature);
    //     Serial.print(" x: ");
    //     Serial.print(block.m_x);
    //     Serial.print(" y: ");
    //     Serial.print(block.m_y);
    //     Serial.print(" width: ");
    //     Serial.print(block.m_width);
    //     Serial.print(" height: ");
    //     Serial.print(block.m_height);
    // }
}
#else
void updateBallData() {
    Pixy.isNewDataPresent(); // checks if new data is present and parses it
    ballAngle = Pixy.getBallAngle();
    ballDistance = Pixy.getBallDistance();
}
#endif

int ballTrack() {
    // Move perpendicular to ball if near, move straight if far

    if (ballAngle == -1) return -1; // no ball detected

    float ballDistInCm = BALL_FUNCTION_THRESHOLD * ballDistance * ballDistance;

    float moveAngle = 0;

    if (ballAngle < 10 || ballAngle > 350) { // ball is in front (TODO: ball cap light gate integration?)
        moveAngle = 0; // move straight
    } else if (ballAngle >= 0 && ballAngle < 180) {
        moveAngle = ballAngle + 90 * (1 - pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8));
    } else if (ballAngle >= 180 && ballAngle < 360) {
        moveAngle = ballAngle - 90 * (1 - (pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8)));
    }
    if (moveAngle < 0) moveAngle += 360;

    // Serial.print(ballDistInCm); Serial.print("\t");
    Serial.print("moveAng: "); Serial.print(moveAngle); Serial.print("\t");

    return moveAngle;
}

int moveTo(int targetX, int targetY, int tolerance) {
    float targetAngle = DEG(atan2(targetY - y, targetX - x)) + 90;
    targetAngle = targetAngle < 0 ? targetAngle + 360 : targetAngle;
    float dist = hypot(targetX - x, targetY - y);

    if (dist > tolerance) { // not reached target, move
        return targetAngle;
    } else { // reached target, stop
        return -1;
    }
}

void updatePosition() {
    float angle = botHeading <= 180 ? botHeading : 360 - botHeading;
    frontDist = lidarFront.read() * cosf(RAD(angle));
    backDist = lidarBack.read() * cosf(RAD(angle));
    leftDist = lidarLeft.read() * cosf(RAD(angle));
    rightDist = lidarRight.read() * cosf(RAD(angle));

    x = (leftDist + 182 - rightDist)/2;
    y = (frontDist + 243 - backDist)/2;
}

void setupDevices() {
    // I2C for LiDAR
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.setTimeout(1); // set timeout to 1 ms
    Wire.begin();

    // I2C for IMU
    imu.setCalibration(159, 32, 516, 530, -53);
    imu.init();
    imu.tare();

    // Pin for bottom plate
    pinMode(BOTTOM_PLATE_PIN, INPUT);
}

void updateData() {
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 1000/250) { // 250 Hz
        updatePosition();
        lastMillis = millis();
    }
    botHeading = imu.readAngle();
    rotateAngle = botHeading <= 180 ? botHeading : botHeading - 360; // from -180 to 180
    isOnLine = digitalRead(BOTTOM_PLATE_PIN);
}

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

void setup1() {
    #if defined(USE_MULTICORE) && !defined(USE_OFFICIAL_PIXY_LIB)
    setupDevices();
    #endif
    #ifdef USE_OFFICIAL_PIXY_LIB
    pixy.init();
    #endif
}

void loop() {
    unsigned long long now = micros(); // loop time

    #if !defined(USE_MULTICORE) || defined(USE_OFFICIAL_PIXY_LIB)
    updateData();
    #endif

    #ifdef USE_OFFICIAL_PIXY_LIB
    categoriseBlock();
    #else
    updateBallData();
    #endif

    int moveAngle = ballTrack(); // -1 if no ball detected
    // int moveAngle = ballAngle; // direct movement straight to ball 
    // int moveAngle = moveTo(91, 122, 2); // -1 if reached target

    // Serial.print(isOnLine); Serial.print("\t");
    Serial.print(ballAngle); Serial.print("\t");
    Serial.print(ballDistance); Serial.print("\t");
    // Serial.print(botHeading); Serial.print("\t");
    // Serial.print(frontDist); Serial.print("\t");
    // Serial.print(backDist); Serial.print("\t");
    // Serial.print(leftDist); Serial.print("\t");
    // Serial.print(rightDist); Serial.print("\t");
    // Serial.print(x); Serial.print("\t");
    // Serial.print(y); Serial.print("\t");

    // Serial.print(numBlocks); Serial.print("\t");

    // for (int i = 0; i < numBlocks; i++) {
    //     Block block = blocks[i];
    //     Serial.print(" block ");
    //     Serial.print(i);
    //     Serial.print(": ");

    //     Serial.print("sig: ");
    //     Serial.print(block.m_signature);
    //     Serial.print(" x: ");
    //     Serial.print(block.m_x);
    //     Serial.print(" y: ");
    //     Serial.print(block.m_y);
    //     Serial.print(" width: ");
    //     Serial.print(block.m_width);
    //     Serial.print(" height: ");
    //     Serial.print(block.m_height);
    // }

    // Staying within bounds
    if (isOnLine == true) { // failsafe: if on line, move to the center
        moveAngle = moveTo(91, 122, 2);
    }

    // Movement
    if (moveAngle == -1) { // stop if moveAngle is -1
        driveBase.setDrive(0, 0, constrain(rotateAngle/360, -1, 1));
    } else {
        driveBase.setDrive(SPEED, moveAngle, constrain(rotateAngle/360, -1, 1));
    }

    // Loop time
    Serial.print((float)(micros()-now)/1000); Serial.print("\t");
    Serial.println();

    // blinkLED();
}   

void loop1() {
    #ifdef USE_OFFICIAL_PIXY_LIB
    pixy.ccc.getBlocks();

    numBlocks = pixy.ccc.numBlocks;
    memset(blocks, 0, sizeof(blocks));
    for (int i = 0; i < numBlocks; i++) {
        memcpy(&blocks[i], &pixy.ccc.blocks[i], sizeof(Block));
    }
    #else
    Pixy.readData();
    #endif

    #if defined(USE_MULTICORE) && !defined(USE_OFFICIAL_PIXY_LIB)
    updateData();
    #endif
}
