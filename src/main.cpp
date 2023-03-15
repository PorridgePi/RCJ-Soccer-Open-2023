#include <Arduino.h>
#include <Definitions.h>
#include <Wire.h>
#include <Camera.h>
#include <Lidar.h>

#define DEBUG true
#define DEBUG_PRINT_LIGHT_GATE false
#define DEBUG_PRINT_PIXY false
#define DEBUG_LOOP_TIME false
#define DEBUG_ON_LINE false

#define MAX_BALL_DIST_THRESHOLD 420
#define MIN_BALL_DIST_THRESHOLD 150
// #define BALL_FUNCTION_THRESHOLD 0.000323979729302f
#define BALL_FUNCTION_THRESHOLD 0.0004f

Camera Pixy(PIXY_RX, PIXY_TX, 139, 104);

int   frontDist, backDist, rightDist, leftDist;
Lidar lidarFront(0x12, -5);
Lidar lidarRight(0x13, +4);
Lidar lidarBack(0x11, +5);
Lidar lidarLeft(0x10, +4);

unsigned long loopStartMicros;

int emptyLightGateThreshold;
int ballAngle;
int ballDistance;

int calibrateLightGate() {
    delay(500);
    int sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(LIGHT_GATE_PIN);
    }
    return sum / 100;
}

bool isBallInGate() {
    if (analogRead(LIGHT_GATE_PIN) < emptyLightGateThreshold - LIGHT_GATE_DIFFERENCE_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}

bool isOnLine() {
    return digitalRead(BOTTOM_PLATE_PIN);
}

float moveAngle = 0;

void ballTrack() {
    // Move perpendicular to ball if near, move straight if far

    if (ballAngle == -1) return; // no ball detected

    float ballDistInCm = BALL_FUNCTION_THRESHOLD * ballDistance * ballDistance;

    if (ballAngle >= 0 && ballAngle < 180) {
        moveAngle = ballAngle + 90 * (1 - pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8));
    } else if (ballAngle >= 180 && ballAngle < 360) {
        moveAngle = ballAngle - 90 * (1 - (pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8)));
    }

    if (moveAngle < 0) moveAngle += 360;

    Serial.print(ballAngle); Serial.print("\t");
    Serial.print(ballDistance); Serial.print("\t");
    Serial.print(ballDistInCm); Serial.print("\t");
    Serial.print(moveAngle); Serial.print("\t");
    Serial.println();
}

void localisation() {
    int front, right, back, left, x, y;

    front = lidarFront.read();
    right = lidarRight.read();
    back = lidarBack.read();
    left = lidarLeft.read();

    // field width = 182cm, field length = 243cm
    x = (left + 182 - right) / 2;
    y = (front + 243 - back) / 2;

    Serial.print(x); Serial.print("\t");
    Serial.print(y); Serial.print("\t");
    Serial.print(front); Serial.print("\t");
    Serial.print(right); Serial.print("\t");
    Serial.print(back); Serial.print("\t");
    Serial.print(left); Serial.print("\t");
    Serial.println();
}

void setup() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); // turn on LED to indicate start of setup

    pinMode(BOTTOM_PLATE_PIN, INPUT);
    pinMode(LIGHT_GATE_PIN, INPUT);

    Serial.begin(9600);
    Pixy.begin(19200);

    // I2C for LiDAR
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.begin();

    emptyLightGateThreshold = calibrateLightGate();
    digitalWrite(PIN_LED, LOW); // turn off LED to indicate end of setup
}

void loop() {
    loopStartMicros = micros();

    if (!USE_MULTICORE) Pixy.readData();
    Pixy.isNewDataPresent(); // checks if new data is present and parses it
    ballAngle = Pixy.getBallAngle();
    ballDistance = Pixy.getBallDistance();

    ballTrack();
    localisation();

    if (DEBUG_ON_LINE) {
        Serial.print(isOnLine()); Serial.print("\t");
    }
    if (DEBUG_PRINT_LIGHT_GATE) {
        Serial.print(emptyLightGateThreshold); Serial.print("\t");
        Serial.print(isBallInGate()); Serial.print("\t");
        Serial.print(analogRead(LIGHT_GATE_PIN)); Serial.print("\t");
    }
    if (DEBUG_PRINT_PIXY) {
        Serial.print(ballDistance); Serial.print("\t");
        Serial.print(ballAngle); Serial.print("\t");
    } 
    if (DEBUG_LOOP_TIME) {
        Serial.print((float) (micros() - loopStartMicros) / 1000);
        Serial.print("\t");
    }
    if (DEBUG_PRINT_LIGHT_GATE || DEBUG_PRINT_PIXY || DEBUG_LOOP_TIME || DEBUG_ON_LINE) Serial.println();
}

void loop1() {
    if (USE_MULTICORE) Pixy.readData();
}
