#include <Arduino.h>
#include <Drive.h>
#include <IMU.h>
#include <Lidar.h>
#include <PID.h>
#include <Wire.h>
#include <Camera.h>

#define MAX_BALL_DIST_THRESHOLD 420
#define MIN_BALL_DIST_THRESHOLD 150
// #define BALL_FUNCTION_THRESHOLD 0.000323979729302f
#define BALL_FUNCTION_THRESHOLD 0.0004f

Camera Pixy(PIXY_RX, PIXY_TX, 139, 104);

bool isOnLine;

int ballAngle;
int ballDistance;

#define MAX_SPEED 0.5

PID pid(0.5, 0, 30, 1000);

Motor motorFR(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motorBR(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motorBL(3, 7, MAX_SPEED);   // bottom right JST, bottom left motor
Motor motorFL(11, 9, MAX_SPEED);  // top right JST, top left motor

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

#ifdef USE_MULTICORE
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

void updateBallData() {
    Pixy.isNewDataPresent(); // checks if new data is present and parses it
    ballAngle = Pixy.getBallAngle();
    ballDistance = Pixy.getBallDistance();
}

void ballTrack() {
    // Move perpendicular to ball if near, move straight if far

    if (ballAngle == -1) return; // no ball detected

    float ballDistInCm = BALL_FUNCTION_THRESHOLD * ballDistance * ballDistance;

    float moveAngle = 0;

    if (ballAngle >= 0 && ballAngle < 180) {
        moveAngle = ballAngle + 90 * (1 - pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8));
    } else if (ballAngle >= 180 && ballAngle < 360) {
        moveAngle = ballAngle - 90 * (1 - (pow((float) (ballDistance - MIN_BALL_DIST_THRESHOLD) / MAX_BALL_DIST_THRESHOLD, 0.8)));
    }
    if (moveAngle < 0) moveAngle += 360;

    driveBase.setDrive(ballAngle != -1 ? 0.15 : 0, moveAngle, constrain(rotateAngle/360, -1, 1));

    Serial.print(ballDistInCm); Serial.print("\t");
    Serial.print(moveAngle); Serial.print("\t");
}

void moveTo(int targetX, int targetY, int tolerance) {
    float targetAngle = DEG(atan2(targetY - y, targetX - x)) + 90;
    targetAngle = targetAngle < 0 ? targetAngle + 360 : targetAngle;
    float dist = hypot(targetX - x, targetY - y);

    if (dist > tolerance) {
        driveBase.setDrive(0.2, targetAngle, constrain(rotateAngle/360, -1, 1));
    } else {
        driveBase.setDrive(0, 0, 0);
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
    // I2C for IMU
    imu.setCalibration(159, 32, 516, 530, -53);
    imu.init();
    imu.tare();

    // I2C for LiDAR
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.setTimeout(1); // set timeout to 1 ms
    Wire.begin();

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
    Pixy.begin(19200);

    #ifndef USE_MULTICORE
    setupDevices();
    #endif

    // LED
    pinMode(PIN_LED, OUTPUT);
}

void setup1() {
    #ifdef USE_MULTICORE
    setupDevices();
    #endif
}

void loop() {
    unsigned long long now = micros(); // loop time

    #ifndef USE_MULTICORE
    updateData();
    #endif

    // ballTrack();
    // moveTo(91, 122, 2);
    driveBase.setDrive(ballAngle != -1 ? 0.3 : 0, ballAngle, constrain(rotateAngle/360, -1, 1));

    // Serial.print(isOnLine); Serial.print("\t");
    // Serial.print(ballAngle); Serial.print("\t");
    // Serial.print(ballDistance); Serial.print("\t");
    Serial.print(botHeading); Serial.print("\t");
    // Serial.print(frontDist); Serial.print("\t");
    // Serial.print(backDist); Serial.print("\t");
    // Serial.print(leftDist); Serial.print("\t");
    // Serial.print(rightDist); Serial.print("\t");
    Serial.print(x); Serial.print("\t");
    Serial.print(y); Serial.print("\t");

    // Loop time
    Serial.print((float)(micros()-now)/1000); Serial.print("\t");
    Serial.println();

    blinkLED();
}   

void loop1() {
    Pixy.readData();
    #ifdef USE_MULTICORE
    updateData();
    #endif
}
