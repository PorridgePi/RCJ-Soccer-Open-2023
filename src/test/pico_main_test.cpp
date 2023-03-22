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

int ballAngle;
int ballDistance;

#define MAX_SPEED 0.5

PID pid(0.5, 0, 30, 1000);

Motor motorFR(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motorBR(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motorBL(3, 7, MAX_SPEED);   // bottom right JST, bottom left motor
Motor motorFL(11, 9, MAX_SPEED);  // top right JST, top left motor

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

float   frontDist, backDist, rightDist, leftDist;
int x, y;
Lidar lidarFront(0x12, -5);
Lidar lidarRight(0x13, +4);
Lidar lidarBack(0x11, +5);
Lidar lidarLeft(0x10, +4);

float targetSpeed, rotationRate;

float botHeading;
IMU imu(0x1E);

void updateBallData() {
    Pixy.isNewDataPresent(); // checks if new data is present and parses it
    ballAngle = Pixy.getBallAngle();
    ballDistance = Pixy.getBallDistance();
}

void ballTrack() {
    // Move perpendicular to ball if near, move straight if far

    if (ballAngle == -1) return; // no ball detected

    float ballDistInCm = BALL_FUNCTION_THRESHOLD * ballDistance * ballDistance;

    /*
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
    
    */
}

void moveTo(int targetX, int targetY) {
    float moveAngle = DEG(atan2(targetY - y, targetX - x)) + 90;
    moveAngle = moveAngle < 0 ? moveAngle + 360 : moveAngle;
    float dist = hypot(targetX - x, targetY - y);

    Serial.print("Move Angle: ");
    Serial.print(moveAngle);
    Serial.print("\t");

    float rotateAngle = botHeading <= 180 ? botHeading : botHeading - 360; // from -180 to 180
    if (dist > 10) {
        driveBase.setDrive(0.2, moveAngle, constrain(rotateAngle/360, -1, 1));
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
    
    Serial.print("Front: ");
    Serial.print(frontDist); Serial.print("\t");

    Serial.print("Back: ");
    Serial.print(backDist); Serial.print("\t");
    
    Serial.print("Left: ");
    Serial.print(leftDist); Serial.print("\t");

    Serial.print("Right: ");
    Serial.print(rightDist); Serial.print("\t");

    Serial.print("Angle: ");
    Serial.print(angle); Serial.print("\t");

    x = (leftDist + 182 - rightDist)/2;
    y = (frontDist + 243 - backDist)/2;

    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
}

void setup() {
    Serial.begin(9600);
    Pixy.begin(19200);

    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.setTimeout(1); // set timeout to 1 ms
    Wire.begin();

    imu.setCalibration(159, 32, 516, 530, -53);
    imu.init();
    imu.tare();
    pinMode(PIN_LED, OUTPUT);
}

void loop() {
    unsigned long long now = micros();
    static unsigned long lastMicros = 0;
    static bool ledState = false;

    botHeading = imu.readAngle();
    // bool isOnLine = digitalRead(1);
    float rotateAngle = botHeading <= 180 ? botHeading : botHeading - 360; // from -180 to 180

    updateBallData();
    Serial.print(ballAngle);
    Serial.print("\t");
    Serial.println(ballDistance);
    
    // // Serial.print(rotateAngle);
    // Serial.print("\t");

    // For Yikun's Drive lib
    // driveBase.setDrive(0.5, floor((millis()%4000)/1000)*90 + rotateAngle, constrain(-rotateAngle/45, -1, 1));
    // driveBase.setDrive(0.2, floor((millis()%4000)/1000)*90-rotateAngle, constrain(pid.compute(0, -rotateAngle/180), -1, 1));
    
    // WITHOUT PID
    // driveBase.setDrive(0.2, floor((millis()%4000)/1000)*90 - rotateAngle, constrain(rotateAngle/360, -1, 1));
    // WITH PID
    // driveBase.setDrive(0.2, floor((millis()%4000)/1000)*90 - rotateAngle, constrain(pid.compute(0, -rotateAngle / 180), -1, 1));

    //updatePosition();
    //moveTo(91, 122);

    // Serial.print((float)(micros()-now)/1000);
    // Serial.println();
    if (micros() - lastMicros >= 50 * 1000) {
        lastMicros = micros();
        if (ledState) {
            digitalWrite(PIN_LED, LOW);
            ledState = false;
        } else {
            digitalWrite(PIN_LED, HIGH);
            ledState = true;
        }
    }
}   

void loop1() {
    Pixy.readData();
}
