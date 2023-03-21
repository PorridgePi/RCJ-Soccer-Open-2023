#include <Arduino.h>
#include <Drive.h>
#include <IMU.h>
#include <Lidar.h>
#include <PID.h>
#include <Wire.h>

#define MAX_SPEED 0.5

PID pid(0.5, 0, 30, 1000);

Motor motorFR(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motorBR(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motorBL(3, 7, MAX_SPEED);   // bottom right JST, bottom left motor
Motor motorFL(11, 9, MAX_SPEED);  // top right JST, top left motor

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

int   frontDist, backDist, rightDist, leftDist;
int x, y;
Lidar lidarFront(0x12, -5);
Lidar lidarRight(0x13, +4);
Lidar lidarBack(0x11, +5);
Lidar lidarLeft(0x10, +4);

float targetSpeed, rotationRate;

float botHeading
IMU imu(0x1E);

void updatePosition() {
    /*
    Serial.print("Front: ");
    Serial.print(lidarFront.read());

    Serial.print("\tBack: ");
    Serial.print(lidarBack.read());

    Serial.print("\tRight: ");
    Serial.print(lidarRight.read());

    Serial.print("\tLeft: ");
    Serial.print(lidarLeft.read());
*/

    frontDist = lidarFront.read();
    backDist = lidarBack.read();
    rightDist = lidarRight.read();
    leftDist = lidarLeft.read();

    x = (lidarLeft.read() - lidarRight.read()) / 2;
    y = (lidarFront.read() - lidarBack.read()) / 2;
    Serial.print(x);
    Serial.print("\t");
    Serial.println(y);
}

void setup() {
    Serial.begin(9600);
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.setTimeout(1); // set timeout to 1 ms
    Wire.begin();

    imu.setCalibration(159, 32, 516, 530, -53);
    imu.init();
    imu.tare();
}

void loop() {
    unsigned long long now = micros();

    botHeading = imu.readAngle();
    // bool isOnLine = digitalRead(1);
    float rotateAngle = botHeading <= 180 ? botHeading : botHeading - 360; // from -180 to 180

    // Serial.print(rotateAngle);
    // Serial.print("\t");

    // For Yikun's Drive lib
    // driveBase.setDrive(0.5, floor((millis()%4000)/1000)*90 + rotateAngle, constrain(-rotateAngle/45, -1, 1));
    // driveBase.setDrive(0.2, floor((millis()%4000)/1000)*90-rotateAngle, constrain(pid.compute(0, -rotateAngle/180), -1, 1));
    
    // WITHOUT PID
    // driveBase.setDrive(0.2, floor((millis()%4000)/1000)*90 - rotateAngle, constrain(rotateAngle/360, -1, 1));
    // WITH PID
    driveBase.setDrive(0.2, floor((millis()%4000)/1000)*90 - rotateAngle, constrain(pid.compute(0, -rotateAngle / 180), -1, 1));

    updatePosition();
    
    /*
    if (!isOnLine) {
    targetSpeed = 0.3;// rotationRate = 0;
    driveBase.setDrive(targetSpeed, 0, constrain(-(imu.readAngle())/90,-1,1));
    } else {
        driveBase.setDrive(0.5, 180, 0);
    }
    */

    Serial.print((float)(micros()-now)/1000);
    Serial.println();
}   
