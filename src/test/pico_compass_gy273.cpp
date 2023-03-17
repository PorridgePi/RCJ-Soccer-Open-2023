#include <Arduino.h>
#include <Wire.h>
#include <IMU.h>
#include <Drive.h>
#include <Lidar.h>

#define MAX_SPEED 0.5

Motor motorFR(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motorBR(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motorBL(3, 7, MAX_SPEED);  // bottom right JST, bottom left motor
Motor motorFL(11, 9, MAX_SPEED); // top right JST, top left motor

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

int   frontDist, backDist, rightDist, leftDist;
Lidar lidarFront(0x12, -5);
Lidar lidarRight(0x13, +4);
Lidar lidarBack(0x11, +5);
Lidar lidarLeft(0x10, +4);

unsigned long timestamp;

int x,y;

float targetSpeed, rotationRate;

IMU imu(0x1E);

float targetAngle;
float moveAngle;
/*
void updatePosition() {
    // field width = 182cm, field length = 243cm
    //Serial.println(lidarBack.read());
    x = (lidarLeft.read() - lidarRight.read());
    y = (lidarFront.read() - lidarBack.read());
}
*/
void setup() {
    Serial.begin(9600);
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.begin();

    imu.setCalibration(159,32,516,530,-53);

    imu.init();
    
    imu.tare();
     //targetAngle = atan2(mag[0], mag[1]) / PI * 180;
}

void loop() {
    //bool isOnLine = digitalRead(1);
    driveBase.setDrive(0.3, 0, constrain(-imu.readAngle()/45,-0.3,0.3));
    //updatePosition();
    /*
    if (!isOnLine) {
    targetSpeed = 0.3;// rotationRate = 0;
    //moveAngle = 
    driveBase.setDrive(targetSpeed, 0, constrain(-(imu.readAngle())/90,-1,1));
    } else {
        driveBase.setDrive(0.5, 180, 0);
    }
    */
    }
