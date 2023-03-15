#include <Arduino.h>
#include <Wire.h>

#include <Drive.h>

#define MAX_SPEED 0.5

Motor motorFR(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motorBR(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motorBL(3, 7, MAX_SPEED);  // bottom right JST, bottom left motor
Motor motorFL(11, 9, MAX_SPEED); // top right JST, top left motor


Drive driveBase(motorFR, motorBR, motorBL, motorFL);

float targetSpeed, rotationRate;

#define HMC388L_ADDRESS 0x1E

int mag[3];

float targetAngle;
float moveAngle;

void setup() {
    Serial.begin(9600);
    Wire.setSCL(5);
    Wire.setSDA(4);
    Wire.begin();

    Wire.beginTransmission(HMC388L_ADDRESS);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();

     //targetAngle = atan2(mag[0], mag[1]) / PI * 180;
}

void loop() {
    bool isOnLine = digitalRead(1);
    if (!isOnLine) {
        Wire.beginTransmission(HMC388L_ADDRESS);
    Wire.write(0x03); // select register 3, X MSB register
    Wire.endTransmission();

    Wire.requestFrom(HMC388L_ADDRESS, 6);
    if (Wire.available() >= 6) {
        int buff[6];
        for (unsigned int i = 0; i < 6; i++) {
            buff[i] = Wire.read();
        }
        mag[0] = -1 * (int16_t) (((((uint16_t) buff[4]) << 8) | buff[5])); // X axis (internal sensor -y axis)
        mag[1] = -1 * (int16_t) (((((uint16_t) buff[0]) << 8) | buff[1])); // Y axis (internal sensor -x axis)
        mag[2] = -1 * (int16_t) (((((uint16_t) buff[2]) << 8) | buff[3])); // Z axis (internal sensor -z axis)
    }
    float a = atan2(mag[0], mag[1]) / PI * 180;
    if (targetAngle == 0) {
        targetAngle = a;
    }
    targetSpeed = 0.3;// rotationRate = 0;
    //moveAngle = 
    driveBase.setDrive(targetSpeed, 0, constrain((a-targetAngle)/90,-1,1));
    } else {
        driveBase.setDrive(1, 180, 0);
    }
    }
