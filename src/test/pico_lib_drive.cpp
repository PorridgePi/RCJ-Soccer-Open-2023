#include <Arduino.h>
#include <Drive.h>

#define MAX_SPEED 0.5

Motor motorFR(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motorBR(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motorBL(3, 7, MAX_SPEED);   // bottom right JST, bottom left motor
Motor motorFL(11, 9, MAX_SPEED);  // top right JST, top left motor

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

float targetSpeed, rotationRate;

void setup() {
    Serial.begin(9600);
}

void loop() {
    targetSpeed  = 0.2;
    rotationRate = 0;
    driveBase.setDrive(targetSpeed, 0, rotationRate);
    delay(1000);
    driveBase.setDrive(targetSpeed, 90, rotationRate);
    delay(1000);
    driveBase.setDrive(targetSpeed, 180, rotationRate);
    delay(1000);
    driveBase.setDrive(targetSpeed, 270, rotationRate);
    delay(1000);
}
