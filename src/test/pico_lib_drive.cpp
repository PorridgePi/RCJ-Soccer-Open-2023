#include <Arduino.h>
#include <Drive.h>
#include <Definitions.h>

#define MAX_SPEED 0.5

Motor motorFR(PIN_BOT_B_LPWM, PIN_BOT_B_RPWM);
Motor motorBR(PIN_BOT_A_LPWM, PIN_BOT_A_RPWM);
Motor motorBL(PIN_TOP_B_LPWM, PIN_TOP_B_RPWM);
Motor motorFL(PIN_TOP_A_LPWM, PIN_TOP_A_RPWM);
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
