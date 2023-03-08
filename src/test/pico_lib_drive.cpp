#include <Arduino.h>
#include <Drive.h>

#define MAX_SPEED 1.0

Motor motorFR(0, 0, MAX_SPEED);
Motor motorBR(0, 0, MAX_SPEED);
Motor motorBL(0, 0, MAX_SPEED);
Motor motorFL(0, 0, MAX_SPEED);

Drive driveBase(motorFR, motorBR, motorBL, motorFL); // wheel pointing to 0 degrees

int targetAngle, targetSpeed, rotationRate;

void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    // for debugging:
    // if (velocity.direction > 360) {
    //     velocity.direction = 0;
    // } else {
    //     velocity.direction++;
    //     velocity.direction++;
    //     velocity.direction++;
    // }

    targetAngle = 0;
    targetSpeed = 1;
    rotationRate = -1;

    driveBase.setDrive(targetSpeed, targetAngle, rotationRate);
}
