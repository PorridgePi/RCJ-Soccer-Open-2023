#include <Arduino.h>
#include <Drive.h>

Drive A(270); // wheel pointing to 0 degrees

int targetAngle;
int targetSpeed;

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

    A.setDirection(targetAngle);
    A.setSpeed(targetSpeed);

    Serial.print("Target Angle: ");
    Serial.print(targetAngle);
    Serial.print("\tSpeed: ");
    Serial.print(targetSpeed);
    Serial.print("\tanalogWrite value / 255: ");
    Serial.println(A.drive());
}
