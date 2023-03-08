#include <Arduino.h>

#define MOTOR_PIN0 22
#define MOTOR_PIN1 26

#define MAX_SPEED 1.0

void setup() {
    Serial.begin(115200);
    pinMode(MOTOR_PIN0, OUTPUT);
    pinMode(MOTOR_PIN1, OUTPUT);
}

float speed = 0.1;

void loop() {
    speed = sin(millis() / 5000.0);
    Serial.print(speed);
    Serial.print("\t");
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    Serial.print(speed);
    Serial.println();

    if (speed >= 0) {
        analogWrite(MOTOR_PIN0, speed * 255);
        analogWrite(MOTOR_PIN1, 0);
    } else {
        analogWrite(MOTOR_PIN0, 0);
        analogWrite(MOTOR_PIN1, -speed * 255);
    }
}
