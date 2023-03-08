#include <Arduino.h>

#define MOTOR_PIN0 22
#define MOTOR_PIN1 26

#define MAX_SPEED 0.5

void setup() {
    Serial.begin(115200);
    pinMode(MOTOR_PIN0, OUTPUT);
    pinMode(MOTOR_PIN1, OUTPUT);
}

int i = 0;
float speed = 0.1;

void loop() {
    speed = sin(i++ / 1000.0);
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
