#include <Arduino.h>
#include <Motor.h>

#define MAX_SPEED 0.5

Motor motor1(21, 20, MAX_SPEED); // top left JST, top right motor
Motor motor2(26, 22, MAX_SPEED); // bottom left JST, bottom right motor
Motor motor3(3, 7, MAX_SPEED);  // bottom right JST, bottom left motor
Motor motor4(11, 9, MAX_SPEED); // top right JST, top left motor

// Conflicting PWM pins:

// GPIO 6  PWM 3A * -> GPIO 3  PWM 1B
// GPIO 7  PWM 3B

// GPIO 10 PWM 5A * -> GPIO 9  PWM 4B
// GPIO 11 PWM 5B

// GPIO 20 PWM 2A
// GPIO 21 PWM 2B

// GPIO 22 PWM 3A *
// GPIO 26 PWM 5A *

void setup() {
    Serial.begin(115200);
}

void loop() {
    float speed = sin(millis() / 5000.0);
    Serial.print(speed);
    Serial.println();
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor3.setSpeed(speed);
    motor4.setSpeed(speed);
}
