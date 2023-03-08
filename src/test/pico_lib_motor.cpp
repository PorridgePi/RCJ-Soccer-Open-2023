#include <Arduino.h>
#include <Motor.h>

#define MAX_SPEED 1.0

Motor motor(22, 26, MAX_SPEED);

void setup() {
    Serial.begin(115200);
}

void loop() {
    float speed = sin(millis() / 5000.0);
    Serial.print(speed);
    Serial.println();
    motor.setSpeed(speed);
}
