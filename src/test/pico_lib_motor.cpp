#include <Arduino.h>
#include <Motor.h>
#include <Definitions.h>

#define MAX_SPEED 0.5

Motor motorBR(PIN_BOT_A_LPWM, PIN_BOT_A_RPWM); // top left JST, top right motor
Motor motorFR(PIN_BOT_B_LPWM, PIN_BOT_B_RPWM); // bottom left JST, bottom right motor
Motor motorFL(PIN_TOP_A_LPWM, PIN_TOP_A_RPWM);   // bottom right JST, bottom left motor
Motor motorBL(PIN_TOP_B_RPWM, PIN_TOP_B_LPWM);  // top right JST, top left motor

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
    float speed = 0.3; // sin(millis() / 5000.0);
    Serial.print(speed);
    Serial.println();

    motorFR.setSpeed(speed);
    delay(1000);
    motorFR.setSpeed(0);
    motorBR.setSpeed(speed);
    delay(1000);
    motorBR.setSpeed(0);
    motorBL.setSpeed(speed);
    delay(1000);
    motorBL.setSpeed(0);
    motorFL.setSpeed(speed);
    delay(1000);
    motorFL.setSpeed(0);
    delay(3000);
}
