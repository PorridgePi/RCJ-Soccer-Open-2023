#include "IMU.h"

#define BUTTON_PIN 19
volatile unsigned long pulseInTimeBegin          = micros();
volatile unsigned long pulseInTimeEnd            = micros();
volatile bool          newPulseDurationAvailable = false;

void buttonPinInterrupt() {
    if (digitalRead(BUTTON_PIN) == HIGH) {
        // start measuring
        pulseInTimeBegin = micros();
    } else {
        // stop measuring
        pulseInTimeEnd            = micros();
        newPulseDurationAvailable = true;
    }
}

IMU::IMU(int pin) : _pin(pin) {
}

void IMU::setup() {
    pinMode(BUTTON_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),
                    buttonPinInterrupt,
                    CHANGE);

    for (int i = 0; i < 10; i++) {
        read();
    }
    //  offset = 0;
}

float IMU::read() {
    if (newPulseDurationAvailable) {
        newPulseDurationAvailable   = false;
        unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;
        rot                         = (pulseDuration - 700) / 3 - offset;
    }

    if (rot < 0) {
        rot += 360;
    }

    return constrain(rot, 0, 360);
}
