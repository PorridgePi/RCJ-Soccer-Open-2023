#include <Arduino.h>
#include <Definitions.h>

#define DEBUG true

bool isBallInGate() {
    if (analogRead(LIGHT_GATE_PIN) < LIGHT_GATE_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    if (isBallInGate()) {
        digitalWrite(PIN_LED, HIGH);
    } else {
        digitalWrite(PIN_LED, LOW);
    }
}
