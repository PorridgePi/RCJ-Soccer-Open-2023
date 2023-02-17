#include <Arduino.h>

#define IR_THRESHOLD 30

int ambientLight = 0;

void setup() {
    Serial.begin(9600);
    // Get average of 100 readings for calibration
    int sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(A0);
        delay(10);
    }
    ambientLight = sum / 100;
    Serial.println(IR_THRESHOLD);
}

void loop() {
    int reading = analogRead(A0);
    Serial.println(reading);
    if (reading < ambientLight - IR_THRESHOLD) {
        digitalWrite(PIN_LED, HIGH);
    } else {
        digitalWrite(PIN_LED, LOW);
    }
    delay(10);
}
