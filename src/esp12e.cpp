#include <Arduino.h>

// byte Pins[] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Serial.println("Hello World!");
    // delay(1000);
    unsigned long time =micros();
    // Serial.println(millis());
    // analog read - create array of anolog inputs

    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);

    Serial.println(micros()-time);
}
