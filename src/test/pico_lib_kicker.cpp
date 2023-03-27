#include <Arduino.h>
#include <Kicker.h>

Kicker kicker(2);

void setup() {
    Serial.begin(9600);
}

void loop() {
    kicker.kick();
    delay(5000);
}
