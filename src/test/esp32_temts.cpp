#include "Arduino.h"

void setup() {
  Serial.begin(9600);
}

void loop() {
    int reading;
    reading = analogRead(39);
    Serial.println(reading);
}
