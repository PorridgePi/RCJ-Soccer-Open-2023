#include "Arduino.h"

void setup() {
  Serial.begin(115200);
}

// PINS FOR TEMTS
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15

void loop() {
    Serial.print(analogRead(33));
    Serial.print("\t");
    Serial.print(analogRead(32));
    Serial.print("\t");
    Serial.print(analogRead(4));
    Serial.print("\t");
    Serial.print(analogRead(2));
    Serial.print("\t");
    Serial.print(analogRead(15));
    Serial.print("\t");
    Serial.print(analogRead(13));
    Serial.print("\t");
    Serial.print(analogRead(14));
    Serial.print("\t");
    Serial.print(analogRead(27));
    Serial.print("\t");
    Serial.print(analogRead(26));
    Serial.print("\t");
    Serial.println(analogRead(25));
    delay(10);
}
