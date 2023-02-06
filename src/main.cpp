#include "Arduino.h"
#include "Motor.h"
Motor A(45,1,12,13);
void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  Posori.X ++;
  Posori.Orientation ++;
  Serial.println(Posori.X);
  Serial.println(A.drive(30));
  digitalWrite(PIN_LED, HIGH);
  delay(100);
  digitalWrite(PIN_LED, LOW);
  delay(100);
}
