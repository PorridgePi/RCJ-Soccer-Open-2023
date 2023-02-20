#include <Motor.h>
#include <Arduino.h>

Motor A(45, 1, 12, 13);

void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    // for debugging:
    if (Velocity.direction > 360) {
        Velocity.direction = 0;
    } else {
        Velocity.direction++;
        Velocity.direction++;
        Velocity.direction++;
    }
    Serial.println(Velocity.direction);
    Serial.println(Velocity.magnitude);
    Serial.println(A.drive());
    digitalWrite(PIN_LED, HIGH);
    delay(10);
    digitalWrite(PIN_LED, LOW);
    delay(10);
}
