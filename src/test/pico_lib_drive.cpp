#include <Arduino.h>
#include <Drive.h>

Drive A(45, 1, 12, 13);

void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    // for debugging:
    if (velocity.direction > 360) {
        velocity.direction = 0;
    } else {
        velocity.direction++;
        velocity.direction++;
        velocity.direction++;
    }
    Serial.println(velocity.direction);
    Serial.println(velocity.magnitude);
    Serial.println(A.drive());
    digitalWrite(PIN_LED, HIGH);
    delay(10);
    digitalWrite(PIN_LED, LOW);
    delay(10);
}
