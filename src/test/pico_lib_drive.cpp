#include <Arduino.h>
#include <Drive.h>

Drive A(45);

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
    Serial.print("Target Orientation: ");
    Serial.print(velocity.direction);
    Serial.print("\tSpeed: ");
    Serial.print(velocity.speed);
    Serial.print("\tanalogWrite value / 255: ");
    Serial.println(A.drive());
}
