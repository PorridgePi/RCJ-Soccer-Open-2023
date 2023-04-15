#include <Arduino.h>
#include <Kicker.h>
#include <Definitions.h>

Kicker kicker(PIN_RELAY);

void setup() {}

void loop() {
    kicker.kick();
    delay(10000);
}
