#include "Arduino.h"
#include "sk.h"

sk led;

void setup() {
    led.begin(12, 10);
}

void loop() {
    led.begin(12, 10);
    for (int i = 0; i < 10; i++) {
        led.color(i, 255, 255, 255, 255);
    }
    led.show();
}
