#include "Arduino.h"
#include "sk.h"

sk led;

void setup() {
    led.begin(18, 10);
    for (int i = 0; i < 10; i++) {
        led.color(i, 255, 255, 255, 255);
    }
    led.show();
}

void loop() {
    // led.begin(18, 10);
    // for (int i = 0; i < 10; i++) {
    //     led.color(i, 255, 255, 255, 255);
    // }
    // led.show();
}
