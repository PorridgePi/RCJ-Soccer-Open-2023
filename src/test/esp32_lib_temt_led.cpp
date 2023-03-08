#include <Arduino.h>
#include <Led.h>
#include <Temt.h>

Led led;

void led_color(int pin, int r, int g, int b, int w) {
    led.begin(pin, 10);
    for (int i = 0; i < 10; i++) {
        led.color(i, r, g, b, w);
    }
    led.show();
}

void setup() {
    Serial.begin(115200);

    led_color(18, 255, 0, 0, 0);
    delay(500);
    led_color(18, 0, 0, 0, 0);
    delay(500);
    led_color(18, 255, 255, 255, 255);
}

// PIN, X POSITION, Y POSITION
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15

Temt temts[10] = {
    Temt(33, 0.4, 1),
    Temt(32, 1, 0.4),
    Temt(4, 1, 0),
    Temt(2, 1, -0.4),
    Temt(15, 0.4, -1),
    Temt(13, -0.4, -1),
    Temt(14, -1, -0.4),
    Temt(27, -1, 0),
    Temt(26, -1, 0.4),
    Temt(25, -0.4, 1),
};

void loop() {
    for (int i = 0; i < 10; i++) {
        Serial.print(temts[i].read());
        Serial.print("\t");
    }
    Serial.println();
}
