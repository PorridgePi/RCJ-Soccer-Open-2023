#include <Arduino.h>
#include <FastLED.h>
#include <Temt.h>

#define NUM_LEDS 10
CRGB leds[NUM_LEDS];

void setup() {
    Serial.begin(115200);

    FastLED.addLeds<SK6812, 18, GRB>(leds, NUM_LEDS);
    FastLED.showColor(CRGB::White);
    delay(250);
    FastLED.showColor(CRGB::Black);
    delay(250);
    FastLED.showColor(CRGB::White);
}

// PIN, X POSITION, Y POSITION
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15
#define TEMT_DIFF  70
Temt temts[10] = {
    Temt(33, 0.4, 1, TEMT_DIFF),
    Temt(32, 1, 0.4, TEMT_DIFF),
    Temt(4, 1, 0, TEMT_DIFF),
    Temt(2, 1, -0.4, TEMT_DIFF),
    Temt(15, 0.4, -1, TEMT_DIFF),
    Temt(13, -0.4, -1, TEMT_DIFF),
    Temt(14, -1, -0.4, TEMT_DIFF),
    Temt(27, -1, 0, TEMT_DIFF),
    Temt(26, -1, 0.4, TEMT_DIFF),
    Temt(25, -0.4, 1, TEMT_DIFF),
};

void loop() {
    for (int i = 0; i < 10; i++) {
        Serial.print(temts[i].read());
        Serial.print("\t");
    }
    Serial.println();
}
