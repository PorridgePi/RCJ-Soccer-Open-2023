#include "FastLED.h"

#define NUM_LEDS 10
CRGB leds[NUM_LEDS];

void setup() {
    FastLED.addLeds<SK6812, 18, GRB>(leds, NUM_LEDS);
    FastLED.showColor(CRGB::White);
    delay(250);
    FastLED.showColor(CRGB::Black);
    delay(250);
    FastLED.showColor(CRGB::White);
}

void loop() {}
