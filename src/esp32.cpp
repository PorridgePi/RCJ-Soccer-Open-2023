#include <Arduino.h>
#include <Definitions.h>
#include <FastLED.h>
#include <Temt.h>
#include <Wire.h>

#define TEMT_THRESHOLD  2000 // to be calibrated
#define DEBUG           false
#define DEBUG_LOOP_TIME true
#define NUM_LEDS        10

// PIN, X POSITION, Y POSITION
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15

unsigned long loopStartMicros;
bool          isOnLine;
CRGB          leds[NUM_LEDS];

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

void setup() {
    Serial.begin(115200);
    pinMode(22, OUTPUT);

    FastLED.addLeds<SK6812, 18, GRB>(leds, NUM_LEDS);
    FastLED.showColor(CRGB::White); // White to indicate power on
    delay(250);
    FastLED.showColor(CRGB::Black); // Turn off the LED temporarily to blink
    delay(250);
    FastLED.showColor(CRGB::White); // White to indicate ready
}

void loop() {
    loopStartMicros = micros(); // For debugging loop time
    isOnLine        = false;

    // Read the values from the sensors
    for (int i = 0; i < 10; i++) {
        if (DEBUG) {
            Serial.print(temts[i].read());
            Serial.print("\t");
        }

        if (temts[i].read() > TEMT_THRESHOLD) {
            isOnLine = true;
            break;
        }
    }

    if (isOnLine == true) {
        digitalWrite(22, HIGH); // write SCL HIGH
        if (DEBUG) {
            Serial.print("LINE");
            Serial.print('\t');
        }
    } else {
        digitalWrite(22, LOW); // write SCL HIGH
    }

    if (DEBUG_LOOP_TIME) {
        Serial.print((float) (micros() - loopStartMicros) / 1000);
        Serial.print('\t');
    }

    if (DEBUG || DEBUG_LOOP_TIME)
        Serial.println();
}
