#include <Arduino.h>
#include <CommonUtils.h>
#include <Definitions.h>
#include <FastLED.h>
#include <Temt.h>
#include <Wire.h>

#define TEMT_THRESHOLD  800 // to be calibrated
#define DEBUG           false
#define DEBUG_LOOP_TIME false
#define NUM_LEDS        10
#define USE_DIGITAL     true

// PIN, X POSITION, Y POSITION
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15

unsigned long loopStartMicros;
bool          isOnLine;
CRGB          leds[NUM_LEDS];
float      sumX, sumY, angle;

#define TEMT_COUNT 8

Temt temts[TEMT_COUNT] = {
    Temt(33, 0.4, 1),
    Temt(32, 1, 0.4),
    // Temt(4, 1, 0),
    Temt(2, 1, -0.4),
    Temt(15, 0.4, -1),
    Temt(13, -0.4, -1),
    Temt(14, -1, -0.4),
    // Temt(27, -1, 0),
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
    static unsigned long lastOnLine = millis();
    loopStartMicros = micros(); // For debugging loop time
    isOnLine        = false;

    sumX = 0;
    sumY = 0;

    // Read the values from the sensors
    for (int i = 0; i < TEMT_COUNT; i++) {
        if (DEBUG) {
            Serial.print(temts[i].read());
            Serial.print("\t");
        }

        if (temts[i].read() > TEMT_THRESHOLD) {
            isOnLine = true;
            // break;
            sumX += temts[i].X;
            sumY += temts[i].Y;
        }
    }

    const int NUM_SEGMENTS = 8;
    if (sumX != 0 || sumY != 0) {
        angle = LIM_ANGLE(DEG(atan2f(-sumX, -sumY)));
        angle = floor(angle / (360 / NUM_SEGMENTS)) + 1; // Convert to 1 to 8
    } else {
        angle = 0; // no line
    }

    if (DEBUG) {
        Serial.print(isOnLine); Serial.print("\t");
        Serial.print(angle / (NUM_SEGMENTS + 1) * 255);
        Serial.print("\t");
    }
    
    if (USE_DIGITAL) {
        if (isOnLine) {
            lastOnLine = millis();
            digitalWrite(22, isOnLine); // Pico D1
            digitalWrite(21, isOnLine); // Pico D2
            if (DEBUG) Serial.print("1");
        } else if ((millis() - lastOnLine) < 300) {
            digitalWrite(22, isOnLine); // Pico D1
            digitalWrite(21, isOnLine); // Pico D2
            if (DEBUG) Serial.print("1-");
        } else {
            if (DEBUG) Serial.print("0");
        }

    } else {
        analogWrite(22, angle / (NUM_SEGMENTS + 1) * 255);
        analogWrite(21, angle / (NUM_SEGMENTS + 1) * 255);
    }

    if (DEBUG_LOOP_TIME) {
        Serial.print((float) (micros() - loopStartMicros) / 1000);
        Serial.print('\t');
    }

    if (DEBUG || DEBUG_LOOP_TIME)
        Serial.println();
}
