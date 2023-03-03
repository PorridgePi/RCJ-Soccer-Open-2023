#include <Arduino.h>
#include <Led.h>
#include <Temt.h>
#include <Wire.h>
#include <Definitions.h>

#define TEMT_THRESHOLD    1000 // to be calibrated
#define DEBUG             false
#define DEBUG_LOOP_TIME   true

// PIN, X POSITION, Y POSITION
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15

Led           led;
unsigned long loopStartMicros;
bool isOnLine;

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

// Set the color of the LED
void led_color(int pin, int r, int g, int b, int w) {
    led.begin(pin, 10);
    for (int i = 0; i < 10; i++) {
        led.color(i, r, g, b, w);
    }
    led.show();
}

void setup() {
    Serial.begin(115200);
    pinMode(22, OUTPUT);

    led_color(18, 255, 0, 0, 0); // Non-white to indicate power on
    delay(500);
    led_color(18, 0, 0, 0, 0); // Turn off the LED temporarily to blink
    delay(500);
    led_color(18, 255, 255, 255, 255); // White to indicate ready
}

void loop() {
    loopStartMicros = micros(); // For debugging loop time
    isOnLine = false;

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
    } else {
        digitalWrite(22, LOW); // write SCL HIGH
    }

    if (DEBUG_LOOP_TIME) {
        Serial.print((float) (micros() - loopStartMicros) / 1000);
        Serial.print('\t');
    }

    if (DEBUG || DEBUG_LOOP_TIME) Serial.println();
}
