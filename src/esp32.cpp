#include <Arduino.h>
#include <Led.h>
#include <Temt.h>
#include <Wire.h>

#define I2C_ADDRESS_ESP32 8
#define TEMT_THRESHOLD 1000 // to be calibrated
#define DEBUG          true

// PIN, X POSITION, Y POSITION
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15

int angle;
Led   led;
uint8_t data[2];

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

// Write the data to the I2C bus on request
void request() {
    Wire.write(data, sizeof(data));
}

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

    Wire.begin(I2C_ADDRESS_ESP32);
    Wire.onRequest(request);

    led_color(18, 255, 0, 0, 0); // Non-white to indicate power on
    delay(500);
    led_color(18, 0, 0, 0, 0); // Turn off the LED temporarily to blink
    delay(500);
    led_color(18, 255, 255, 255, 255); // White to indicate ready
}

void loop() {
    // Reset the values
    float sumX = 0, sumY = 0;
    bool  canSeeLine = false;
    static int angle;

    // Read the values from the sensors
    for (int i = 0; i < 10; i++) {
        if (DEBUG) {
            Serial.print(temts[i].read());
            Serial.print("\t");
        }

        if (temts[i].read() > TEMT_THRESHOLD) {
            sumX += temts[i].X;
            sumY += temts[i].Y;
        }
    }

    // Calculate the angle
    if (sumX != 0 || sumY != 0) {
        canSeeLine = true;
        angle      = atan2f(-sumX, -sumY) / 3.14159265358979323846f * 180;
        if (angle < 0) angle += 360; // Convert to 0-359 degrees
    } else {
        angle = 65535; // Largest 16 bit number indicating no need to avoid line
    }

    // Send the data to the ESP32
    data[0] = angle & 0xFF;
    data[1] = (angle >> 8) & 0xFF;

    // Print the data for debugging
    if (DEBUG) {
        Serial.print(sumX);
        Serial.print("\t");
        Serial.print(sumY);
        Serial.print("\t");
        Serial.print(canSeeLine);
        Serial.print("\t");
        Serial.print(angle);
    }

    Serial.println();
}
