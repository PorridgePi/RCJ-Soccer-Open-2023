#include <Arduino.h>
#include <Led.h>
#include <Temt.h>
#include <Wire.h>

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

void request() {
    Wire.write(data, sizeof(data));
}

void led_color(int pin, int r, int g, int b, int w) {
    led.begin(pin, 10);
    for (int i = 0; i < 10; i++) {
        led.color(i, r, g, b, w);
    }
    led.show();
}

void setup() {
    Serial.begin(115200);

    Wire.begin(8); // Need to change address?
    Wire.onRequest(request);

    led_color(18, 255, 0, 0, 0);
    delay(500);
    led_color(18, 0, 0, 0, 0);
    delay(500);
    led_color(18, 255, 255, 255, 255);
}

/*

Wire.requestFrom(52,2);
byte packet[2];
packet[1] = Wire.read();
packet[2] = Wire.read();
if (packet[1] == 0xFF) {
    return 256 * packet[1] + packet[2];
} else {
    return packet[1]
}

// Need to make the part that detects the 65535 value when nothing it detected by the temts

*/

void loop() {
    // Reset the values
    float sumX = 0, sumY = 0;
    bool  canSeeLine = false;
    static int angle;

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

    if (sumX != 0 || sumY != 0) {
        canSeeLine = true;
        angle      = atan2f(-sumX, -sumY) / 3.14159265358979323846f * 180;
        if (angle < 0) {
            angle += 360;
        }
    } else {
        angle = 65535; // Largest 16 bit number, used to indicate when there is no need to avoid the line
    }

    data[0] = angle & 0xFF;
    data[1] = (angle >> 8) & 0xFF;

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
