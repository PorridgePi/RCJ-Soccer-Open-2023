#include "Temt.h"
#include "Wire.h"
#include <Arduino.h>
#define TEMT_THRESHOLD 100

// PIN, X POSITION, Y POSITION
//    25   33
// 26         32
// 27         04
// 14         02
//    13   15

float angle      = 0;
bool  canSeeLine = false;

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
    int16_t angleToBeSent = angle;
    byte    packet[2];
    packet[0] = (angleToBeSent >> 8) & 0xFF;
    packet[1] = angleToBeSent & 0xFF;
    Wire.write(packet, 2);
}

void setup() {
    Wire.begin(54); // Need to change address?
    Serial.begin(115200);
    Wire.onRequest(request);
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

// Need to make the part that detects the 65535 value when nothing it detected by them temts

*/

void loop() {
    float sumX = 0;
    float sumY = 0;
    for (int i = 0; i < 11; i++) {
        if (temts[i].read() > TEMT_THRESHOLD) {
            sumX = sumX + temts[i].X;
            sumY = sumY + temts[i].Y;
        }
    }
    if (sumX != 0 || sumY != 0) {
        canSeeLine = true;
        angle      = atan2f(sumX, sumY) / 3.14159265358979323846f * 180;
    } else {
        angle = 65535; // Largest 16 it number, used to indicate when there is
                       // no need to avoid the line
    }
    Serial.println(angle);
}
