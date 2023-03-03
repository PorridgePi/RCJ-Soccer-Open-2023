#include <Arduino.h>
#include <Wire.h>

#define READ_FREQUENCY 1000 // every x ms

unsigned long long prevI2C = 0;
int angle;
int data[2];

void setup() {
    Serial.begin(115200);
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();
}

void loop() {
    unsigned long long time = micros();

    if (micros() - prevI2C > READ_FREQUENCY * 1000) {
        Wire.requestFrom(8, 2);
        if (Wire.available() == 2) {
            data[0] = Wire.read();
            data[1] = Wire.read();
        }
        angle = data[0] + data[1] * 256;
        prevI2C = micros();
    }

    Serial.print(angle);
    // Serial.print((float)(micros() - time) / 1000);
    Serial.println();
}
