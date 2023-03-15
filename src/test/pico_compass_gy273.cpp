#include <Arduino.h>
#include <Wire.h>

#define HMC388L_ADDRESS 0x1E

int mag[3];

void setup() {
    Serial.begin(9600);
    Wire.setSCL(5);
    Wire.setSDA(4);
    Wire.begin();

    Wire.beginTransmission(HMC388L_ADDRESS);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();
}

void loop() {
    Wire.beginTransmission(HMC388L_ADDRESS);
    Wire.write(0x03); // select register 3, X MSB register
    Wire.endTransmission();

    Wire.requestFrom(HMC388L_ADDRESS, 6);
    if (Wire.available() >= 6) {
        int buff[6];
        for (unsigned int i = 0; i < 6; i++) {
            buff[i] = Wire.read();
        }
        mag[0] = -1 * (int16_t) (((((uint16_t) buff[4]) << 8) | buff[5])); // X axis (internal sensor -y axis)
        mag[1] = -1 * (int16_t) (((((uint16_t) buff[0]) << 8) | buff[1])); // Y axis (internal sensor -x axis)
        mag[2] = -1 * (int16_t) (((((uint16_t) buff[2]) << 8) | buff[3])); // Z axis (internal sensor -z axis)
    }
    Serial.println(atan2(m[0], m[1]) / PI * 180);
}
