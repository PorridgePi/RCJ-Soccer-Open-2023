#include <Arduino.h>
#include <SoftwareSerial.h>

#define DEBUG_DELAY true

#define PIXY_TX 8
#define PIXY_RX 15

SoftwareSerial PixySerial(PIXY_RX, PIXY_TX);

void setup() {
    Serial.begin(19200);
    PixySerial.begin(19200);
}

int signature, x, y, width, height;

int headers[6];
int block[14];

void loop() {
    long long time = micros();
    uint8_t buf[6] = {174, 193, 32, 2, 255, 255};
    PixySerial.write(buf, 6);
    if (PixySerial.available()) {
        headers[0] = PixySerial.read();
        while (headers[0] != 175) { // 16-bit sync - Bit 0 = 175
            headers[0] = PixySerial.read();
        }
        for (int i = 1; i < 6; i++) {
            headers[i] = PixySerial.read();
        }
    }

    int packetType = headers[2]; // should be 33
    int payloadLength = headers[3]; // multiple of 14

    if (headers[1] == 193 && packetType == 33 && payloadLength > 0) { // 16-bit sync - Bit 1 = 193

        for (int i = 0; i < 6; i++) {
            Serial.print(headers[i]);
            Serial.print("\t");
        }

        for (int i = 0; i < payloadLength + 1; i++) { // +1 to re-run loop
            // bytes 6-7 = signature
            // bytes 8-9 = x
            // bytes 10-11 = y
            // bytes 12-13 = width
            // bytes 14-15 = height

            if (i % 14 == 0) {
                if (i > 0) {
                    Serial.print("\t");
                    int signature = block[0] + block[1] * 256;
                    int x = block[2] + block[3] * 256;
                    int y = block[4] + block[5] * 256;
                    int width = block[6] + block[7] * 256;
                    int height = block[8] + block[9] * 256;

                    Serial.print(signature);
                    if (signature == 1) {
                        Serial.print("\t");
                        Serial.print(x);
                        Serial.print("\t");
                        Serial.print(y);
                        Serial.print("\t");
                        Serial.print(width);
                        Serial.print("\t");
                        Serial.print(height);
                        Serial.print("\t");
                    }

                    // for (int j = 0; j < 14; j++) {
                    //     Serial.print(block[j]);
                    //     Serial.print("\t");
                    // }
                    Serial.print("\t");

                }
            }
            if (i != payloadLength + 1) {
                block[i % 14] = PixySerial.read();
            }
        }
        Serial.println();
    }
    if (DEBUG_DELAY) {
        delay(10);
    }
}
