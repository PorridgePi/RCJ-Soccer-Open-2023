#include <Arduino.h>

void setup() {
    Serial.begin(19200);
    Serial1.begin(19200);
}

int signature, x, y, width, height;
int response[20];

void loop() {
    uint8_t buf[6] = {174, 193, 32, 2, 255, 255};
    Serial1.write(buf, 6);
    if (Serial1.available()) {
        response[0] = Serial1.read();
        while (response[0] != 175) {
            response[0] = Serial1.read();
        }
        for (int i = 1; i < 20; i++) {
            response[i] = Serial1.read();
        }
        delay(100);
    }

    // for (int i = 0; i < 20; i++) {
    //     Serial.print(response[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    signature = response[6] + response[7] * 256;

    if (signature == 1) { // ball
        x = response[8] + response[9] * 256;
        y = response[10] + response[11] * 256;
        width = response[12] + response[13] * 256;
        height = response[14] + response[15] * 256;

        Serial.print(signature);
        Serial.print("\t");
        Serial.print(x);
        Serial.print("\t");
        Serial.print(y);
        Serial.print("\t");
        Serial.print(width);
        Serial.print("\t");
        Serial.println(height);
        return;
    }
}
