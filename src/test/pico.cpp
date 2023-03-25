#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial2.setRX(5);
    Serial2.setTX(4);
    Serial2.begin(4800);
}

void loop() {
    static uint8_t writeBuffer[6] = {174, 193, 32, 2, 255, 255};
    if (Serial2.available() == 0) Serial2.write(writeBuffer,6);
    if (Serial2.available()) {
      Serial.print(Serial2.read());
      Serial.print(' ');
    }
}
