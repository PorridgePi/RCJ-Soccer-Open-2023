#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial IMU(15,14);

void setup() {
    IMU.begin(57600);
    Serial.begin(9600);
}

void loop() {
    delay(10);
    if (IMU.available() > 0) {
        while (IMU.available() > 0) {
            Serial.print(IMU.read());
        }
        Serial.println();
    }
}