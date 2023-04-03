#include <Arduino.h>
#include <IMU.h>
#include <Wire.h>

IMU imu(Wire, 0x1E);

void setup() {
    Wire.setSCL(5);
    Wire.setSDA(4);
    Wire.begin();
    Serial.begin(9600);
    imu.init();
    imu.tare();
}

void loop() {
    Serial.println(imu.readAngle());
    delay(10);
}