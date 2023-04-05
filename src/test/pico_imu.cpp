#include <Arduino.h>
#include <IMU.h>
#include <Wire.h>

IMU imu(Wire1, 0x1E);

void setup() {
    Wire1.setSCL(3);
    Wire1.setSDA(2);
    Wire1.begin();
    Serial.begin(9600);
    imu.init();
    imu.tare();
}

void loop() {
    Serial.println(imu.readAngle());
    delay(10);
}
