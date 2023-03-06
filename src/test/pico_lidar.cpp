#include <Arduino.h>
#include <Lidar.h>

int frontDist;
Lidar lidarFront(0x10);

void setup() {
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.begin();
    Serial.begin(115200);
}

void loop() {
    frontDist = lidarFront.read();
    Serial.println(frontDist);
}
