#include <Arduino.h>
#include <Lidar.h>

int   frontDist, backDist, rightDist, leftDist;
Lidar lidarFront(0x12);
Lidar lidarBack(0x11);
Lidar lidarRight(0x13);
Lidar lidarLeft(0x10);

void setup() {
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.begin();
    Serial.begin(115200);
}

void loop() {
    frontDist = lidarFront.read(); // 2cm from center
    backDist  = lidarBack.read();  // 2cm from center
    rightDist = lidarRight.read(); // 3.5cm from center
    leftDist  = lidarLeft.read();  // 3.5cm from center

    Serial.print("Front: ");
    Serial.print(frontDist - 5);
    Serial.print("\tBack: ");
    Serial.print(backDist + 5);
    Serial.print("\tRight: ");
    Serial.print(rightDist + 4);
    Serial.print("\tLeft: ");
    Serial.print(leftDist + 4);
    Serial.println();
}
