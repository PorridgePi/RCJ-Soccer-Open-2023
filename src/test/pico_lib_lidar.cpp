#include <Arduino.h>
#include <Lidar.h>

int   frontDistRaw, backDistRaw, rightDistRaw, leftDistRaw;
int   frontDist, backDist, rightDist, leftDist;
Lidar lidarFront(0x12, -5);
Lidar lidarBack(0x11, +5);
Lidar lidarRight(0x13, +4);
Lidar lidarLeft(0x10, +4);

void setup() {
    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.begin();
    Serial.begin(115200);
}

void loop() {
    frontDistRaw = lidarFront.readRaw(); // 2cm from center
    backDistRaw  = lidarBack.readRaw();  // 2cm from center
    rightDistRaw = lidarRight.readRaw(); // 3.5cm from center
    leftDistRaw  = lidarLeft.readRaw();  // 3.5cm from center

    Serial.print("fRaw: ");
    Serial.print(frontDistRaw);
    Serial.print("\tbRaw: ");
    Serial.print(backDistRaw);
    Serial.print("\trRaw: ");
    Serial.print(rightDistRaw);
    Serial.print("\tlRaw: ");
    Serial.print(leftDistRaw);

    frontDist = lidarFront.read(); // 2cm from center
    backDist  = lidarBack.read();  // 2cm from center
    rightDist = lidarRight.read(); // 3.5cm from center
    leftDist  = lidarLeft.read();  // 3.5cm from center

    Serial.print("\tFront: ");
    Serial.print(frontDist);
    Serial.print("\tBack: ");
    Serial.print(backDist);
    Serial.print("\tRight: ");
    Serial.print(rightDist);
    Serial.print("\tLeft: ");
    Serial.print(leftDist);

    Serial.println();
}
