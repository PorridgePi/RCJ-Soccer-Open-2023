#include <Arduino.h>
#include <IMU.h>
#include <Wire.h>

IMU imu(Wire, 0x1E);

#include <Drive.h>
#define MAX_SPEED 0.5
Motor motorFR(21, 20); // top left JST, top right motor
Motor motorBR(26, 22); // bottom left JST, bottom right motor
Motor motorBL(3, 7);   // bottom right JST, bottom left motor
Motor motorFL(11, 9);  // top right JST, top left motor

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

void setup() {
    Serial.begin(9600);

    Wire.setSCL(13);
    Wire.setSDA(12);
    Wire.begin();

    imu.init();

    imu.setCalibration(159, 32, 516, 530, -53);
    // imu.tare();
}

void loop() {
    // driveBase.setDrive(0,0,0.2);
    imu.printRaw();
    delay(100);
}