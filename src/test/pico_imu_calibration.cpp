#include <Arduino.h>
#include <Definitions.h>
#include <MechaQMC5883.h>
#include <Wire.h>
#include <IMU.h>

// MechaQMC5883 imu(Wire1, -190, -236, 1.03225739008, 53.8968998045);
IMU imu(Wire1, 0, 0, 0.983015342964, 49.1136970449);

#include <Drive.h>
#define MAX_SPEED 0.5
Motor motorFR(PIN_BOT_B_LPWM, PIN_BOT_B_RPWM);
Motor motorBR(PIN_BOT_A_RPWM, PIN_BOT_A_LPWM);
Motor motorBL(PIN_TOP_B_RPWM, PIN_TOP_B_LPWM);
Motor motorFL(PIN_TOP_A_RPWM, PIN_TOP_A_LPWM);

Drive driveBase(motorFR, motorBR, motorBL, motorFL);

void setup() {
    Serial.begin(9600);

    // I2C for IMU
    Wire1.setSCL(PIN_WIRE1_GY_SCL);
    Wire1.setSDA(PIN_WIRE1_GY_SDA);
    Wire1.setTimeout(1); // set timeout to 1 ms
    Wire1.begin();
    imu.init();
    // imu.tare();
}

void loop() {
    if (millis() % 10000 <= 5000) {
        driveBase.setDrive(0, 0, 0.05);
    } else {
        driveBase.setDrive(0, 0, -0.05);
    }
    imu.printRaw();
    // Serial.println(imu.readAngle());
    delay(5);
}