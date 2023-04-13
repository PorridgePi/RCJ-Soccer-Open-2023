#include <Arduino.h>
#include <Definitions.h>
//#include <MechaQMC5883.h>
#include <Wire.h>
#include <IMU.h>

//MechaQMC5883 imu(Wire1, -244, -305, 1.05993520658, 56.2635641705);
// MechaQMC5883 imu(Wire1, 0, 0, 1.07350865912, 66.1081956153);

IMU imu(Wire1, 75, -10, 1, 0);

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
        driveBase.setDrive(0, 0, 0.05, 0);
    } else {
        driveBase.setDrive(0, 0, -0.05, 0);
    }
    imu.printRaw();
    // Serial.println(imu.readAngle());
    delay(5);
}