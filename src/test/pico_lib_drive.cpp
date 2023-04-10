#include <Arduino.h>
#include <Definitions.h>
#include <Drive.h>

#define MAX_SPEED 0.5
#define WITH_IMU

#ifdef WITH_IMU
#include <MechaQMC5883.h>
#include <PID.h>
PID          pid(0.005, 0, 0.2, 1000);
MechaQMC5883 imu(Wire1, 0, 0, 0, 0);
#endif

Motor motorFR(PIN_BOT_B_LPWM, PIN_BOT_B_RPWM);
Motor motorBR(PIN_BOT_A_RPWM, PIN_BOT_A_LPWM);
Motor motorBL(PIN_TOP_B_RPWM, PIN_TOP_B_LPWM);
Motor motorFL(PIN_TOP_A_RPWM, PIN_TOP_A_LPWM);
Drive driveBase(motorFR, motorBR, motorBL, motorFL);

void setup() {
    Serial.begin(9600);

    #ifdef WITH_IMU
    // I2C for IMU
    Wire1.setSCL(PIN_WIRE1_SCL);
    Wire1.setSDA(PIN_WIRE1_SDA);
    Wire1.setTimeout(1); // set timeout to 1 ms
    Wire1.begin();
    imu.init();
    imu.tare(); // set current angle as heading 0
    #endif
}
void loop() {
    float targetSpeed, rotationRate;

    targetSpeed  = 0.5;
    rotationRate = 0;

    #ifdef WITH_IMU
    float botHeading = imu.readAngle();                                                                                              // from 0 to 360
    // rotationRate     = constrain((LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360) / 540, -1, 1); // from -180 to 180
    // rotationRate     = constrain(abs(rotationRate), 0.03, 1) * copysign(1, constrain(pid.compute(0, -(LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)), -1, 1));
    rotationRate = constrain(pid.compute(0, -(LIM_ANGLE(botHeading) <= 180 ? LIM_ANGLE(botHeading) : LIM_ANGLE(botHeading) - 360)), -1, 1);
    if (digitalRead(PIN_ORI_RESET) == HIGH) { // reset IMU if button pressed
        imu.tare();
    }
    #endif


    driveBase.setDrive(targetSpeed, 0, rotationRate, 0);
    // delay(1000);
    // driveBase.setDrive(targetSpeed, 90, rotationRate, 0);
    // delay(1000);
    // driveBase.setDrive(targetSpeed, 180, rotationRate, 0);
    // delay(1000);
    // driveBase.setDrive(targetSpeed, 270, rotationRate, 0);
    // delay(1000);
}
