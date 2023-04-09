#include <CommonUtils.h>
#include <MechaQMC5883.h>
#include <Wire.h>

MechaQMC5883  qmc(Wire1, 0, 0, 0, 0);
unsigned long t     = 0;
float         angle = -1;

void setup() {
    Wire1.setSCL(3);
    Wire1.setSDA(2);
    Wire1.begin();
    Serial.begin(9600);
    qmc.init();
    qmc.tare();
}

void loop() {
    t = micros(); // loop time

    // int x, y, z;
    // byte err = qmc.read(&x, &y, &z);
    // if (err) {
    //     qmc.init();
    //     Serial.print("err: ");
    //     Serial.print(err);
    //     Serial.print('\t');
    //     angle = -1;
    // } else {
    //     angle = 360-LIM_ANGLE(DEG(atan2(x, y)));
    // }

    angle = qmc.readAngle();
    Serial.print(angle);
    Serial.print('\t');
    Serial.print((float) (micros() - t) / 1000);
    Serial.print("\t");
    Serial.println();
}
