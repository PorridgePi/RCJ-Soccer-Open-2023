#include "LidarPlusI2C.h"

LidarPlusI2C::LidarPlusI2C() {
}

void LidarPlusI2C::setup(TwoWire &LIDAR_BUS, int Offset = 0) {
    LIDAR_BUS.begin(400000);
    tf.Set_Bus(&LIDAR_BUS);
    delay(50);

    tf.Set_Frame_Rate(tfFPS, tfAddr);
    delay(50);

    tfOffset = Offset;

    Serial.println("Lidar setup done.");
}

float LidarPlusI2C::read() {
    if (i2cMetro.check() == 1) {
        tf.getData(tfDist, tfAddr);
        return tfDist -= tfOffset;
    }
    return -1;
}
