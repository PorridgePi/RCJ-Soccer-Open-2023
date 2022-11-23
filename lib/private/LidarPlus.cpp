#include "LidarPlus.h"

LidarPlus::LidarPlus() {
}

void LidarPlus::setup(HardwareSerial &LIDAR_SERIAL) {
    LIDAR_SERIAL.begin(TFMINIPLUS_BAUDRATE);
    delay(50);

    tf.begin(&LIDAR_SERIAL);
    tf.sendCommand(SET_FRAME_RATE, FRAME_100);
    delay(50);

    Serial.println("Lidar setup done.");
}

float LidarPlus::read() {
    if (serialMetro.check() == 1) {
        tf.getData(tfDist);
        return tfDist;
    }
    return -1;
}
