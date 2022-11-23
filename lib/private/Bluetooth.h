#ifndef BLUETOOTH_h
#define BLUETOOTH_h

#define packetSize 3
// #define BLUETOOTH_BAUDRATE 115200

#include "Camera.h"
#include "Localisation.h"
#include "config.h"
#include <Arduino.h>
#include <HardwareSerial.h>

class Bluetooth {
    private:
        HardwareSerial &_BLUETOOTH_SERIAL;
        // https://stackoverflow.com/questions/22766907/whats-the-semantically-accurate-position-for-the-ampersand-in-c-references
        // https://stackoverflow.com/questions/1943276/what-does-do-in-a-c-declaration
        int packetIdx = packetSize - 1;
        int bltBuffer[packetSize];

    public:
        Bluetooth(HardwareSerial &BLUETOOTH_SERIAL);
        void setup();
        void read();
        void write(byte ball_x, byte ball_y);
        void updateBot();
        void requestBall();
};

#endif
