#include "Bluetooth.h"
#include "Camera.h"
#include "Localisation.h"
#include "config.h"

Bluetooth::Bluetooth(HardwareSerial &BLUETOOTH_SERIAL) :
    _BLUETOOTH_SERIAL(BLUETOOTH_SERIAL) { // initialisation list
                                          // e.g. Serial1
}

void Bluetooth::setup() {
    // setTX setRX to define the pins for Serial 1 - is it necessary?
    Serial1.setTX(12); // not using BLUETOOTH_SERIAL???
    Serial1.setRX(13);
    _BLUETOOTH_SERIAL.begin(BLUETOOTH_BAUDRATE);
    delay(1000);
    Serial.println("Bluetooth setup done.");
}

void Bluetooth::read() {
    while (_BLUETOOTH_SERIAL
               .available()) { // while there is data to read from buffer
        byte incomByte =
            _BLUETOOTH_SERIAL
                .read(); // first byte of incoming serial data available

        for (int i = packetIdx; i > 0; i--) {
            bltBuffer[i] = bltBuffer[i - 1];
        }
        bltBuffer[0] = incomByte;
        byte flag    = bltBuffer[packetIdx];

        if (flag == 255) {
            byte ball_dist = _BLUETOOTH_SERIAL.read();
            int  ball_dir  = _BLUETOOTH_SERIAL.read() * 3;

            Serial.print("(");
            Serial.print(ball_dist);
            Serial.print(", ");
            Serial.print(ball_dir);
            Serial.print(") ");
        }
    }
}

void Bluetooth::write(
    byte ball_x,
    byte ball_y) { // coords from camera (cam.ball_x, cam.ball_y)
    _BLUETOOTH_SERIAL.write(255);
    _BLUETOOTH_SERIAL.write(ball_x);
    _BLUETOOTH_SERIAL.write(ball_y);
}

void Bluetooth::requestBall() {
    // _BLUETOOTH_SERIAL.write(0); // change stuff in bracket
}
