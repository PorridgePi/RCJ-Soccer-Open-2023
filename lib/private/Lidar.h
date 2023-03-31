#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <Wire.h>

class Lidar {
    public:
        Lidar(TwoWire &wire, int addr, int calibration = 0) :
            _addr(addr), _calibration(calibration), _wire(wire) {
        } // constructor

        int read() {
            return constrain(readRaw() + _calibration, 0, 255);
        }

        int readRaw() {
            _wire.beginTransmission(_addr); // start bit
            _wire.write(0x00);              // tell tfluna which register to read from, this register is low dist

            // unsigned int timeStart = millis();
            _wire.endTransmission(); // end bit
            // Serial.print(millis()-timeStart);
            // Wire.endTransmission() and Wire.requestFrom() are timing out?

            _wire.requestFrom(_addr, 1); // request 1 bit
            int response = _wire.read(); // read the bit

            // Serial.print("\t");
            // Serial.println(response);
            return response;
        }

        int getFPS() {
            _wire.beginTransmission(_addr);
            _wire.write(0x26); // this register is low FPS
            _wire.endTransmission();
            _wire.requestFrom(_addr, 1);
            return _wire.read();
        }

        void setFPS(int fps) {
            _wire.beginTransmission(_addr);
            _wire.write(0x26);
            _wire.write(fps);
            _wire.endTransmission();
        }

        void setAddress(int newaddr) {
            _wire.beginTransmission(_addr); // start bit
            _wire.write(0x22);              // this register is the slave address
            _wire.write(newaddr);           // write data, which is the new address
            _wire.endTransmission();        // end bit
            _addr = newaddr;
        }

    private:
        int _addr, _calibration;
        TwoWire &_wire;
};

#endif
