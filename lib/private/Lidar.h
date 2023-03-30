#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <Wire.h>

class Lidar {
    public:
        Lidar(int addr, int calibration = 0) :
            _addr(addr), _calibration(calibration) {
        } // constructor

        int read() {
            return constrain(readRaw() + _calibration, 0, 255);
        }

        int readRaw() {
            Wire.beginTransmission(_addr); // start bit
            Wire.write(0x00);              // tell tfluna which register to read from, this register is low dist

            // unsigned int timeStart = millis();
            Wire.endTransmission(); // end bit
            // Serial.print(millis()-timeStart);
            // Wire.endTransmission() and Wire.requestFrom() are timing out?

            Wire.requestFrom(_addr, 1); // request 1 bit
            int response = Wire.read(); // read the bit

            // Serial.print("\t");
            // Serial.println(response);
            return response;
        }

        int getFPS() {
            Wire.beginTransmission(_addr);
            Wire.write(0x26); // this register is low FPS
            Wire.endTransmission();
            Wire.requestFrom(_addr, 1);
            return Wire.read();
        }

        void setFPS(int fps) {
            Wire.beginTransmission(_addr);
            Wire.write(0x26);
            Wire.write(fps);
            Wire.endTransmission();
        }

        void setAddress(int newaddr) {
            Wire.beginTransmission(_addr); // start bit
            Wire.write(0x22);              // this register is the slave address
            Wire.write(newaddr);           // write data, which is the new address
            Wire.endTransmission();        // end bit
            _addr = newaddr;
        }

    private:
        int _addr, _calibration;
};

#endif
