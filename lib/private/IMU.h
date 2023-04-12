#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <CommonUtils.h>
#include <Wire.h>

class IMU {
    public:
    IMU(TwoWire &wire, float x, float y, float e, float angle) :
        _wire(wire), _calibration{x, y, e, angle} {};

    void init() {
        _wire.beginTransmission(_addr);
        _wire.write(0x02);
        _wire.write(0x00);
        _wire.endTransmission();
        _initialised = true;
    };

    void tare() {
        _update();
        _zeroError = DEG(atan2(_mag[0], _mag[1])); // Account for the subtraction of the zero error
    }

    float readAngle() {
        _update();
        float angle = DEG(atan2(_mag[0], _mag[1])) - _zeroError;

        return LIM_ANGLE(angle);
    }

    void printRaw() {
        _update();
        Serial.print("(");
        Serial.print(_mag[0]);
        Serial.print(",");
        Serial.print(_mag[1]);
        Serial.println(")");
    }

    private:
    const int _addr = 0X1E;
    int       _calibration[4]; //[x,y,a,b,c]
    float     _zeroError;
    bool      _initialised = false;
    int       _mag[2];
    TwoWire  &_wire;

    void _correctReadings(int *x, int *y) {
        // _calibration[4] = {xOffset, yOffset, a/b in desmos, angle}
        if (_calibration[0] != 0 and _calibration[1] != 0) {
            // Shift the center of the ellipse to the origin

            *x -= _calibration[0];
            *y -= _calibration[1];

            float _x = *x;
            float _y = *y;

            float sine   = sinf(_calibration[3] / 180 * PI);
            float cosine = cosf(_calibration[3] / 180 * PI);
            // Rotate the ellipse to the axis and compress the x axis into a circle
            *x = _x * cosine + _y * sine / _calibration[2];
            *y = -_x * sine + _y * cosine;

            _x = *x;
            _y = *y;

            *x = _x * cosine - _y * sine;
            *y = _x * sine + _y * cosine;
        };
    };

    void _update() {
        if (_wire.available() < 6) {
            _wire.beginTransmission(_addr);
            _wire.write(0x03); // select register 3, X MSB register
            byte error = _wire.endTransmission();
            if (error) {
                Serial.println("Error occured when writing");
                if (error == 5)
                    Serial.println("It was a timeout");
            }
        }

        _wire.requestFrom(_addr, 6);
        if (_wire.available() >= 6) {
            int buff[6];
            for (unsigned int i = 0; i < 6; i++) {
                buff[i] = _wire.read();
            }
            static int x, y;
            x = -1 * (int16_t) (((((uint16_t) buff[4]) << 8) | buff[5])); // X axis (internal sensor -y axis)
            y = -1 * (int16_t) (((((uint16_t) buff[0]) << 8) | buff[1])); // Y axis (internal sensor -x axis)
            // uncorrected[2] = -1 * (int16_t) (((((uint16_t) buff[2]) << 8) | buff[3])); // Z axis (internal sensor -z axis
            _correctReadings(&x, &y);
        }
    };
};

#endif