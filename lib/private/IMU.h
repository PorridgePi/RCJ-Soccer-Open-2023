#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Definitions.h>
#include <Wire.h>

class IMU {
    public:
        IMU(int addr) :
            _addr(addr){};

        void init() {
            Wire.beginTransmission(_addr);
            Wire.write(0x02);
            Wire.write(0x00);
            Wire.endTransmission();
            _initialised = true;
        };

        void tare() {
            _update();
            _zeroError = DEG(atan2(_mag[0], _mag[1])); // Account for the subtraction of the zero error
        }

        float readAngle() {
            _update();
            float angle = DEG(atan2(_mag[0], _mag[1])) - _zeroError;

            return angle > 0 ? fmod(angle, 360) : fmod(angle, 360) + 360;
        }

        void printRaw() {
            _update();
            Serial.print("(");
            Serial.print(_mag[0]);
            Serial.print(",");
            Serial.print(_mag[1]);
            Serial.println(")");
        }

        void setCalibration(int x, int y, int a, int b, int c) {
            // REFER TO https://www.desmos.com/calculator/jk8bslhyx1 FOR THE VALUES
            _calibration[0] = x;
            _calibration[1] = y;
            _calibration[2] = a;
            _calibration[3] = b;
            _calibration[4] = c;
        }

    private:
        const int _addr;
        int       _calibration[5]; //[x,y,a,b,c]
        float     _zeroError;
        bool      _initialised = false;
        int       _mag[2];

        void _update() {
            if (Wire.available() < 6) {
                Wire.beginTransmission(_addr);
                Wire.write(0x03); // select register 3, X MSB register
                byte error = Wire.endTransmission();
                if (error) {
                    Serial.println("Error occured when writing");
                    if (error == 5) Serial.println("It was a timeout");
                }
            }


            Wire.requestFrom(_addr, 6);
            if (Wire.available() >= 6) {
                int buff[6];
                for (unsigned int i = 0; i < 6; i++) {
                    buff[i] = Wire.read();
                }

                int j[2];
                j[0] = -1 * (int16_t) (((((uint16_t) buff[4]) << 8) | buff[5])); // X axis (internal sensor -y axis)
                j[1] = -1 * (int16_t) (((((uint16_t) buff[0]) << 8) | buff[1])); // Y axis (internal sensor -x axis)
                // uncorrected[2] = -1 * (int16_t) (((((uint16_t) buff[2]) << 8) | buff[3])); // Z axis (internal sensor -z axis)

                // CORRECTING FOR ERRORS
                if (_calibration[2] != 0 && _calibration[3] != 0) {
                    // Center the ellipse:
                    j[0] -= _calibration[0];
                    j[1] -= _calibration[1];

                    //_calibration = [x, y, a, b, angle]
                    float sine   = sinf(_calibration[4] / 180 * PI);
                    float cosine = cosf(_calibration[4] / 180 * PI);

                    // Rotate the vectors by c:
                    int k[2];
                    k[0] = j[0] * cosine + j[1] * sine * _calibration[2] / _calibration[3];
                    k[1] = -j[0] * sine + j[1] * cosine;

                    _mag[0] = k[0] * cosine + k[1] * sine;
                    _mag[1] = -k[0] * sine + k[1] * cosine;
                } else {
                    _mag[0] = j[0];
                    _mag[1] = j[1];
                }
                _mag[0] *= -1; // Since imu is upside down
            };
        };
};

#endif