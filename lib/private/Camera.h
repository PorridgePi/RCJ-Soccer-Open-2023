#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define BUFFER_LENGTH 50

class Camera : public SoftwareSerial {
    public:
        Camera(pin_size_t rx, pin_size_t tx) : SoftwareSerial (rx, tx) {};

        bool isNewDataPresent() {
            if (_newData == true) {
                parseData();
                memset(_buffer, 0, sizeof(_buffer));
                _newData = false;
            }
            return _newData;
        }

        void readData() {
            uint8_t buf[6] = {174, 193, 32, 2, 255, 255};
            SoftwareSerial::write(buf, 6);

            while (SoftwareSerial::available() > 0 && _newData == false) {
                _prevNum = _r;
                _r = SoftwareSerial::read();

                if (_recvInProgress == true) {
                    if (_i == 3) {
                        if (_r % 14 == 0) {
                            _length += _r + 2; // account for 2 bytes of checksum
                            constrain(_length, 0, BUFFER_LENGTH - 1);
                        }
                    }
                    if (_i < _length) {
                        _buffer[_i] = _r;
                        _i++;
                    } else {
                        _recvInProgress = false;
                        _i = 2;
                        _length = 4;
                        _newData = true;
                    }
                } else if (_r == 193 && _prevNum == 175) {
                    _buffer[0] = 175;
                    _buffer[1] = 193;
                    _recvInProgress = true;
                }
            }
        }

        void parseData() {
            // bytes 6-7 = signature
            // bytes 8-9 = x
            // bytes 10-11 = y
            // bytes 12-13 = width
            // bytes 14-15 = height

            if (_buffer[2] == 33 && _buffer[6] > 0) {
                _signature = _buffer[6] + _buffer[7] * 256;
                _x         = _buffer[8] + _buffer[9] * 256;
                _y         = _buffer[10] + _buffer[11] * 256;
                _width     = _buffer[12] + _buffer[13] * 256;
                _height    = _buffer[14] + _buffer[15] * 256;
                _blockLastDetected = 0;
            } else {
                if (_blockLastDetected > 10) { // if no block detected for 10 loops, set values to -1
                    _signature = -1;
                    _x         = -1;
                    _y         = -1;
                    _width     = -1;
                    _height    = -1;
                }
                _blockLastDetected++;
            }
        }

        void printData() {
            Serial.print(_signature);
            Serial.print("\t");
            Serial.print(_x);
            Serial.print("\t");
            Serial.print(_y);
            Serial.print("\t");
            Serial.print(_width);
            Serial.print("\t");
            Serial.print(_height);
            Serial.print("\t");
        }

    private:
        bool _newData = false;
        bool _recvInProgress = false;

        int _signature, _x, _y, _width, _height;

        int _i = 2;
        int _r = 0;
        int _length = 4;
        int _prevNum = 0;
        int _blockLastDetected = 0;

        int _buffer[BUFFER_LENGTH];
};

#endif
