#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define BUFFER_LENGTH 50

class Camera : public SoftwareSerial {
    public:
        Camera(pin_size_t rx, pin_size_t tx, int xC, int yC) :
            SoftwareSerial(rx, tx), _xC(xC), _yC(yC) {
        }

        bool isNewDataPresent() {
            if (_newData == true) {
                parseData();
                memset(_buffer, 0, sizeof(_buffer));
                _newData = false;
            }
            return _newData;
        }

        void readData() {
            static int  currentByte    = 0;
            static int  prevByte       = 0;
            static int  i              = 2;
            static int  maxLength      = 4;
            static bool recvInProgress = false;

            static uint8_t writeBuffer[6] = {174, 193, 32, 2, 255, 255};
            if (SoftwareSerial::available() == 0) SoftwareSerial::write(writeBuffer, 6);

            while (SoftwareSerial::available() > 0 && _newData == false) {
                prevByte    = currentByte;
                currentByte = SoftwareSerial::read();

                if (recvInProgress == true) {
                    if (i == 3) {
                        if (currentByte % 14 == 0) {
                            // CONSTRAIN!
                            maxLength = constrain(maxLength + currentByte + 2, 0, BUFFER_LENGTH - 1); // account for 2 bytes of checksum
                        }
                    }
                    if (i < maxLength) {
                        _buffer[i] = currentByte;
                        i++;
                    } else {
                        recvInProgress = false;
                        i              = 2;
                        maxLength      = 4;
                        _newData       = true;
                    }
                } else if (currentByte == 193 && prevByte == 175) {
                    _buffer[0]     = 175;
                    _buffer[1]     = 193;
                    recvInProgress = true;
                }
            }
        }

        void parseData() {
            static int _blockLastDetected = 0;

            // bytes 6-7 = signature
            // bytes 8-9 = x
            // bytes 10-11 = y
            // bytes 12-13 = width
            // bytes 14-15 = height

            if (_buffer[2] == 33 && _buffer[6] > 0) {
                _signature         = constrain(_buffer[6] + _buffer[7] * 256, 0, 255);
                _x                 = constrain(_buffer[8] + _buffer[9] * 256, 0, 315);
                _y                 = constrain(_buffer[10] + _buffer[11] * 256, 0, 207);
                _width             = constrain(_buffer[12] + _buffer[13] * 256, 0, 316);
                _height            = constrain(_buffer[14] + _buffer[15] * 256, 0, 208);
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

        int getBallDistance() {
            if (_signature == 1) {
                int xDiff = _x - _xC;
                int yDiff = _y - _yC;

                // max distance = 5 * 84 = 420 since max change is around 84 pixels
                return constrain(5 * hypot(xDiff, yDiff), 0, 3000);
            } else {
                return -1;
            }
        }

        int getBallAngle() {
            if (_signature == 1) {
                int xDiff = _x - _xC;
                int yDiff = _y - _yC;

                int angle = atan2(yDiff, xDiff) * 180 / PI + 90;
                if (angle < 0) angle += 360; // make sure angle is positive
                angle = 360 - angle; // invert angle

                return angle;
            } else {
                return -1;
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
        int  _signature, _x, _y, _width, _height;

        int _buffer[BUFFER_LENGTH];

        int _xC, _yC;
};

#endif
