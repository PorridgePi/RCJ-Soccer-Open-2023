#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <SerialPIO.h>

#define BUFFER_LENGTH 200
#define BALL_TIMEOUT 10

// Reading speed dependent on loop time

struct Block {
    int  signature, x, y, width, height;
};

class Camera : public SerialPIO {
    public:
        Camera(pin_size_t rx, pin_size_t tx, int xC, int yC) :
            SerialPIO(tx, rx, 512), _xC(xC), _yC(yC) {
            _maxLength = 4;
        }

        bool isNewDataPresent() {
            // static int time;
            if (_newData == true) {
                parseData();
                // for (int i = 0; i < 100; i++) {
                //     Serial.print(_buffer[i]);
                //     Serial.print(" ");
                // }
                // Serial.println();
                memset(_buffer, 0, sizeof(_buffer));
                _maxLength = 4;
                _newData = false;
                // Serial.println(millis() - time);
                // time = millis();
            }
            return _newData;
        }

        void readData() {
            static int  currentByte    = 0;
            static int  prevByte       = 0;
            static int  i              = 2;
            static bool recvInProgress = false;

            static uint8_t writeBuffer[6] = {174, 193, 32, 2, 7, 255};
            if (SerialPIO::available() == 0) SerialPIO::write(writeBuffer, 6);

            while (SerialPIO::available() > 0 && _newData == false) {
                prevByte    = currentByte;
                currentByte = SerialPIO::read();
                Serial.print(currentByte);
                Serial.print(" ");
                if (recvInProgress == true) {
                    if (i == 3) {
                        if (currentByte % 14 == 0) {
                            // CONSTRAIN!
                            _maxLength = constrain(_maxLength + currentByte + 2, 0, BUFFER_LENGTH - 1); // account for 2 bytes of checksum
                        }
                    }
                    if (i < _maxLength) {
                        _buffer[i] = currentByte;
                        i++;
                    } else {
                        recvInProgress = false;
                        i              = 2;
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
                // Serial.print(_maxLength); Serial.print("A\t");
                _numBlocks = (_maxLength - 6) / 14;
                for (int i = 0; i < _numBlocks; i++) {
                    _signature         = constrain(_buffer[i * 14 + 6] + _buffer[i * 14 + 7] * 256, 0, 255);
                    // if (_signature <= 0 || _signature > 3) continue;
                    _blocks[i].signature = _signature;
                    _blocks[i].x = constrain(_buffer[i * 14 + 8] + _buffer[i * 14 + 9] * 256, 0, 315);
                    _blocks[i].y = constrain(_buffer[i * 14 + 10] + _buffer[i * 14 + 11] * 256, 0, 207);
                    _blocks[i].width = constrain(_buffer[i * 14 + 12] + _buffer[i * 14 + 13] * 256, 0, 316);
                    _blocks[i].height = constrain(_buffer[i * 14 + 14] + _buffer[i * 14 + 15] * 256, 0, 208);
                    
                    if (true) {
                        // Serial.print(_blocks[i].signature);
                        // Serial.print("\t");
                        // Serial.print(_blocks[i].x);
                        // Serial.print("\t");
                        // Serial.print(_blocks[i].y)
                        // Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 8]); Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 9]); Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 10]); Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 11]); Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 12]); Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 13]); Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 14]); Serial.print("\t");
                        // Serial.print(_buffer[i * 14 + 15]); Serial.print("\t");
                    }
                }
                Serial.println("");
                _blockLastDetected = 0;
            } else {
                if (_blockLastDetected > 10) { // if no block detected for 10 loops, set values to -1
                    memset(_blocks, 0, sizeof(_blocks));
                }
                _blockLastDetected++;
            }
        }

        int getBallDistance() {
            static unsigned long ballLastDetected = millis();
            static int prevBallDistance = -1;
            for (int i = 0; i < _numBlocks; i++) {
                if (_blocks[i].signature == 1) {
                    ballLastDetected = 0;
                    int xDiff = _blocks[i].x - _xC;
                    int yDiff = _blocks[i].y - _yC;

                    // max distance = 5 * 84 = 420 since max change is around 84 pixels
                    prevBallDistance = constrain(5 * hypot(xDiff, yDiff), 0, 3000);
                    return prevBallDistance;
                }
            }
            if (millis() - ballLastDetected > BALL_TIMEOUT) {
                return -1;
            } else {
                return prevBallDistance;
            }
        }

        int getBallAngle() {
            static unsigned long ballLastDetected = millis();
            static int prevBallAngle = -1;
            for (int i = 0; i < _numBlocks; i++) {
                if (_blocks[i].signature == 1) {
                    int xDiff = _blocks[i].x - _xC;
                    int yDiff = _blocks[i].y - _yC;
                    int angle = atan2(yDiff, xDiff) * 180 / PI + 90;
                    if (angle < 0) angle += 360; // make sure angle is positive
                    angle = 360 - angle; // invert angle
                    prevBallAngle = angle;
                    return angle;
                }
            }
            if (millis() - ballLastDetected > BALL_TIMEOUT) {
                return -1;
            } else {
                return prevBallAngle;
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

        int _maxLength, _numBlocks;
        int _buffer[BUFFER_LENGTH];
        Block _blocks[(int) BUFFER_LENGTH/10];

        int _xC, _yC;
};

#endif
