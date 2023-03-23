#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define READING_TIMEOUT 10
#define BALL_TIMEOUT 10

// Reading speed dependent on loop time

struct Block {
    int  signature, x, y, width, height;
};

class Camera : public SoftwareSerial {
    public:
        Camera(pin_size_t rx, pin_size_t tx, int xC, int yC) :
            SoftwareSerial(rx, tx), _xC(xC), _yC(yC) {
            SoftwareSerial::setTimeout(1);
            //_maxLength = 4;
        }

        bool isNewDataPresent() {
            // static int time;
            if (_newData == true) {
                //parseData();
                memset(_header, 0, sizeof(_header));
                _i = 0;
                _newData = false;
                // Serial.println(millis() - time);
                // time = millis();
            }
            return _newData;
        }

        void step() {//Run this every loop cycle; reads 1 byte from the data stream;
            static uint8_t writeBuffer[6] = {174, 193, 32, 2, 255, 255};
            static int bytesToRead = SoftwareSerial::available();
            if (bytesToRead == 0) SoftwareSerial::write(writeBuffer, 6);
            _timeAtLastStep = millis();
                Serial.print(bytesToRead);
                Serial.print("\t");
            SoftwareSerial::readBytes(_header, 6) ;
            for (size_t i; i < 6; i++) {
                Serial.print(_header[i]);
                Serial.print("\t");
            }
            Serial.println();
            if (_header[0] != 175 || _header[1] != 193 || _header[2] != 14) {// IF HEADER IS WRONG CLEAR THE BUFFER
                //memset(_header, 0, sizeof(_header));

                //Serial.println("Clearing buffer");
                while (SoftwareSerial::available()) {SoftwareSerial::read();} // How slow is this?
                return;
            } else { //IF THE HEADER IS CORRECT:
                _numBlocks = (int)_header[3] / 14;
                static char packet[14];
                for (static size_t i=0; i < _numBlocks; i++) {
                    memset(packet,0,sizeof(packet));
                    SoftwareSerial::readBytes(packet, 14);
                    Serial.println((int)(packet[0] << 8 | packet[1]));
                    //Serial.print("\t");
                }
            }
            Serial.print("t=");
            Serial.print(millis()-_timeAtLastStep);
            Serial.println();
        }
        /*
        void parseData() {
            static int _blockLastDetected = 0;

            // bytes 6-7 = signature
            // bytes 8-9 = x
            // bytes 10-11 = y
            // bytes 12-13 = width
            // bytes 14-15 = height

            if (_buffer[2] == 33 && _buffer[6] > 0) {
                Serial.print(_maxLength); Serial.print("A\t");
                _numBlocks = (_maxLength - 6) / 14;
                for (int i = 0; i < _numBlocks; i++) {
                    _signature         = constrain(_buffer[i * 14 + 6] + _buffer[i * 14 + 7] * 256, 0, 255);
                    if (_signature <= 0 || _signature > 3) continue;
                    _blocks[i].signature = _signature;
                    _blocks[i].x = constrain(_buffer[i * 14 + 8] + _buffer[i * 14 + 9] * 256, 0, 315);
                    _blocks[i].y = constrain(_buffer[i * 14 + 10] + _buffer[i * 14 + 11] * 256, 0, 207);
                    _blocks[i].width = constrain(_buffer[i * 14 + 12] + _buffer[i * 14 + 13] * 256, 0, 316);
                    _blocks[i].height = constrain(_buffer[i * 14 + 14] + _buffer[i * 14 + 15] * 256, 0, 208);
                    if (false) {
                        Serial.print(_blocks[i].signature);
                        Serial.print("\t");
                        Serial.print(_blocks[i].x);
                        Serial.print("\t");
                        Serial.print(_blocks[i].y);
                        Serial.print("\t");
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
        */
    private:


        static int  _i;
        unsigned long _timeAtLastStep;
        bool _newData = false;
        int  _signature, _x, _y, _width, _height;

        size_t _numBlocks;
        char _header[6];
        //Block _blocks[(int) BUFFER_LENGTH/10];

        int _xC, _yC;
};

#endif
