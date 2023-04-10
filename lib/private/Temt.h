#ifndef TEMT_H
#define TEMT_H

#include <Arduino.h>

class Temt {
    public:
        Temt(int pin, float x, float y, int diff) :
            _pin(pin), x(x), y(y), _diff(diff) { // constructor
            pinMode(_pin, INPUT);
        };

        int read() {
            return analogRead(_pin);
        };

        void setThreshold(int threshold) {
            _threshold = threshold;
        };

        bool isOnLine() {
            return read() > (_threshold + _diff);
        };

        void printThreshold() {
            Serial.print(_threshold); Serial.print('\t');
        };

        const float x;
        const float y;

    private:
        const int _pin;
        const int _diff;
        int _threshold;
};

#endif
