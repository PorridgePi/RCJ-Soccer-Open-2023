#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>

class Kicker {
    public:
        Kicker(int pin) : _pin(pin) { // constructor
            pinMode(_pin, OUTPUT);
        };
        void kick() {
            digitalWrite(_pin, HIGH);
            delay(100);
            digitalWrite(_pin, LOW);
        };

    private:
        const int _pin;
};

#endif
