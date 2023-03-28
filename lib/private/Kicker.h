#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>

class Kicker {
    public:
        Kicker(int pin) :
            _pin(pin) { // constructor
            pinMode(_pin, OUTPUT);
        };
        void kick() {
            digitalWrite(_pin, LOW);
            delay(50);
            digitalWrite(_pin, HIGH);
        };

    private:
        const int _pin;
};

#endif
