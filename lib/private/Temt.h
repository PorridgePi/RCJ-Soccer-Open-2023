#include <Arduino.h>

class Temt {
    public:
        Temt(int pin) : _pin(pin) { // constructor
            pinMode(_pin, INPUT);
        };
        int read() {
            return analogRead(_pin);
        };
    private:
        const int _pin;
};
