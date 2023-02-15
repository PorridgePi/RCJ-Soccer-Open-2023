#include <Arduino.h>

class Temt {
    public:
        Temt(int pin, float x, float y) : _pin(pin), X(x), Y(y) { // constructor
            pinMode(_pin, INPUT);
        };
        int read() {
            return analogRead(_pin);
        };
        const float X;
        const float Y;
    private:
        const int _pin;
};
