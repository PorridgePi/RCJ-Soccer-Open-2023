#include "Motors.h"

Motor::Motor(int pin1, int pin2) : _pin1(pin1), _pin2(pin2) {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
}

void Motor::move(double spd) {
    if (spd >= 0) {
        analogWrite(_pin1, min(spd, 1) * 255);
        analogWrite(_pin2, 0);
    }
    if (spd < 0) {
        analogWrite(_pin1, 0);
        analogWrite(_pin2, min(abs(spd), 1) * 255);
    }
}
