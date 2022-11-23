#include "BasePlate.h"

// SEEMS TO BE UNUSED?

BasePlate::BasePlate(int triggerPin) : _triggerPin(triggerPin) {
    // https://stackoverflow.com/questions/2785612/c-what-does-the-colon-after-a-constructor-mean
}

void BasePlate::setup() {
    pinMode(_triggerPin, INPUT);
}

bool BasePlate::boundaryCheck() {
    if (digitalRead(_triggerPin) == HIGH) {
        return true;
    } else {
        return false;
    }
}

// return true when line detected?
