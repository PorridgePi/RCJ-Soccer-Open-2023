#ifndef BASEPLATE_h
#define BASEPLATE_h

#include <Arduino.h>

// SEEMS TO BE UNUSED?

class BasePlate { // ADD KICKER STUFF
    private:
        int _triggerPin;

    public:
        BasePlate(int triggerPin);
        void setup();
        bool boundaryCheck();
};

#endif
