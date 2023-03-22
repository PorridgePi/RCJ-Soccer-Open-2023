#include <Arduino.h>
#include <Camera.h>
#include <SoftwareSerial.h>

#define USE_MULTICORE true

#define PIXY_TX 8
#define PIXY_RX 15
Camera Pixy(PIXY_RX, PIXY_TX, 142, 118);

void setup() {
    Serial.begin(19200);
    Pixy.begin(19200);
    pinMode(PIN_LED, OUTPUT);
}

void loop() {
    long long time = micros();
    static unsigned long lastMicros = 0;
    static bool ledState = false;

    if (!USE_MULTICORE) Pixy.readData();

    Pixy.isNewDataPresent();

    // Pixy.printData();
    Serial.print(Pixy.getBallDistance());
    Serial.print("\t");
    Serial.print(Pixy.getBallAngle());
    Serial.print("\t");
    Serial.print((float) (micros() - time) / 1000);
    Serial.println("ms");

    if (micros() - lastMicros >= 50 * 1000) {
        lastMicros = micros();
        if (ledState) {
            digitalWrite(PIN_LED, LOW);
            ledState = false;
        } else {
            digitalWrite(PIN_LED, HIGH);
            ledState = true;
        }
    }
}

void loop1() {
    if (USE_MULTICORE) Pixy.readData();
}
