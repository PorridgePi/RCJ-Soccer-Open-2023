#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Camera.h>

#define USE_MULTICORE true

#define PIXY_TX 8
#define PIXY_RX 15
Camera Pixy(PIXY_RX, PIXY_TX);

void setup() {
    Serial.begin(19200);
    Pixy.begin(19200);
}

void loop() {
    long long time = micros();

    if (! USE_MULTICORE) Pixy.readData();
    
    Pixy.isNewDataPresent();

    Pixy.printData();
    Serial.print((float)(micros()-time)/1000);
    Serial.println("ms");
}

void loop1() {
    if (USE_MULTICORE) Pixy.readData();
}
