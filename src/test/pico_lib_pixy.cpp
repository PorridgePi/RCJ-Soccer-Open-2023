#include <Arduino.h>
#include <Camera2.h>
#include <Definitions.h>

Camera Pixy(PIXY_RX, PIXY_TX, 139, 104);

void setup() {
    Serial.begin(9600);
    Pixy.begin(19200);
}

void loop() {
    Pixy.step();
}