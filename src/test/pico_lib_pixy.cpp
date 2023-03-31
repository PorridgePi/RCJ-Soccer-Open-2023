#include <Arduino.h>
#include <Camera2.h>
#include <Definitions.h>

Camera Pixy(PIN_CAM_RX, PIN_CAM_TX_MISO, 139, 104);

void setup() {
    Serial.begin(9600);
    Pixy.begin(19200);
}

void loop() {
    Pixy.step();
}