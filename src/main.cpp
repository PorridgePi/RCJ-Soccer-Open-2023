#include <Arduino.h>
#include <Definitions.h>
#include <Wire.h>
#include <Camera.h>

#define DEBUG true
#define DEBUG_PRINT_LIGHT_GATE false
#define DEBUG_PRINT_PIXY false
#define DEBUG_LOOP_TIME true
#define DEBUG_ON_LINE false

Camera Pixy(PIXY_RX, PIXY_TX, 142, 118);

unsigned long loopStartMicros;

int emptyLightGateThreshold;
int ballAngle;
int ballDistance;

int calibrateLightGate() {
    delay(500);
    int sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(LIGHT_GATE_PIN);
    }
    return sum / 100;
}

bool isBallInGate() {
    if (analogRead(LIGHT_GATE_PIN) < emptyLightGateThreshold - LIGHT_GATE_DIFFERENCE_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}

bool isOnLine() {
    return digitalRead(BOTTOM_PLATE_PIN);
}

void ballTrack() {
    if (ballAngle >= 0 && ballAngle < 90) {

    } else if (ballAngle >= 90 && ballAngle < 180) {

    } else if (ballAngle >= 180 && ballAngle < 270) {

    } else if (ballAngle >= 270 && ballAngle < 360) {

    }
}

void setup() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); // turn on LED to indicate start of setup

    pinMode(BOTTOM_PLATE_PIN, INPUT);
    pinMode(LIGHT_GATE_PIN, INPUT);

    Serial.begin(9600);
    Pixy.begin(19200);

    emptyLightGateThreshold = calibrateLightGate();
    digitalWrite(PIN_LED, LOW); // turn off LED to indicate end of setup
}

void loop() {
    loopStartMicros = micros();

    if (!USE_MULTICORE) Pixy.readData();
    Pixy.isNewDataPresent(); // checks if new data is present and parses it
    ballAngle = Pixy.getBallAngle();
    ballDistance = Pixy.getBallDistance();

    if (DEBUG_ON_LINE) {
        Serial.print(isOnLine()); Serial.print("\t");
    }
    if (DEBUG_PRINT_LIGHT_GATE) {
        Serial.print(emptyLightGateThreshold); Serial.print("\t");
        Serial.print(isBallInGate()); Serial.print("\t");
        Serial.print(analogRead(LIGHT_GATE_PIN)); Serial.print("\t");
    }
    if (DEBUG_PRINT_PIXY) {
        Serial.print(ballDistance); Serial.print("\t");
        Serial.print(ballAngle); Serial.print("\t");
    } 
    if (DEBUG_LOOP_TIME) {
        Serial.print((float) (micros() - loopStartMicros) / 1000);
        Serial.print("\t");
    }
    if (DEBUG_PRINT_LIGHT_GATE || DEBUG_PRINT_PIXY || DEBUG_LOOP_TIME || DEBUG_ON_LINE) Serial.println();
}

void loop1() {
    if (USE_MULTICORE) Pixy.readData();
}
