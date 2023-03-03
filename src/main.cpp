#include <Arduino.h>
#include <Definitions.h>
#include <Wire.h>
#include <Camera.h>

#define DEBUG true
#define DEBUG_PRINT_LIGHT_GATE false
#define DEBUG_PRINT_PIXY false
#define DEBUG_LOOP_TIME false
#define DEBUG_PRINT_TEMT_ANGLE false

Camera Pixy(PIXY_RX, PIXY_TX, 142, 118);

unsigned long loopStartMicros;

unsigned long long prevI2C = 0;
int                temtAngle;

bool isBallInGate() {
    if (analogRead(LIGHT_GATE_PIN) < EMPTY_LIGHT_GATE_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}

void setup() {
    Serial.begin(9600);
    Pixy.begin(19200);
    // I2C connecting to ESP32
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();
}

void readTemtAngle() {
    if (micros() - prevI2C > I2C_READ_ESP32_FREQUENCY * 1000) {
        int data[2];    
        Wire.requestFrom(8, 2);
        if (Wire.available() == 2) {
            data[0] = Wire.read();
            data[1] = Wire.read();
        }
        temtAngle   = data[0] + data[1] * 256;
        prevI2C = micros();
    }
    if (DEBUG_PRINT_TEMT_ANGLE && (millis() % 1000 >= 900)) {
        Serial.print(temtAngle);
        Serial.println();
    }
}

void loop() {
    loopStartMicros = micros();
    if (!USE_MULTICORE) Pixy.readData();
    Pixy.isNewDataPresent();
    readTemtAngle();

    if (DEBUG_PRINT_LIGHT_GATE) {
        Serial.print(isBallInGate()); Serial.print("\t");
        Serial.print(analogRead(LIGHT_GATE_PIN)); Serial.print("\t");
    }
    if (DEBUG_PRINT_PIXY) {
        Serial.print(Pixy.getBallDistance()); Serial.print("\t");
        Serial.print(Pixy.getBallAngle()); Serial.print("\t");
    } 
    if (DEBUG_LOOP_TIME) {
        Serial.print((float) (micros() - loopStartMicros) / 1000);
        Serial.print("\t");
    }
    if (DEBUG_PRINT_LIGHT_GATE || DEBUG_PRINT_PIXY || DEBUG_LOOP_TIME) Serial.println();
    
    if (millis() % 1000 == 0) Serial.println(millis()/1000);
}

void loop1() {
    if (USE_MULTICORE) Pixy.readData();
}
