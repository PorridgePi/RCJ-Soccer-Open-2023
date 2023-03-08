#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define USE_MULTICORE true // for fast SoftwareSerial

#define I2C_ADDRESS_ESP32 8

#define I2C_READ_ESP32_FREQUENCY 1000 // every x ms

// Thresholds
#define EMPTY_LIGHT_GATE_THRESHOLD 200

// Pins
#define LIGHT_GATE_PIN A2
#define PIXY_TX 8
#define PIXY_RX 15

#endif
