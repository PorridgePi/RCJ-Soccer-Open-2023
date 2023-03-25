#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define I2C_ADDRESS_ESP32 8

#define I2C_READ_ESP32_FREQUENCY 1000 // every x ms

// Thresholds
#define LIGHT_GATE_DIFFERENCE_THRESHOLD 100

// Pins
#define LIGHT_GATE_PIN   A2
#define PIXY_TX          8
#define PIXY_RX          15
#define BOTTOM_PLATE_PIN 1

// I2C Addresses
#define GY273_ADDRESS 0x1E

#endif
