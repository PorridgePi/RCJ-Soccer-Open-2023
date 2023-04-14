#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define I2C_ADDRESS_ESP32 8

#define I2C_READ_ESP32_FREQUENCY 1000 // every x ms



// PINS
// Digital
#define PIN_ORI_RESET 0
#define PIN_RELAY 4
#define PIN_BOTPLATE_D1 6
#define PIN_BOTPLATE_D2 10

// I2C
#define PIN_WIRE0_LUNA_SDA 12
#define PIN_WIRE0_LUNA_SCL 13
#define PIN_WIRE1_GY_SDA 2 // Wire1 is used for the GY273 and the Pixy (optional)
#define PIN_WIRE1_GY_SCL 3 // Wire1 is used for the GY273 and the Pixy (optional)

// UART/SPI
#define PIN_BT_TX 17
#define PIN_BT_RX 16
#define PIN_CAM_TX_MISO 8 // Both TX and MISO are on pin 8
#define PIN_CAM_RX 5
#define PIN_CAM_SCK 14
#define PIN_CAM_MOSI 15

// PWM
#define PIN_TOP_A_LPWM 9    // Top Outer Pins       Top Left Motors
#define PIN_TOP_A_RPWM 11   // Top Outer Pins       Top Left Motors
#define PIN_TOP_B_LPWM 1    // Top Inner Pins       Bottom Left Motors
#define PIN_TOP_B_RPWM 7    // Top Inner Pins       Bottom Left Motors
#define PIN_BOT_A_LPWM 22   // Bottom Outer Pins    Bottom Right Motors
#define PIN_BOT_A_RPWM 26   // Bottom Outer Pins    Bottom Right Motors
#define PIN_BOT_B_LPWM 20   // Bottom Inner Pins    Top Right Motors
#define PIN_BOT_B_RPWM 21   // Bottom Inner Pins    Top Right Motors

// Analog
#define PIN_CAM_ANALOG A1 // GP27
#define PIN_BALL_CAP_ANALOG A2 // GP28

// I2C Addresses
#define GY273_ADDRESS 0x1E

#endif
