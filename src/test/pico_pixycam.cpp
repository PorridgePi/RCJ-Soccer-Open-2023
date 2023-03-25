
#include <Arduino.h>
#include <Pixy2UART.h>
Pixy2UART pixy;

void setup() {
    Serial.begin(9600);
    Serial.print("Starting...\n");

    pixy.init();
}

void loop() {
    long long time = micros();

    // grab blocks!
    pixy.ccc.getBlocks();

    // If there are detect blocks, print them!
    if (pixy.ccc.numBlocks) {
        Serial.print("Detected ");
        Serial.println(pixy.ccc.numBlocks);
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            Block block = pixy.ccc.blocks[i];
            Serial.print(" block ");
            Serial.print(i);
            Serial.print(": ");

            // block.print();

            Serial.print("sig: ");
            Serial.print(block.m_signature);
            Serial.print(" x: ");
            Serial.print(block.m_x);
            Serial.print(" y: ");
            Serial.print(block.m_y);
            Serial.print(" width: ");
            Serial.print(block.m_width);
            Serial.print(" height: ");
            Serial.print(block.m_height);
            Serial.print(" angle: ");
            Serial.print(block.m_angle);
            Serial.print(" index: ");
            Serial.print(block.m_index);
            Serial.print(" age: ");
            Serial.print(block.m_age);
            Serial.print('\t');
        }
    }

    Serial.print((float) (micros() - time) / 1000);
    Serial.println("ms");
}

// uint16_t m_signature;
// uint16_t m_x;
// uint16_t m_y;
// uint16_t m_width;
// uint16_t m_height;
// int16_t m_angle;
// uint8_t m_index;
// uint8_t m_age;
