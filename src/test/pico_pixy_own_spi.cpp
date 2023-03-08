#include <Arduino.h>
#include <SPI.h>

#define SPI1_MISO 8
#define SPI1_MOSI 15
#define SPI1_SCLK 14

// arduino::MbedSPI SPI1(SPI1_MISO, SPI1_MOSI, SPI1_SCLK); // works with official raspberrypi core

int object[5] = {};

void setup() {
    Serial.begin(19200);
    SPI1.setRX(SPI1_MISO);
    SPI1.setTX(SPI1_MOSI);
    SPI1.setSCK(SPI1_SCLK);
    SPI1.begin();
    pinMode(25, OUTPUT);
}

void loop() {
    long long time = millis();
    SPI1.beginTransaction(SPISettings(2 * MHZ, MSBFIRST, SPI_MODE3));
    int tx_buffer[20] = {174, 193, 32, 2, 1, 1};
    int rx_buffer[20] = {};
    for (int i = 0; i < 6; i++) {
        SPI1.transfer(tx_buffer[i]);
    }
    delayMicroseconds(20000);
    while (rx_buffer[2] != 33) {
        rx_buffer[2] = SPI1.transfer(0);
    }
    for (int i = 3; i < 20; i++) {
        rx_buffer[i] = SPI1.transfer(0);
    }
    SPI1.endTransaction();

    object[0] = rx_buffer[6] + rx_buffer[7] * 256;
    object[1] = rx_buffer[8] + rx_buffer[9] * 256;
    object[2] = rx_buffer[10] + rx_buffer[11] * 256;
    object[3] = rx_buffer[12] + rx_buffer[13] * 256;
    object[4] = rx_buffer[14] + rx_buffer[15] * 256;

    if (object[0] == 1) {
        for (int n = 0; n < 5; n++) {
            Serial.print(object[n]);
            Serial.print(" ");
        }
        // turn led on
        digitalWrite(PIN_LED, HIGH);

    } else {
        // turn led off
        digitalWrite(PIN_LED, LOW);
    }
    Serial.println(millis() - time);
}
