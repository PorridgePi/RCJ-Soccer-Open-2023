// Adapted from https://playground.arduino.cc/Main/I2cScanner/

#include <Arduino.h>
#include <Definitions.h>
#include <Wire.h>

TwoWire &_wire = Wire1;

void setup() {
    Wire.setSCL(PIN_WIRE0_LUNA_SCL);
    Wire.setSDA(PIN_WIRE0_LUNA_SDA);
    Wire.setClock(400000);
    Wire.setTimeout(1); // set timeout to 1 ms
    Wire.begin();

    Wire1.setSCL(PIN_WIRE1_GY_SCL);
    Wire1.setSDA(PIN_WIRE1_GY_SDA);
    Wire1.setTimeout(1); // set timeout to 1 ms
    Wire1.begin();

    Serial.begin(9600);
    while (!Serial);
    Serial.println("\nI2C Scanner");
}

void loop() {
    byte error, address;
    int  nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        _wire.beginTransmission(address);
        error = _wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(5000); // wait 5 seconds for next scan
}
