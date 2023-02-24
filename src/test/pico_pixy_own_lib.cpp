#include <Arduino.h>
#include <SoftwareSerial.h>

#define DEBUG_DELAY false

#define PIXY_TX 8
#define PIXY_RX 15

bool newData = false;
String state = "read";

SoftwareSerial PixySerial(PIXY_RX, PIXY_TX);

void setup() {
    Serial.begin(19200);
    PixySerial.begin(19200);
}

int signature, x, y, width, height;

int headers[6];
int buffer[100];

void parseData() {
    // bytes 6-7 = signature
    // bytes 8-9 = x
    // bytes 10-11 = y
    // bytes 12-13 = width
    // bytes 14-15 = height
    if (true || buffer[0] != 175 || buffer[1] != 193 || buffer[2] != 33 || buffer[3] == 0 || buffer[3] % 14 != 0) {
        return;
    } else {
        for (int j = 0; j < 15; j++) {
            Serial.print(buffer[j]);
            Serial.print("\t");
        }
        Serial.println();
    }
    int signature = buffer[6] + buffer[7] * 256;
    int x         = buffer[8] + buffer[9] * 256;
    int y         = buffer[10] + buffer[11] * 256;
    int width     = buffer[12] + buffer[13] * 256;
    int height    = buffer[14] + buffer[15] * 256;
}

void printData() {
    // Serial.print(signature);
    // if (true) {
    // // if (signature == 1) {
    //     Serial.print("\t");
    //     Serial.print(x);
    //     Serial.print("\t");
    //     Serial.print(y);
    //     Serial.print("\t");
    //     Serial.print(width);
    //     Serial.print("\t");
    //     Serial.print(height);
    //     Serial.print("\t");
    // }
    // if (buffer[2] == 33) {
    // for (int j = 0; j < 20; j++) {
    //     Serial.print(buffer[j]);
    //     Serial.print("\t");
    // }
    // Serial.println();
    // }
}

int i = 2;
int r = 0;
int length = 4;

int prevNum = 0;

void parseData2() {
    // bytes 6-7 = signature
    // bytes 8-9 = x
    // bytes 10-11 = y
    // bytes 12-13 = width
    // bytes 14-15 = height
    
    if (buffer[2] == 33 && buffer[6] > 0) {
        // for (int j = 0; j < 20; j++) {
        //     Serial.print(buffer[j]);
        //     Serial.print("\t");
        // }
        // Serial.println();
        digitalWrite(PIN_LED, HIGH);
    }

    // int signature = buffer[6] + buffer[7] * 256;
    // int x         = buffer[8] + buffer[9] * 256;
    // int y         = buffer[10] + buffer[11] * 256;
    // int width     = buffer[12] + buffer[13] * 256;
    // int height    = buffer[14] + buffer[15] * 256;
}

void PixySerialEvent() {
    static boolean recvInProgress = false;
    while (PixySerial.available() > 0 && newData == false) {
        prevNum = r;
        r = PixySerial.read();

        // Serial.print(i);
        // Serial.print('\t');
        // Serial.print(r);
        // Serial.print('\t');
        // Serial.print(prevNum);
        // Serial.println();

        if (recvInProgress == true) {
            if (i == 3) {
                if (r % 14 == 0) {
                    length += r + 2; // account for 2 bytes of checksum
                    constrain(length, 0, 99);
                }
            }
            if (i < length) {
                buffer[i] = r;
                i++;
            } else {
                recvInProgress = false;
                i = 2;
                length = 4;
                newData = true;
            }
        } else if (r == 193 && prevNum == 175) {
            buffer[0] = 175;
            buffer[1] = 193;
            recvInProgress = true;
        }


        // buffer[i] = r;
        // i++;

        // if (r != 175) {
        //     buffer[i] = r;
        //     i++;
        // } else {
        //     int nextR = PixySerial.read();
        //     if (r != 193) {
        //         buffer[i] = r;
        //         buffer[i + 1] = nextR;
        //         i += 2;
        //     } else {
        //         buffer[0] = 175;
        //         buffer[1] = 193;
        //         i = 2;
        //         newData = true;
        //     }
        // }
    }
    // if (PixySerial.available()) {
    //     headers[0] = PixySerial.read();
    // }
    // while (headers[0] != 175) { // 16-bit sync - Bit 0 = 175
    //     if (PixySerial.available()) {
    //         headers[0] = PixySerial.read();
    //     }
    // }
    // for (int i = 1; i < 6; i++) {
    //     if (PixySerial.available()) {
    //         headers[i] = PixySerial.read();
    //     }
    // }

    // int packetType    = headers[2]; // should be 33
    // int payloadLength = headers[3]; // multiple of 14

    // if (headers[1] == 193 && packetType == 33 && payloadLength > 0) { // 16-bit sync - Bit 1 = 193

    //     for (int i = 0; i < 6; i++) {
    //         Serial.print(headers[i]);
    //         Serial.print("\t");
    //     }

    //     for (int i = 0; i < payloadLength + 1; i++) { // +1 to re-run loop
    //         if (i % 14 == 0) {
    //             if (i > 0) {
    //                 newData = true;
    //             }
    //         }
    //         if (i != payloadLength + 1) {
    //             if (PixySerial.available()) {
    //                 block[i % 14] = PixySerial.read();
    //             }
    //         }
    //     }
    // }
}

void loop() {
    long long time = micros();
    uint8_t buf[6] = {174, 193, 32, 2, 255, 255};
    PixySerial.write(buf, 6);

    PixySerialEvent();

    if (newData == true) {
        parseData2();
        // parseData();
        // printData();
        memset(buffer, 0, sizeof(buffer));
        newData = false;
    }

    if (DEBUG_DELAY) {
        delay(10);
    }

    // Serial.print("\t");
    Serial.print((float)(micros()-time)/1000);
    Serial.println("ms");
    digitalWrite(PIN_LED, LOW);
}
