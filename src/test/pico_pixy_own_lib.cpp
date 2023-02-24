#include <Arduino.h>
#include <SoftwareSerial.h>

#define DEBUG_DELAY false
#define DEBUG_LED false

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

void printData() {

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
        if (DEBUG_LED) digitalWrite(PIN_LED, HIGH);
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
    }
}

void loop() {
    long long time = micros();
    uint8_t buf[6] = {174, 193, 32, 2, 255, 255};
    PixySerial.write(buf, 6);

    PixySerialEvent();

    if (newData == true) {
        parseData2();
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
    if (DEBUG_LED) digitalWrite(PIN_LED, LOW);
}
