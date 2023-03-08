#include <Arduino.h>
#include <SoftwareSerial.h>

#define USE_MULTICORE false

#define DEBUG_DELAY false
#define DEBUG_LED   false

#define BUFFER_LENGTH 50

#define PIXY_TX 8
#define PIXY_RX 15
SoftwareSerial PixySerial(PIXY_RX, PIXY_TX);

bool newData        = false;
bool recvInProgress = false;

int signature, x, y, width, height;

int i                 = 2;
int r                 = 0;
int length            = 4;
int prevNum           = 0;
int blockLastDetected = 0;

int buffer[BUFFER_LENGTH];

void setup() {
    Serial.begin(19200);
    PixySerial.begin(19200);
}

void printData() {
    Serial.print(signature);
    Serial.print("\t");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(width);
    Serial.print("\t");
    Serial.print(height);
    Serial.print("\t");
}

void pixyParseData() {
    // bytes 6-7 = signature
    // bytes 8-9 = x
    // bytes 10-11 = y
    // bytes 12-13 = width
    // bytes 14-15 = height

    if (buffer[2] == 33 && buffer[6] > 0) {
        signature = buffer[6] + buffer[7] * 256;
        x         = buffer[8] + buffer[9] * 256;
        y         = buffer[10] + buffer[11] * 256;
        width     = buffer[12] + buffer[13] * 256;
        height    = buffer[14] + buffer[15] * 256;
        if (DEBUG_LED) digitalWrite(PIN_LED, HIGH);
        blockLastDetected = 0;
    } else {
        if (blockLastDetected > 10) { // if no block detected for 10 loops, set values to -1
            signature = -1;
            x         = -1;
            y         = -1;
            width     = -1;
            height    = -1;
        }
        blockLastDetected++;
    }
}

void pixyReadData() {
    uint8_t buf[6] = {174, 193, 32, 2, 255, 255};
    PixySerial.write(buf, 6);

    while (PixySerial.available() > 0 && newData == false) {
        prevNum = r;
        r       = PixySerial.read();

        if (recvInProgress == true) {
            if (i == 3) {
                if (r % 14 == 0) {
                    length += r + 2; // account for 2 bytes of checksum
                    constrain(length, 0, BUFFER_LENGTH - 1);
                }
            }
            if (i < length) {
                buffer[i] = r;
                i++;
            } else {
                recvInProgress = false;
                i              = 2;
                length         = 4;
                newData        = true;
            }
        } else if (r == 193 && prevNum == 175) {
            buffer[0]      = 175;
            buffer[1]      = 193;
            recvInProgress = true;
        }
    }
}

void loop() {
    long long time = micros();

    if (!USE_MULTICORE) pixyReadData();
    if (newData == true) {
        pixyParseData();
        memset(buffer, 0, sizeof(buffer));
        newData = false;
    }

    if (DEBUG_DELAY) {
        delay(10);
    }

    printData();
    Serial.print((float) (micros() - time) / 1000);
    Serial.println("ms");
    if (DEBUG_LED) digitalWrite(PIN_LED, LOW);
}

void loop1() {
    if (USE_MULTICORE) {
        pixyReadData();
    }
}
