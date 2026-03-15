#include <Arduino.h>
#define TIP 6
#define RING 5
#define TIMEOUT_RX_US 500
#define TIMEOUT_TX_US 500000
bool error_level;

uint8_t getByte() {
    uint8_t data = 0;
    error_level = 0;
    int time_us = 0;
    for (int i = 0; i < 8; i++) {
        time_us = 0;
        while(digitalRead(TIP) & digitalRead(RING)) {
            if (time_us > TIMEOUT_RX_US) {
                error_level = 1;
                return 0;
            }
            delayMicroseconds(1);
            time_us++;
        }
        time_us = 0;
        if (digitalRead(RING) == 0) {
            data |= 1 << i;
            pinMode(TIP, OUTPUT);
            digitalWrite(TIP, LOW);
            while(!digitalRead(RING)) {
                delayMicroseconds(1);
                time_us++;
            }
            digitalWrite(TIP, HIGH);
            pinMode(TIP, INPUT_PULLUP);
        } else {
            pinMode(RING, OUTPUT);
            digitalWrite(RING, LOW);
            while(!digitalRead(TIP)) {
                delayMicroseconds(1);
                time_us++;
            }
            digitalWrite(RING, HIGH);
            pinMode(RING, INPUT_PULLUP);
        }
    }
    return data;
}

void sendByte(uint8_t data) {
    int time_us;
    error_level = 0;
    for (int i = 0; i < 8; i++) {
        time_us = 0;
        while(!digitalRead(TIP) || !digitalRead(RING)) {
            if (time_us > TIMEOUT_TX_US) {
                error_level = 1;
                return;
            }
            delayMicroseconds(1);
            time_us++;
        }
        time_us = 0;
        if ((data >> i) & 1) {
            pinMode(RING, OUTPUT);
            digitalWrite(RING, LOW);
            while(digitalRead(TIP)) {
                delayMicroseconds(1);
                time_us++;
            }
            pinMode(RING, INPUT_PULLUP);
            time_us = 0;
            while(!digitalRead(TIP)) {
                delayMicroseconds(1);
                time_us++;
            }
        } else {
            pinMode(TIP, OUTPUT);
            digitalWrite(TIP, LOW);
            while(digitalRead(RING)) {
                delayMicroseconds(1);
                time_us++;
            }
            pinMode(TIP, INPUT_PULLUP);
            time_us = 0;
            while(!digitalRead(RING)) {
                delayMicroseconds(1);
                time_us++;
            }
        }
    }
}

void sendShortPacket(uint8_t cmd) {
    sendByte(0x73);
    sendByte(cmd);
    sendByte(0x00);
    sendByte(0x00);
}

void setup() {
    Serial.begin(9600);
    delay(1500);
    pinMode(TIP, INPUT_PULLUP);
    pinMode(RING, INPUT_PULLUP);
    Serial.println("Ready. Send a variable from the calculator.");
}

void loop() {
    static int byteCount = 0;
    static uint16_t expectedBytes = 4;
    static uint8_t lenLo = 0;
    static enum { WAIT_ANNOUNCE, WAIT_VAR_HDR, WAIT_ACK_TO_CTS, WAIT_DATA, WAIT_EOT } state = WAIT_ANNOUNCE;

    uint8_t data = getByte();
    if (!error_level) {
        Serial.print("0x");
        if (data < 0x10) Serial.print("0");
        Serial.println(data, HEX);
        byteCount++;

        if (byteCount == 3) lenLo = data;
        if (byteCount == 4) {
            uint16_t dataLen = (uint16_t)lenLo | ((uint16_t)data << 8);
            if (state == WAIT_ANNOUNCE) {
                expectedBytes = 4; // announce is always 4 bytes
            } else {
                expectedBytes = 4 + dataLen + (dataLen > 0 ? 2 : 0);
            }
            Serial.print("(expecting ");
            Serial.print(expectedBytes);
            Serial.println(" bytes total)");
        }

        if (byteCount == expectedBytes) {
            byteCount = 0;
            expectedBytes = 4;
            lenLo = 0;

            switch (state) {
                case WAIT_ANNOUNCE:
                    Serial.println("Got announce, ACKing");
                    sendShortPacket(0x56);
                    state = WAIT_VAR_HDR;
                    break;

                case WAIT_VAR_HDR:
                    Serial.println("Got VAR header, ACKing + sending CTS");
                    sendShortPacket(0x56);
                    delay(5);
                    sendShortPacket(0x09);
                    state = WAIT_ACK_TO_CTS;
                    break;

                case WAIT_ACK_TO_CTS:
                    Serial.println("Got ACK to our CTS, waiting for data");
                    state = WAIT_DATA;
                    break;

                case WAIT_DATA:
                    Serial.println("Got DATA, ACKing");
                    sendShortPacket(0x56);
                    state = WAIT_EOT;
                    break;

                case WAIT_EOT:
                    Serial.println("Got EOT, ACKing. Transfer complete!");
                    sendShortPacket(0x56);
                    state = WAIT_ANNOUNCE; // reset for next transfer
                    break;
            }
        }
    }
}