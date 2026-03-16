#include <Arduino.h>
#define TIP 6
#define RING 5
#define TIMEOUT_RX_US 500
#define TIMEOUT_TX_US 500000
#define BOOT_PIN 9

uint8_t recvBuf[256];
uint16_t recvBufLen = 0;
uint8_t recvVarType = 0;
bool pendingSendProgram = false;

bool error_level;

const uint8_t espreqProgram[] = {
    0xE1,                                                     // ClrHome
    0x3F,                                                     // newline
    0xDC,                                                     // Input
    0x2A,                                                     // "
    'I','N','P','U','T',0x29,'Q','U','E','R','Y',':',         // INPUT QUERY: (0x29=space)
    0x2A,                                                     // "
    0x2B,                                                     // ,
    0xAA, 0x09,                                               // Str0
    0x3F,                                                     // newline
    0xE1,                                                     // ClrHome
    0x3F,                                                     // newline
    0xDE,                                                     // Disp
    0x2A,                                                     // "
    'W','A','I','T','I','N','G','.','.','.',                   // WAITING...
    0x2A,                                                     // "
    0x3F,                                                     // newline
    0xE7, 0xAA, 0x09, 0x11,                                   // Send(Str0)
    0x3F,                                                     // newline
    0xBB, 0x53, 0xAA, 0x00, 0x11,                            // GetCalc(Str1)
    0x3F,                                                     // newline
    0xE1,                                                     // ClrHome
    0x3F,                                                     // newline
    0xDE, 0xAA, 0x00                                          // Disp Str1
};
const uint16_t espreqProgramSize = sizeof(espreqProgram);
const char espreqName[] = "TIGPT";

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

void sendPacket(uint8_t cmd, const uint8_t *data, uint16_t length) {
    sendByte(0x73);
    sendByte(cmd);
    sendByte(length & 0xFF);
    sendByte((length >> 8) & 0xFF);

    uint16_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sendByte(data[i]);
        checksum += data[i];
    }

    if (length > 0) {
        sendByte(checksum & 0xFF);
        sendByte((checksum >> 8) & 0xFF);
    }
}

uint8_t recvShortPacket() {
    uint8_t b0 = getByte();
    uint8_t b1 = getByte();
    uint8_t b2 = getByte();
    uint8_t b3 = getByte();
    Serial.print("  recv: 0x"); Serial.print(b0, HEX);
    Serial.print(" 0x"); Serial.print(b1, HEX);
    Serial.print(" 0x"); Serial.print(b2, HEX);
    Serial.print(" 0x"); Serial.println(b3, HEX);
    return b1;
}

uint8_t recvShortPacketWait() {
    uint32_t start = millis();
    while (digitalRead(TIP) && digitalRead(RING)) {
        if (millis() - start > 30000) {
            Serial.println("Timed out waiting for response");
            return 0;
        }
    }
    uint8_t b0 = getByte();
    uint8_t b1 = getByte();
    uint8_t b2 = getByte();
    uint8_t b3 = getByte();
    Serial.print("  recv: 0x"); Serial.print(b0, HEX);
    Serial.print(" 0x"); Serial.print(b1, HEX);
    Serial.print(" 0x"); Serial.print(b2, HEX);
    Serial.print(" 0x"); Serial.println(b3, HEX);
    return b1;
}

void sendString(const char *str, uint8_t strSlot, bool autoMode = false) {
    Serial.println("\nSending string to calculator...");

    if (!autoMode) {
        Serial.println("Put calc in receive mode: 2nd > LINK > RECEIVE, then press ENTER");
        delay(5000);
    } else {
        delay(100);
    }

    uint16_t strLen = strlen(str);
    uint16_t dataSize = 2 + strLen;

    uint8_t payload[256];
    payload[0] = strLen & 0xFF;
    payload[1] = (strLen >> 8) & 0xFF;
    for (uint16_t i = 0; i < strLen; i++) payload[2 + i] = str[i];

    uint8_t header[13] = {0};
    header[0] = dataSize & 0xFF;
    header[1] = (dataSize >> 8) & 0xFF;
    header[2] = 0x04;
    header[3] = 0xAA;
    header[4] = strSlot;

    sendPacket(0x06, header, 13);
    delay(10);

    uint8_t cmd = recvShortPacket();
    if (cmd != 0x56) { Serial.println("Expected ACK, aborting"); return; }

    cmd = recvShortPacketWait();
    if (cmd != 0x09) { Serial.println("Expected CTS, aborting"); return; }

    sendShortPacket(0x56);
    delay(10);

    sendPacket(0x15, payload, dataSize);
    delay(10);

    cmd = recvShortPacketWait();
    if (cmd != 0x56) { Serial.println("Expected ACK to data, aborting"); return; }

    sendShortPacket(0x92);
    delay(10);

    recvShortPacketWait();
    Serial.println("String sent!");
}

void sendProgram() {
    Serial.println("\nSending TIGPT program...");
    Serial.println("Put calc in receive mode: 2nd > LINK > RECEIVE");
    Serial.println("You have 10 seconds...");
    delay(10000);

    uint16_t dataSize = 2 + espreqProgramSize;
    uint8_t payload[512];
    payload[0] = espreqProgramSize & 0xFF;
    payload[1] = (espreqProgramSize >> 8) & 0xFF;
    memcpy(&payload[2], espreqProgram, espreqProgramSize);

    uint8_t header[13] = {0};
    header[0] = dataSize & 0xFF;
    header[1] = (dataSize >> 8) & 0xFF;
    header[2] = 0x05;
    memcpy(&header[3], espreqName, strlen(espreqName));

    Serial.println("Sending program header...");
    sendPacket(0x06, header, 13);
    delay(10);

    uint8_t r = recvShortPacket();
    Serial.print("ACK: 0x"); Serial.println(r, HEX);
    if (r != 0x56) { Serial.println("Expected ACK, aborting"); return; }

    r = recvShortPacketWait();
    Serial.print("CTS: 0x"); Serial.println(r, HEX);
    if (r != 0x09) { Serial.println("Expected CTS, aborting"); return; }

    sendShortPacket(0x56);
    delay(10);

    Serial.println("Sending program data...");
    sendPacket(0x15, payload, dataSize);
    delay(10);

    r = recvShortPacketWait();
    Serial.print("ACK to data: 0x"); Serial.println(r, HEX);
    if (r != 0x56) { Serial.println("Expected ACK, aborting"); return; }

    sendShortPacket(0x92);
    delay(10);

    recvShortPacketWait();
    Serial.println("TIGPT program sent!");
}

void setup() {
    Serial.begin(9600);
    delay(1500);
    pinMode(TIP, INPUT_PULLUP);
    pinMode(RING, INPUT_PULLUP);
    pinMode(BOOT_PIN, INPUT_PULLUP);
    Serial.println("Ready.");
    Serial.println("Press BOOT to send TIGPT program to calculator.");
    Serial.println("Or send A=69 from calculator to trigger silent program send.");
}

void loop() {
    static int byteCount = 0;
    static uint16_t expectedBytes = 4;
    static uint8_t lenLo = 0;
    static enum {
        WAIT_PACKET,
        WAIT_DATA,
        WAIT_EOT
    } state = WAIT_PACKET;

    uint8_t data = getByte();
    if (!error_level) {
        recvBuf[byteCount] = data;
        Serial.print("0x");
        if (data < 0x10) Serial.print("0");
        Serial.println(data, HEX);
        byteCount++;

        if (byteCount == 3) lenLo = data;
        if (byteCount == 4) {
            uint16_t dataLen = (uint16_t)lenLo | ((uint16_t)data << 8);
            if (state == WAIT_EOT ||
                (recvBuf[0] == 0x83 && recvBuf[1] == 0x68) ||
                recvBuf[1] == 0x56 ||
                recvBuf[1] == 0x09 ||
                recvBuf[1] == 0x92) {
                expectedBytes = 4;
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

            uint8_t cmd = recvBuf[1];

            switch (state) {
                case WAIT_PACKET:
                    if (cmd == 0xC9) {
                        Serial.println("Got RTS, ACKing");
                        recvVarType = recvBuf[6];
                        sendShortPacket(0x56);
                    } else if (cmd == 0x09) {
                        Serial.println("Got CTS, ACKing, waiting for data");
                        sendShortPacket(0x56);
                        state = WAIT_DATA;
                    } else if (cmd == 0xA2) {
                        Serial.println("Got REQ, sending Str1 response");
                        sendShortPacket(0x56);
                        delay(5);
                        const char *response = "RESPONSE";
                        uint16_t strLen = strlen(response);
                        uint16_t dataSize = 2 + strLen;
                        uint8_t header[11] = {0};
                        header[0] = dataSize & 0xFF;
                        header[1] = (dataSize >> 8) & 0xFF;
                        header[2] = 0x04;
                        header[3] = 0xAA;
                        header[4] = 0x00;
                        sendPacket(0x06, header, 11);
                        delay(10);
                        uint8_t r = recvShortPacket();
                        Serial.print("ACK to header: 0x"); Serial.println(r, HEX);
                        r = recvShortPacketWait();
                        Serial.print("CTS: 0x"); Serial.println(r, HEX);
                        sendShortPacket(0x56);
                        delay(5);
                        uint8_t payload[256];
                        payload[0] = strLen & 0xFF;
                        payload[1] = (strLen >> 8) & 0xFF;
                        for (uint16_t i = 0; i < strLen; i++) payload[2 + i] = response[i];
                        sendPacket(0x15, payload, dataSize);
                        delay(10);
                        r = recvShortPacketWait();
                        Serial.print("ACK to data: 0x"); Serial.println(r, HEX);
                        sendShortPacket(0x92);
                        Serial.println("Response sent!");
                    } else if (cmd == 0x68) {
                        Serial.println("Got manual announce, ACKing");
                        sendShortPacket(0x56);
                    } else if (cmd == 0x06) {
                        Serial.println("Got VAR header, ACKing + CTS");
                        recvVarType = recvBuf[6];
                        sendShortPacket(0x56);
                        delay(5);
                        sendShortPacket(0x09);
                        state = WAIT_DATA;
                    } else {
                        Serial.print("Unknown cmd: 0x");
                        Serial.println(cmd, HEX);
                    }
                    break;

                case WAIT_DATA:
                    if (cmd == 0x56) {
                        Serial.println("Got ACK to CTS, data incoming");
                    } else if (cmd == 0x15) {
                        Serial.println("Got DATA, ACKing");
                        sendShortPacket(0x56);
                        if (recvVarType == 0x04) {
                            uint16_t strLen = recvBuf[4] | (recvBuf[5] << 8);
                            char receivedStr[256] = {0};
                            for (uint16_t i = 0; i < strLen; i++) {
                                receivedStr[i] = recvBuf[6 + i];
                            }
                            Serial.print("String value: ");
                            Serial.println(receivedStr);
                            if (strcmp(receivedStr, "69") == 0) {
                                Serial.println("Magic string! Will send program after EOT.");
                                pendingSendProgram = true;
                            }
                        }
                        if (recvVarType == 0x00) {
                            if (recvBuf[5] == 0x81 && recvBuf[6] == 0x69) {
                                Serial.println("Magic number 69! Will send program after EOT.");
                                pendingSendProgram = true;
                            }
                        }
                        state = WAIT_EOT;
                    }
                    break;

                case WAIT_EOT:
                    Serial.println("Got EOT, ACKing.");
                    sendShortPacket(0x56);
                    state = WAIT_PACKET;
                    if (pendingSendProgram) {
                        pendingSendProgram = false;
                        sendProgram();
                    }
                    break;
            }
        }
    }

    if (digitalRead(BOOT_PIN) == LOW) {
        delay(50);
        if (digitalRead(BOOT_PIN) == LOW) {
            sendProgram();
            while (digitalRead(BOOT_PIN) == LOW) delay(10);
        }
    }
}