#include <Arduino.h>
#define TIP 6
#define RING 5
#define TIMEOUT_RX_US 500
#define TIMEOUT_TX_US 500000
#define BOOT_PIN 9

uint8_t recvBuf[256];
uint16_t recvBufLen = 0;
uint8_t recvVarType = 0;

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
        Serial.println("Auto mode - sending immediately...");
        delay(100);
    }

    uint16_t strLen = strlen(str);
    uint16_t dataSize = 2 + strLen;

    uint8_t payload[256];
    payload[0] = strLen & 0xFF;
    payload[1] = (strLen >> 8) & 0xFF;
    for (uint16_t i = 0; i < strLen; i++) {
        payload[2 + i] = str[i];
    }

    uint8_t header[13] = {0};
    header[0] = dataSize & 0xFF;
    header[1] = (dataSize >> 8) & 0xFF;
    header[2] = 0x04;
    header[3] = 0xAA;
    header[4] = strSlot;

    Serial.println("Sending VAR header...");
    sendPacket(0x06, header, 13);
    delay(10);

    uint8_t cmd = recvShortPacket();
    Serial.print("Step2 ACK: 0x"); Serial.println(cmd, HEX);
    if (cmd != 0x56) { Serial.println("Expected ACK, aborting"); return; }

    cmd = recvShortPacketWait();
    Serial.print("Step3 CTS: 0x"); Serial.println(cmd, HEX);
    if (cmd != 0x09) { Serial.println("Expected CTS, aborting"); return; }

    sendShortPacket(0x56);
    delay(10);

    Serial.println("Sending data...");
    sendPacket(0x15, payload, dataSize);
    delay(10);

    cmd = recvShortPacketWait();
    Serial.print("Step6 ACK: 0x"); Serial.println(cmd, HEX);
    if (cmd != 0x56) { Serial.println("Expected ACK to data, aborting"); return; }

    sendShortPacket(0x92);
    delay(10);

    cmd = recvShortPacketWait();
    Serial.print("Step8 ACK: 0x"); Serial.println(cmd, HEX);

    Serial.println("Done!");
}

void setup() {
    Serial.begin(9600);
    delay(1500);
    pinMode(TIP, INPUT_PULLUP);
    pinMode(RING, INPUT_PULLUP);
    pinMode(BOOT_PIN, INPUT_PULLUP);
    Serial.println("Ready.");
    Serial.println("Press BOOT to send string to calculator.");
    Serial.println("Or run ESPREQ program on calculator.");
}

void loop() {
    static int byteCount = 0;
    static uint16_t expectedBytes = 4;
    static uint8_t lenLo = 0;
    static enum {
        WAIT_PACKET,    // waiting for any packet
        WAIT_DATA,      // waiting for data after CTS exchange
        WAIT_EOT        // waiting for EOT after data
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
            if (state == WAIT_EOT) {
                expectedBytes = 4;
            } else if (
                // These are always 4 bytes regardless of length field
                (recvBuf[0] == 0x83 && recvBuf[1] == 0x68) ||  // manual announce
                (recvBuf[1] == 0x56) ||                          // ACK
                (recvBuf[1] == 0x09) ||                          // CTS
                (recvBuf[1] == 0x92)                             // EOT
            ) {
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

            uint8_t machineId = recvBuf[0];
            uint8_t cmd       = recvBuf[1];

            switch (state) {
                case WAIT_PACKET:
                    if (cmd == 0xC9) {
                        // Calc sending us a variable via Send()
                        // Just ACK, then wait for calc to send CTS
                        Serial.println("Got RTS, ACKing, waiting for CTS from calc");
                        recvVarType = recvBuf[6];
                        sendShortPacket(0x56);  // ACK only — do NOT send CTS
                        // stay in WAIT_PACKET, next packet should be CTS from calc
                    } else if (cmd == 0x09) {
                        // CTS from calc — ACK it and wait for data
                        Serial.println("Got CTS from calc, ACKing, waiting for data");
                        sendShortPacket(0x56);
                        state = WAIT_DATA;
                    } else if (cmd == 0xA2) {
                        // REQ — calc requesting a variable from us
                        Serial.println("Got REQ (calc wants Str1), sending it");
                        sendShortPacket(0x56);  // ACK the REQ
                        delay(5);
                        // Send VAR header for Str1
                        const char *response = "RESPONSE";
                        uint16_t strLen = strlen(response);
                        uint16_t dataSize = 2 + strLen;
                        uint8_t header[11] = {0};
                        header[0] = dataSize & 0xFF;
                        header[1] = (dataSize >> 8) & 0xFF;
                        header[2] = 0x04;   // string type
                        header[3] = 0xAA;   // Str token
                        header[4] = 0x00;   // Str1 is internally slot 0
                        sendPacket(0x06, header, 11);
                        delay(10);
                        // Wait for ACK then CTS
                        uint8_t r = recvShortPacket();
                        Serial.print("ACK to header: 0x"); Serial.println(r, HEX);
                        r = recvShortPacketWait();
                        Serial.print("CTS: 0x"); Serial.println(r, HEX);
                        // ACK the CTS
                        sendShortPacket(0x56);
                        delay(5);
                        // Send data
                        uint8_t payload[256];
                        payload[0] = strLen & 0xFF;
                        payload[1] = (strLen >> 8) & 0xFF;
                        for (uint16_t i = 0; i < strLen; i++) payload[2 + i] = response[i];
                        sendPacket(0x15, payload, dataSize);
                        delay(10);
                        // Wait for ACK
                        r = recvShortPacketWait();
                        Serial.print("ACK to data: 0x"); Serial.println(r, HEX);
                        // Send EOT
                        sendShortPacket(0x92);
                        Serial.println("Str1 sent!");
                    } else if (cmd == 0x68) {
                        // Manual announce
                        Serial.println("Got manual announce, ACKing");
                        sendShortPacket(0x56);
                        // stay in WAIT_PACKET, VAR header comes next
                    } else if (cmd == 0x06) {
                        // Manual VAR header
                        Serial.println("Got VAR header, ACKing + CTS");
                        recvVarType = recvBuf[6];
                        sendShortPacket(0x56);
                        delay(5);
                        sendShortPacket(0x09);
                        state = WAIT_DATA;
                    } else {
                        Serial.print("Unknown packet cmd: 0x");
                        Serial.println(cmd, HEX);
                    }
                    break;

                case WAIT_DATA:
                    if (cmd == 0x56) {
                        // ACK to our CTS — data coming next, stay in WAIT_DATA
                        Serial.println("Got ACK to CTS, data incoming");
                    } else if (cmd == 0x15) {
                        // DATA packet
                        Serial.println("Got DATA, ACKing");
                        sendShortPacket(0x56);
                        if (recvVarType == 0x04) {
                            uint16_t strLen = recvBuf[4] | (recvBuf[5] << 8);
                            Serial.print("String value: ");
                            for (uint16_t i = 0; i < strLen; i++) {
                                Serial.print((char)recvBuf[6 + i]);
                            }
                            Serial.println();
                        }
                        state = WAIT_EOT;
                    }
                    break;

                case WAIT_EOT:
                    Serial.println("Got EOT, ACKing. Receive complete!");
                    sendShortPacket(0x56);
                    state = WAIT_PACKET;
                    break;
            }
        }
    }

    if (digitalRead(BOOT_PIN) == LOW) {
        delay(50);
        if (digitalRead(BOOT_PIN) == LOW) {
            sendString("SUBSCRIBE", 1, false);
            while (digitalRead(BOOT_PIN) == LOW) delay(10);
        }
    }
}