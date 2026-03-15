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
    // Wait up to 30 seconds for first byte (user is interacting with calc)
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

void sendVariable() {
    Serial.println("\nSending variable to calculator...");
    Serial.println("Put calc in receive mode: 2nd > LINK > RECEIVE, then press ENTER");
    delay(5000);

    const uint8_t varData[] = {
    0x00, 0x83, 0x12, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00  // just the 9 float bytes
    };
    const uint16_t varDataSize = sizeof(varData);  // = 9

    uint8_t header[13] = {
        0x09, 0x00,
        0x00,
        'B', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
        0x00
    };

    // Step 1: Send VAR header
    Serial.println("Sending VAR header...");
    sendPacket(0x06, header, 13);
    delay(10);

    // Step 2: Receive ACK
    uint8_t cmd = recvShortPacket();
    Serial.print("Step2 ACK: 0x"); Serial.println(cmd, HEX);
    if (cmd != 0x56) { Serial.println("Expected ACK, aborting"); return; }

    // Step 3: Receive CTS (calc may show overwrite screen here, wait for user)
    cmd = recvShortPacketWait();
    Serial.print("Step3 CTS: 0x"); Serial.println(cmd, HEX);
    if (cmd != 0x09) { Serial.println("Expected CTS, aborting"); return; }

    // Step 4: ACK the CTS
    sendShortPacket(0x56);
    delay(10);

    // Step 5: Send DATA
    Serial.println("Sending data...");
    sendPacket(0x15, varData, varDataSize);
    delay(10);

    // Step 6: Receive ACK for data (wait — calc may be writing to memory)
    cmd = recvShortPacketWait();
    Serial.print("Step6 ACK: 0x"); Serial.println(cmd, HEX);
    if (cmd != 0x56) { Serial.println("Expected ACK to data, aborting"); return; }

    // Step 7: Send EOT
    sendShortPacket(0x92);
    delay(10);

    // Step 8: Receive final ACK
    cmd = recvShortPacketWait();
    Serial.print("Step8 ACK: 0x"); Serial.println(cmd, HEX);

    Serial.println("Done! Check your calculator.");
}

void sendString(const char *str, uint8_t strSlot) {
    // strSlot: 0=Str0, 1=Str1, etc.
    Serial.println("\nSending string to calculator...");
    Serial.println("Put calc in receive mode: 2nd > LINK > RECEIVE, then press ENTER");
    delay(5000);

    uint16_t strLen = strlen(str);
    uint16_t dataSize = 2 + strLen;  // 2-byte length prefix + string bytes

    // Build data payload
    uint8_t payload[256];
    payload[0] = strLen & 0xFF;
    payload[1] = (strLen >> 8) & 0xFF;
    for (uint16_t i = 0; i < strLen; i++) {
        payload[2 + i] = str[i];
    }

    // Build 13-byte header
    uint8_t header[13] = {0};
    header[0] = dataSize & 0xFF;
    header[1] = (dataSize >> 8) & 0xFF;
    header[2] = 0x04;         // type: string
    header[3] = 0xAA;         // name byte 1 (Str token)
    header[4] = strSlot;      // name byte 2 (slot number)

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
    Serial.println("Press BOOT to send variable B=1234 to calculator.");
    Serial.println("Or send a variable from calc to receive it here.");
}

void loop() {
    static int byteCount = 0;
    static uint16_t expectedBytes = 4;
    static uint8_t lenLo = 0;
    static enum { WAIT_ANNOUNCE, WAIT_VAR_HDR, WAIT_ACK_TO_CTS, WAIT_DATA, WAIT_EOT } state = WAIT_ANNOUNCE;

    uint8_t data = getByte();
    if (!error_level) {
        recvBuf[byteCount] = data;  // save to buffer
        Serial.print("0x");
        if (data < 0x10) Serial.print("0");
        Serial.println(data, HEX);
        byteCount++;    

        if (byteCount == 3) lenLo = data;
        if (byteCount == 4) {
            uint16_t dataLen = (uint16_t)lenLo | ((uint16_t)data << 8);
            if (state == WAIT_ANNOUNCE || state == WAIT_EOT) {
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

            switch (state) {
                case WAIT_ANNOUNCE:
                    Serial.println("Got announce, ACKing");
                    sendShortPacket(0x56);
                    state = WAIT_VAR_HDR;
                    break;

                case WAIT_VAR_HDR:
                    recvVarType = recvBuf[6];  // type byte is at offset 6 (4 header + 2 data offset)
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
                    // Data bytes start at index 4 (after the 4-byte packet header)
                    // First 2 bytes of data are the string length
                    if (recvVarType == 0x04) {  // string
                        uint16_t strLen = recvBuf[4] | (recvBuf[5] << 8);
                        Serial.print("String value: ");
                        for (uint16_t i = 0; i < strLen; i++) {
                            Serial.print((char)recvBuf[6 + i]);
                        }
                        Serial.println();
                    }
                    state = WAIT_EOT;
                    break;

                case WAIT_EOT:
                    Serial.println("Got EOT, ACKing. Transfer complete!");
                    sendShortPacket(0x56);
                    state = WAIT_ANNOUNCE;
                    // Auto-respond with a string
                    if (recvVarType == 0x04) {
                        delay(500);  // brief pause before initiating send
                        sendString("RESPONSE", 1);
                    }
                    break;
            }
        }
    }

    if (digitalRead(BOOT_PIN) == LOW) {
        delay(50);
        if (digitalRead(BOOT_PIN) == LOW) {
            sendString("SUBSCRIBE", 1);  // sends "HELLO" to Str1
            while (digitalRead(BOOT_PIN) == LOW) delay(10);
        }
    }
}