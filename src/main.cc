#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "secrets.h"

#define TIP 6
#define RING 5
#define TIMEOUT_RX_US 500
#define TIMEOUT_RX_FIRST_BIT_US 100000  // 100ms inter-packet gap tolerance
#define TIMEOUT_TX_US 500000
#define BOOT_PIN 9

// Machine ID bytes per TI-83+ Link Protocol Guide:
//   0x73 = TI-83+ calculator
//   0x23 = Computer sending TI-83+/TI-84+ data  <-- ESP32 must use this for silent protocol
// The manual transfer path (sendProgram) keeps 0x73 — it works and is untouched.
// Silent protocol responses (RTS/REQ) use 0x23 via sendPacketSilent / sendShortPacketSilent.
#define MACHINE_ID_COMPUTER 0x73  // Protocol guide examples show PC using 0x73, same as calc

uint8_t recvBuf[256];
uint16_t recvBufLen = 0;
uint8_t recvVarType = 0;
bool pendingSendProgram = false;
char lastQuery[256] = {0};

bool error_level;

// TIGPT BASIC program:
//   ClrHome
//   Input "INPUT QUERY:", Str0
//   ClrHome
//   Disp "WAITING..."
//   Send(Str0)        -- manual link send of Str0 to ESP32
//   GetCalc(Str1)     -- silent receive: waits for ESP32 to push Str1
//   ClrHome
//   Disp Str1
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

// Sanitize Claude's response to characters the TI-83+ can display.
void sanitizeForCalc(const char *input, char *output, uint16_t maxLen) {
    uint16_t j = 0;
    for (uint16_t i = 0; input[i] != '\0' && j < maxLen - 1; i++) {
        char c = input[i];
        if (c >= 'A' && c <= 'Z') { output[j++] = c; continue; }
        if (c >= 'a' && c <= 'z') { output[j++] = c - 32; continue; }
        if (c >= '0' && c <= '9') { output[j++] = c; continue; }
        if (c == ' ' || c == '.' || c == ',' || c == '!' ||
            c == '?' || c == ':' || c == '-' || c == '+' ||
            c == '/' || c == '(' || c == ')') {
            output[j++] = c;
        }
    }
    output[j] = '\0';
}

void callClaude(const char *query, char *output, uint16_t maxLen) {
    Serial.print("Calling Claude with query: ");
    Serial.println(query);

    if (strlen(query) == 0) {
        strncpy(output, "NO QUERY RECEIVED", maxLen);
        return;
    }

    WiFiClientSecure client;
    client.setInsecure();

    if (!client.connect("api.anthropic.com", 443)) {
        strncpy(output, "CONNECTION ERROR", maxLen);
        return;
    }

    JsonDocument doc;
    doc["model"] = "claude-haiku-4-5";
    doc["max_tokens"] = 64;
    JsonArray messages = doc["messages"].to<JsonArray>();
    JsonObject msg = messages.add<JsonObject>();
    msg["role"] = "user";
    msg["content"] = query;
    doc["system"] = "You are an assistant on a TI-83 Plus calculator. "
                    "Respond in 50 characters or less. "
                    "Use only uppercase letters A-Z, digits 0-9, spaces, and basic punctuation (.,!?:-+/()). "
                    "No quotes, no newlines, no special characters. Be concise.";

    String body;
    serializeJson(doc, body);

    client.println("POST /v1/messages HTTP/1.1");
    client.println("Host: api.anthropic.com");
    client.println("Content-Type: application/json");
    client.println("anthropic-version: 2023-06-01");
    client.print("x-api-key: "); client.println(CLAUDE_API_KEY);
    client.print("Content-Length: "); client.println(body.length());
    client.println();
    client.print(body);

    uint32_t timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 10000) {
            strncpy(output, "TIMEOUT ERROR", maxLen);
            client.stop();
            return;
        }
    }

    String response = "";
    bool headersEnded = false;
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!headersEnded) {
            if (line == "\r") headersEnded = true;
        } else {
            response += line;
        }
    }
    client.stop();

    Serial.print("Raw response: ");
    Serial.println(response);

    JsonDocument resp;
    DeserializationError err = deserializeJson(resp, response);
    if (err) {
        strncpy(output, "PARSE ERROR", maxLen);
        return;
    }

    const char *text = resp["content"][0]["text"];
    if (!text) {
        strncpy(output, "NO RESPONSE", maxLen);
        return;
    }

    Serial.print("Claude says: ");
    Serial.println(text);
    sanitizeForCalc(text, output, maxLen);
    Serial.print("Sanitized: ");
    Serial.println(output);
}

uint8_t getByte(bool firstByte = false) {
    uint8_t data = 0;
    error_level = 0;
    int time_us = 0;
    for (int i = 0; i < 8; i++) {
        time_us = 0;
        // For the first bit of the first byte, use a longer timeout to allow
        // inter-packet settling time (e.g. after sending CTS, calc needs a moment).
        int timeout = (i == 0 && firstByte) ? TIMEOUT_RX_FIRST_BIT_US : TIMEOUT_RX_US;
        while(digitalRead(TIP) & digitalRead(RING)) {
            if (time_us > timeout) {
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

// Manual transfer packets — keep 0x73 (working baseline, untouched).
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

// Silent protocol packets — use 0x23 (computer machine ID per protocol guide).
void sendShortPacketSilent(uint8_t cmd) {
    sendByte(MACHINE_ID_COMPUTER);
    sendByte(cmd);
    sendByte(0x00);
    sendByte(0x00);
}

void sendPacketSilent(uint8_t cmd, const uint8_t *data, uint16_t length) {
    sendByte(MACHINE_ID_COMPUTER);
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

// Manual-mode program send. Uses 0x73 machine ID (working baseline, untouched).
// Triggered by A=69 variable send (calc already on receive screen, 10s to navigate there)
// or BOOT button press.
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

// Push Str1 to the calculator silently in response to GetCalc(Str1).
// The BASIC program calls GetCalc(Str1) which sends a silent REQ (0xA2) for Str1.
// We handle that REQ in the 0xA2 case below. This function handles the alternative
// path where we push proactively after receiving Str0 via Send().
// Per protocol guide "Sending a Variable" (silent RTS sequence):
//   1. Computer sends RTS (0xC9)
//   2. Calc sends ACK
//   3. Calc sends CTS
//   4. Computer sends ACK
//   5. Computer sends DATA
//   6. Calc sends ACK
//   7. Computer sends EOT
void sendStr1ToCalc(const char *str) {
    Serial.println("Pushing Str1 to calculator via silent RTS...");

    uint16_t strLen = strlen(str);
    uint16_t dataSize = 2 + strLen;

    // Variable header: 13 bytes per protocol guide.
    // [0-1] data size, [2] type ID, [3-10] name NUL-padded, [11] version=0, [12] typeID2=0
    // String varnames per guide: AA00 = Str1
    uint8_t header[13] = {0};
    header[0] = dataSize & 0xFF;
    header[1] = (dataSize >> 8) & 0xFF;
    header[2] = 0x04;   // Type ID: String
    header[3] = 0xAA;   // Str token byte 1
    header[4] = 0x00;   // Str1 token byte 2 (AA00 per guide)

    // Step 1: RTS
    sendPacketSilent(0xC9, header, 13);
    delay(10);

    // Step 2: Calc ACK
    uint8_t r = recvShortPacketWait();
    Serial.print("ACK to RTS: 0x"); Serial.println(r, HEX);
    if (r != 0x56) { Serial.println("Expected ACK to RTS, aborting"); return; }

    // Step 3: Calc CTS
    r = recvShortPacketWait();
    Serial.print("CTS: 0x"); Serial.println(r, HEX);
    if (r != 0x09) { Serial.println("Expected CTS, aborting"); return; }

    // Step 4: Computer ACK
    sendShortPacketSilent(0x56);
    delay(5);

    // Step 5: Computer DATA
    // String format: [0-1] length (LE), [2+] chars (space = TI token 0x29)
    uint8_t payload[256];
    payload[0] = strLen & 0xFF;
    payload[1] = (strLen >> 8) & 0xFF;
    for (uint16_t i = 0; i < strLen; i++) {
        payload[2 + i] = (str[i] == ' ') ? 0x29 : str[i];
    }
    sendPacketSilent(0x15, payload, dataSize);
    delay(10);

    // Step 6: Calc ACK
    r = recvShortPacketWait();
    Serial.print("ACK to data: 0x"); Serial.println(r, HEX);
    if (r != 0x56) { Serial.println("Expected ACK to data, aborting"); return; }

    // Step 7: Computer EOT
    sendShortPacketSilent(0x92);
    Serial.println("Str1 pushed to calculator!");
}

void setup() {
    Serial.begin(115200);  // High baud rate to minimize time spent in Serial prints
    delay(1500);
    pinMode(TIP, INPUT_PULLUP);
    pinMode(RING, INPUT_PULLUP);
    pinMode(BOOT_PIN, INPUT_PULLUP);

    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
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
    static bool firstByteOfPacket = false;

    uint8_t data = getByte(firstByteOfPacket);
    if (firstByteOfPacket) firstByteOfPacket = false;
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
                        // Calc sent RTS for Str0 as part of Send(Str0)/GetCalc(Str1) swap.
                        // Observed: RTS arrives, then immediately REQ for Str1 — no DATA yet.
                        // The calc is doing a variable swap: it will deliver Str0 AFTER we
                        // respond to its REQ for Str1. Save the announced data size so the
                        // REQ handler knows how many bytes to expect from Str0.
                        recvVarType = recvBuf[6];  // should be 0x04 (String)
                        // recvBuf[4..5] = variable data size (2-byte LE) from the var header
                        // which is at offset 4 in the full packet (after 4-byte packet header)
                        sendShortPacket(0x56);  // ACK the RTS
                        Serial.println("Got RTS for Str0, ACKed — waiting for REQ");
                        // Stay in WAIT_PACKET; REQ for Str1 arrives next
                    } else if (cmd == 0x09) {
                        sendShortPacket(0x56);  // ACK — time critical, before Serial
                        state = WAIT_DATA;
                        firstByteOfPacket = true;
                        Serial.println("Got CTS, ACKing, waiting for data");
                    } else if (cmd == 0xA2) {
                        // Calc sent a silent REQ (0xA2) for Str1 — this is GetCalc(Str1).
                        // We are the sender responding to the calc's request.
                        // Per protocol guide "Requesting a Variable", from the sender's side:
                        //   1. Calc sends REQ (0xA2)     <- here
                        //   2. Computer sends ACK
                        //   3. Computer sends VAR header (0x06)
                        //   4. Calc sends ACK
                        //   5. Calc sends CTS (0x09)
                        //   6. Computer sends ACK
                        //   7. Computer sends DATA
                        //   8. Calc sends ACK
                        Serial.println("Got REQ for Str1 (GetCalc), responding...");

                        // Step 2: ACK the REQ immediately
                        sendShortPacketSilent(0x56);
                        delay(5);

                        // This is a variable swap: Send(Str0) + GetCalc(Str1).
                        // The calc REQ'd Str1 first — we must respond to that before
                        // the calc will deliver Str0. Use a placeholder response for now,
                        // receive Str0, then we cannot update Str1 after the fact.
                        //
                        // ACTUAL OBSERVED PROTOCOL (swap sequence):
                        //   Calc: RTS for Str0    (announcement)
                        //   ESP:  ACK
                        //   Calc: REQ for Str1    <- we are here
                        //   ESP:  ACK             (step 2, done above)
                        //   ESP:  VAR header for Str1
                        //   Calc: ACK
                        //   Calc: CTS
                        //   ESP:  ACK
                        //   ESP:  DATA (Str1)
                        //   Calc: ACK
                        //   Calc: DATA (Str0)     <- Str0 arrives AFTER we send Str1
                        //   ESP:  ACK
                        //   Calc: EOT
                        //   ESP:  ACK
                        //
                        // Strategy: send a placeholder Str1 first, receive Str0 (the query),
                        // call Claude, then use sendStr1ToCalc() to push the real answer.
                        // The BASIC program displays Str1 after a Pause — so we push the
                        // real answer before the user presses any key.

                        // --- Send placeholder Str1 so the swap can complete ---
                        const char *placeholder = "THINKING...";
                        uint16_t strLen = strlen(placeholder);
                        uint16_t dataSize = 2 + strLen;

                        // VAR header for Str1, 13 bytes per protocol guide (AA00 = Str1)
                        uint8_t header[13] = {0};
                        header[0] = dataSize & 0xFF;
                        header[1] = (dataSize >> 8) & 0xFF;
                        header[2] = 0x04;
                        header[3] = 0xAA;
                        header[4] = 0x00;  // Str1
                        sendPacketSilent(0x06, header, 13);
                        delay(10);

                        uint8_t r = recvShortPacketWait();
                        Serial.print("ACK to VAR header: 0x"); Serial.println(r, HEX);
                        if (r != 0x56) { Serial.println("Expected ACK, aborting"); break; }

                        r = recvShortPacketWait();
                        Serial.print("CTS: 0x"); Serial.println(r, HEX);
                        if (r != 0x09) { Serial.println("Expected CTS, aborting"); break; }

                        sendShortPacketSilent(0x56);  // ACK CTS
                        delay(5);

                        uint8_t payload[256];
                        payload[0] = strLen & 0xFF;
                        payload[1] = (strLen >> 8) & 0xFF;
                        for (uint16_t i = 0; i < strLen; i++) {
                            payload[2 + i] = (placeholder[i] == ' ') ? 0x29 : placeholder[i];
                        }
                        sendPacketSilent(0x15, payload, dataSize);
                        delay(10);

                        r = recvShortPacketWait();
                        Serial.print("ACK to Str1 data: 0x"); Serial.println(r, HEX);
                        Serial.println("Placeholder Str1 sent, now receiving Str0...");

                        // --- Now receive Str0 (the actual query) ---
                        // After we send Str1 data, the calc delivers Str0 as DATA
                        // followed by EOT to complete the swap.
                        memset(lastQuery, 0, sizeof(lastQuery));

                        // Expect DATA packet for Str0
                        uint8_t dh[4];
                        for (int i = 0; i < 4; i++) dh[i] = getByte(i == 0);
                        if (dh[1] == 0x15) {
                            uint16_t dlen = dh[2] | (dh[3] << 8);
                            uint8_t sd[256] = {0};
                            for (uint16_t i = 0; i < dlen + 2 && i < sizeof(sd); i++) sd[i] = getByte();

                            uint16_t qLen = sd[0] | (sd[1] << 8);
                            for (uint16_t i = 0; i < qLen && i < 255; i++) {
                                lastQuery[i] = (sd[2 + i] == 0x29) ? ' ' : sd[2 + i];
                            }
                            Serial.print("Str0 query: "); Serial.println(lastQuery);
                            sendShortPacket(0x56);  // ACK the Str0 DATA
                            delay(5);

                            // Receive EOT
                            uint8_t eot[4];
                            for (int i = 0; i < 4; i++) eot[i] = getByte(i == 0);
                            Serial.print("EOT cmd: 0x"); Serial.println(eot[1], HEX);
                            sendShortPacket(0x56);  // ACK EOT
                        } else {
                            Serial.print("Expected DATA for Str0, got cmd: 0x");
                            Serial.println(dh[1], HEX);
                        }

                        // --- Call Claude and push real answer via silent RTS ---
                        if (strlen(lastQuery) > 0) {
                            Serial.println("Calling Claude...");
                            char claudeResponse[64] = {0};
                            callClaude(lastQuery, claudeResponse, sizeof(claudeResponse));
                            memset(lastQuery, 0, sizeof(lastQuery));
                            // Push the real Str1 now — BASIC program shows it on Disp Str1
                            sendStr1ToCalc(claudeResponse);
                        } else {
                            Serial.println("No query received in swap");
                        }
                        Serial.println("Swap complete!");

                    } else if (cmd == 0x68) {
                        sendShortPacket(0x56);  // ACK immediately
                        Serial.println("Got manual announce, ACKing");
                    } else if (cmd == 0x06) {
                        recvVarType = recvBuf[6];
                        sendShortPacket(0x56);  // ACK — time critical, before Serial
                        delay(5);
                        sendShortPacket(0x09);  // CTS
                        state = WAIT_DATA;
                        firstByteOfPacket = true;
                        Serial.println("Got VAR header, ACKing + CTS");
                    } else {
                        Serial.print("Unknown cmd: 0x");
                        Serial.println(cmd, HEX);
                    }
                    break;

                case WAIT_DATA:
                    if (cmd == 0x56) {
                        Serial.println("Got ACK to CTS, data incoming");
                    } else if (cmd == 0x15) {
                        sendShortPacket(0x56);  // ACK DATA immediately — time critical
                        Serial.println("Got DATA, ACKing");
                        if (recvVarType == 0x04) {
                            // String variable — save as Claude query
                            uint16_t strLen = recvBuf[4] | (recvBuf[5] << 8);
                            memset(lastQuery, 0, sizeof(lastQuery));
                            for (uint16_t i = 0; i < strLen && i < 255; i++) {
                                // 0x29 is TI space token, convert to ASCII space
                                lastQuery[i] = (recvBuf[6 + i] == 0x29) ? ' ' : recvBuf[6 + i];
                            }
                            Serial.print("Query received: ");
                            Serial.println(lastQuery);
                            if (strcmp(lastQuery, "69") == 0) {
                                Serial.println("Magic string! Will send program after EOT.");
                                pendingSendProgram = true;
                            }
                        }
                        if (recvVarType == 0x00) {
                            // Real number — check for magic value 69 (TI float: 81 69)
                            if (recvBuf[5] == 0x81 && recvBuf[6] == 0x69) {
                                Serial.println("Magic number 69! Will send program after EOT.");
                                pendingSendProgram = true;
                            }
                        }
                        state = WAIT_EOT;
                    }
                    break;

                case WAIT_EOT:
                    sendShortPacket(0x56);  // ACK EOT immediately — time critical
                    Serial.println("Got EOT, ACKing.");
                    state = WAIT_PACKET;
                    if (pendingSendProgram) {
                        pendingSendProgram = false;
                        sendProgram();
                    } else if (strlen(lastQuery) > 0 && recvVarType == 0x04) {
                        // Str0 query arrived via Send(Str0). Call Claude and push Str1 back.
                        // The BASIC program's GetCalc(Str1) will be waiting.
                        Serial.println("String query received, calling Claude...");
                        char claudeResponse[64] = {0};
                        callClaude(lastQuery, claudeResponse, sizeof(claudeResponse));
                        memset(lastQuery, 0, sizeof(lastQuery));
                        sendStr1ToCalc(claudeResponse);
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