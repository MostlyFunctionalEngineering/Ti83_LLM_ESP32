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
bool pendingRemoteSend = false;
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
    // Menu("TIGPT","ASK",1,"SHOW",2,"EXIT",3)
    0xE6,
    0x2A,'T','I','G','P','T',0x2A,0x2B,
    0x2A,'A','S','K',0x2A,0x2B,0x31,0x2B,
    0x2A,'S','H','O','W',0x2A,0x2B,0x32,0x2B,
    0x2A,'E','X','I','T',0x2A,0x2B,0x33,
    0x11,0x3F,
    // Lbl 1: ASK
    0xD6,0x31,0x3F,
    // DelVar Str0-Str9
    0xBB,0x54,0xAA,0x09,0x3F,
    0xBB,0x54,0xAA,0x00,0x3F,
    0xBB,0x54,0xAA,0x01,0x3F,
    0xBB,0x54,0xAA,0x02,0x3F,
    0xBB,0x54,0xAA,0x03,0x3F,
    0xBB,0x54,0xAA,0x04,0x3F,
    0xBB,0x54,0xAA,0x05,0x3F,
    0xBB,0x54,0xAA,0x06,0x3F,
    0xBB,0x54,0xAA,0x07,0x3F,
    0xBB,0x54,0xAA,0x08,0x3F,
    // ClrHome
    0xE1,0x3F,
    // Input "QUERY:",Str0
    0xDC,0x2A,'Q','U','E','R','Y',':',0x2A,0x2B,0xAA,0x09,0x3F,
    // ClrHome
    0xE1,0x3F,
    // Disp "SENDING"
    0xDE,0x2A,'S','E','N','D','I','N','G',0x2A,0x3F,
    // 69420->B : Send(B) — RTS triggers ESP32 remoteSendStr0()
    0x36,0x39,0x34,0x32,0x30,0x04,0x42,0x3F,
    0xE7,0x42,0x11,0x3F,
    // Stop
    0xD9,0x3F,
    // Lbl 2: SHOW — up to 8 rows (128 chars), generated programmatically
    0xD6,0x32,0x3F,
    0xE1,0x3F,
    // Row 1: always
    0xE0, 0x31, 0x2B, 0x31, 0x2B, 0xAA, 0x00, 0x11, 0x3F,
    // Row 2: if length>16
    0xCE, 0xBB, 0x2B, 0xAA, 0x00, 0x11, 0x6C, 0x31, 0x36, 0x3F,
    0xE0, 0x32, 0x2B, 0x31, 0x2B, 0xBB, 0x0C, 0xAA, 0x00, 0x2B, 0x31, 0x37, 0x2B, 0x31, 0x36, 0x11, 0x11, 0x3F,
    // Row 3: if length>32
    0xCE, 0xBB, 0x2B, 0xAA, 0x00, 0x11, 0x6C, 0x33, 0x32, 0x3F,
    0xE0, 0x33, 0x2B, 0x31, 0x2B, 0xBB, 0x0C, 0xAA, 0x00, 0x2B, 0x33, 0x33, 0x2B, 0x31, 0x36, 0x11, 0x11, 0x3F,
    // Row 4: if length>48
    0xCE, 0xBB, 0x2B, 0xAA, 0x00, 0x11, 0x6C, 0x34, 0x38, 0x3F,
    0xE0, 0x34, 0x2B, 0x31, 0x2B, 0xBB, 0x0C, 0xAA, 0x00, 0x2B, 0x34, 0x39, 0x2B, 0x31, 0x36, 0x11, 0x11, 0x3F,
    // Row 5: if length>64
    0xCE, 0xBB, 0x2B, 0xAA, 0x00, 0x11, 0x6C, 0x36, 0x34, 0x3F,
    0xE0, 0x35, 0x2B, 0x31, 0x2B, 0xBB, 0x0C, 0xAA, 0x00, 0x2B, 0x36, 0x35, 0x2B, 0x31, 0x36, 0x11, 0x11, 0x3F,
    // Row 6: if length>80
    0xCE, 0xBB, 0x2B, 0xAA, 0x00, 0x11, 0x6C, 0x38, 0x30, 0x3F,
    0xE0, 0x36, 0x2B, 0x31, 0x2B, 0xBB, 0x0C, 0xAA, 0x00, 0x2B, 0x38, 0x31, 0x2B, 0x31, 0x36, 0x11, 0x11, 0x3F,
    // Row 7: if length>96
    0xCE, 0xBB, 0x2B, 0xAA, 0x00, 0x11, 0x6C, 0x39, 0x36, 0x3F,
    0xE0, 0x37, 0x2B, 0x31, 0x2B, 0xBB, 0x0C, 0xAA, 0x00, 0x2B, 0x39, 0x37, 0x2B, 0x31, 0x36, 0x11, 0x11, 0x3F,
    // Row 8: if length>112
    0xCE, 0xBB, 0x2B, 0xAA, 0x00, 0x11, 0x6C, 0x31, 0x31, 0x32, 0x3F,
    0xE0, 0x38, 0x2B, 0x31, 0x2B, 0xBB, 0x0C, 0xAA, 0x00, 0x2B, 0x31, 0x31, 0x33, 0x2B, 0x31, 0x36, 0x11, 0x11, 0x3F,
    // Stop
    0xD9,0x3F,
    // Lbl 3: EXIT
    0xD6,0x33,0x3F,
    0xE1,0x3F,
    0xD9
};
const uint16_t espreqProgramSize = sizeof(espreqProgram);
const char espreqName[] = "TIGPT";

// Sanitize Claude's response to characters the TI-83+ can display.
void sanitizeForCalc(const char *input, char *output, uint16_t maxLen) {
    uint16_t j = 0;
    bool lastWasSpace = false;
    for (uint16_t i = 0; input[i] != '\0' && j < maxLen - 1; i++) {
        char c = input[i];
        if (c >= 'A' && c <= 'Z') { output[j++] = c; lastWasSpace = false; continue; }
        if (c >= 'a' && c <= 'z') { output[j++] = c - 32; lastWasSpace = false; continue; }
        if (c >= '0' && c <= '9') { output[j++] = c; lastWasSpace = false; continue; }
        if (c == ' ' || c == '.' || c == ',' || c == '!' ||
            c == '?' || c == ':' || c == '-' || c == '+' ||
            c == '/' || c == '(' || c == ')' || c == '=') {
            // Note: * excluded (0x2A = TI quote token)
            output[j++] = c; lastWasSpace = (c == ' '); continue;
        }
        if (c == '\n' || c == '\r' || c == '\t') {
            // Replace whitespace/newlines with a space (avoid words running together)
            if (!lastWasSpace && j < maxLen - 1) { output[j++] = ' '; lastWasSpace = true; }
            continue;
        }
        // Any other disallowed char (apostrophe, etc.) — insert space if needed
        if (!lastWasSpace && j < maxLen - 1) { output[j++] = ' '; lastWasSpace = true; }
    }
    // Trim trailing space
    while (j > 0 && output[j-1] == ' ') j--;
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
    doc["system"] = "You are an assistant on a TI-83 Plus calculator with a 16-character wide screen. "
                    "Respond in 128 characters or less (fits the full TI-83+ screen of 8 rows x 16 chars). "
                    "Use only uppercase letters A-Z, digits 0-9, spaces, and basic punctuation (.,!?:-+()=/>). "
                    "No quotes, no newlines, no special characters. Be extremely concise.";

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

    // Read past HTTP headers
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (line == "\r" || line == "") break;
    }

    // Read body — handle chunked transfer encoding.
    // Chunked format: hex-size CRLF, data CRLF, repeat, "0" CRLF CRLF to end.
    // We accumulate all data chunks into response.
    String response = "";
    while (client.available()) {
        // Read chunk size line (hex number)
        String sizeLine = client.readStringUntil('\n');
        sizeLine.trim();
        if (sizeLine.length() == 0) continue;
        long chunkSize = strtol(sizeLine.c_str(), nullptr, 16);
        if (chunkSize == 0) break;  // final chunk
        // Read exactly chunkSize bytes
        for (long i = 0; i < chunkSize && client.available(); i++) {
            response += (char)client.read();
        }
        client.readStringUntil('\n');  // consume trailing CRLF after chunk data
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
    // Hard truncate to 32 chars — 2 lines on the TI-83+ 16-char wide screen
    if (strlen(output) > 128) output[128] = '\0';
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

// Silent protocol packets.
// The calc's GetCalc REQ arrives with machine ID 0x03 (TI-83 computer mode).
// We try 0x03 for our REQ back, matching what the calc uses.
#define MACHINE_ID_PC03 0x03

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

// Use machine ID 0x03 — matches what the calc itself uses for silent ops on TI-83
void sendShortPacket03(uint8_t cmd) {
    sendByte(MACHINE_ID_PC03);
    sendByte(cmd);
    sendByte(0x00);
    sendByte(0x00);
}

void sendPacket03(uint8_t cmd, const uint8_t *data, uint16_t length) {
    sendByte(MACHINE_ID_PC03);
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

// Program send — uses remote control to put calc in receive mode first,
// then immediately sends. No manual steps required.
// For BOOT button: calc must be at home screen (not in a menu or program).
// Forward declarations
void sendKey(uint8_t keycode, bool twoAcks = true);
void calcEnterReceiveMode();
void remoteSendStr0();

void sendProgram() {
    Serial.println("\nSending TIGPT program...");
    Serial.println("Using remote control to enter receive mode...");
    calcEnterReceiveMode();  // sends kRecieve (0x14), waits 1s for calc to be ready

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

    // Remote-control: exit receive screen and open PRGM menu
    delay(500);
    Serial.println("Exiting receive screen...");
    sendKey(0x40, false);   // kQuit (0x40) — back to home screen
    delay(500);
    Serial.println("Opening PRGM menu...");
    sendKey(0x2D, false);   // kPrgm (0x2D) — open PRGM EXEC menu
    Serial.println("Calc is now at PRGM menu.");
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
    Serial.println("Sending Str1 to calculator (manual receive mode)...");

    uint16_t strLen = strlen(str);
    uint16_t dataSize = 2 + strLen;

    // Variable header: 13 bytes per protocol guide.
    // String varnames per guide: AA00 = Str1
    uint8_t header[13] = {0};
    header[0] = dataSize & 0xFF;
    header[1] = (dataSize >> 8) & 0xFF;
    header[2] = 0x04;   // Type ID: String
    header[3] = 0xAA;
    header[4] = 0x00;   // Str1 (AA00)

    // Manual send sequence (same as sendProgram, calc must be in receive mode):
    //   1. Computer sends VAR (0x06)
    //   2. Calc ACKs
    //   3. Calc sends CTS
    //   4. Computer ACKs
    //   5. Computer sends DATA
    //   6. Calc ACKs
    //   7. Computer sends EOT
    //   8. Calc ACKs
    sendPacket(0x06, header, 13);
    delay(10);

    uint8_t r = recvShortPacket();
    Serial.print("ACK: 0x"); Serial.println(r, HEX);
    if (r != 0x56) { Serial.println("Expected ACK, aborting"); return; }

    r = recvShortPacketWait();
    Serial.print("CTS: 0x"); Serial.println(r, HEX);
    if (r != 0x09) { Serial.println("Expected CTS, aborting"); return; }

    sendShortPacket(0x56);
    delay(5);

    uint8_t payload[256];
    payload[0] = strLen & 0xFF;
    payload[1] = (strLen >> 8) & 0xFF;
    for (uint16_t i = 0; i < strLen; i++) {
        char c = str[i];
        if (c == ' ')  { payload[2 + i] = 0x29; continue; }  // TI space token
        if (c == '.')  { payload[2 + i] = 0x3A; continue; }  // TI decimal point char
        payload[2 + i] = c;
    }
    sendPacket(0x15, payload, dataSize);
    delay(10);

    r = recvShortPacketWait();
    Serial.print("ACK to data: 0x"); Serial.println(r, HEX);
    if (r != 0x56) { Serial.println("Expected ACK, aborting"); return; }

    sendShortPacket(0x92);
    delay(10);
    recvShortPacketWait();
    Serial.println("Str1 sent!");
}


// Send a remote control keypress to the calculator.
// Per protocol guide: machine ID 0x23, cmd 0x87, 2-byte scancode (little-endian).
// The guide example shows TWO ACKs for echo keys (keys that type characters).
// Navigation/mode keys (like kRecieve) only produce ONE ACK — they open a screen
// and don't echo into a buffer.
void sendKey(uint8_t keycode, bool twoAcks) {
    sendByte(0x23);   // computer machine ID for TI-83+
    sendByte(0x87);   // CMD command
    sendByte(keycode);
    sendByte(0x00);
    recvShortPacketWait();          // ACK 1: always present
    if (twoAcks) {
        recvShortPacketWait();      // ACK 2: only for echo/processing keys
    }
    delay(150);
}

// Put the calculator into receive mode silently via remote control.
// kRecieve = 0x14 per keys.txt — navigates to link receive screen (one ACK only).
// After sending, wait for the calc to fully render the receive screen before
// starting the transfer.
void calcEnterReceiveMode() {
    Serial.println("Sending kRecieve remote key...");
    sendKey(0x14, false);  // kRecieve EQU 014h — navigation key, one ACK
    delay(1000);           // wait for calc to enter receive mode fully
    Serial.println("Calc should now be in receive mode");
}


// Remotely send Str0 via Link Send menu.
// Key sequence per TI-83+ Link menu navigation:
//   kLinkIO -> opens Link menu
//   k1 -> selects "1:Send" 
//   9x kDown -> navigates to "String" category (10th item)
//   kEnter -> opens String list (Str0 is only entry after DelVar)
//   kSelect -> marks Str0 for sending
//   kTrans -> transmits
// Key codes from merthsoft.com/linkguide/ti83+/keys.txt:
//   kLinkIO=0x41, k1=0x8F, kDown=0x04, kEnter=0x05
//   kSelect=0x69, kTrans=0x16
void remoteSendStr0() {
    // Exact manual sequence: 2ND+LINK -> B -> ENTER -> RIGHT -> ENTER
    // kLinkIO (0x41) — opens Link menu (2ND+LINK)
    // kCapB   (0x9B) — jumps to String category
    // kEnter  (0x05) — selects Str0 (only string after DelVar)
    // kRight  (0x01) — moves to TRANSMIT tab
    // kEnter  (0x05) — executes transmit
    Serial.println("Remote-sending Str0 via Link menu...");
    delay(200);
    sendKey(0x41, false);   // kLinkIO — open Link menu
    delay(200);
    sendKey(0x9B, true);    // kCapB — jump to String category
    delay(200);
    sendKey(0x05, false);   // kEnter — select Str0
    delay(200);
    sendKey(0x01, false);   // kRight — move to TRANSMIT tab
    delay(200);
    sendKey(0x05, false);   // kEnter — transmit
    Serial.println("Remote send initiated.");
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
    Serial.println("Send A=69 from calc to install TIGPT program.");
    Serial.println("After typing query in TIGPT, press BOOT to send Str0 to ESP32.");
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
                        recvVarType = recvBuf[6];
                        // recvBuf[9] = variable name byte 1 (offset 4+2+type = byte 9 of packet)
                        // Variable header in RTS: bytes 4-5=data size, 6=type, 7-14=name
                        uint8_t varType = recvBuf[6];
                        uint8_t varName = recvBuf[7];
                        if (varType == 0x00 && varName == 0x42) {
                            // RTS for Real variable B — Send(B) magic trigger from BASIC.
                            // ACK it, then immediately call remoteSendStr0().
                            // Don't wait for EOT — Send() from BASIC never completes properly.
                            sendShortPacket(0x56);  // ACK
                            Serial.println("Got RTS for B — calling remoteSendStr0 now");
                            delay(200);
                            remoteSendStr0();
                            // Stay in WAIT_PACKET
                        } else {
                            // Manual 2ND LINK SEND (machine ID 0x73) — full receive
                            sendShortPacket(0x56);  // ACK
                            delay(5);
                            sendShortPacket(0x09);  // CTS
                            state = WAIT_DATA;
                            firstByteOfPacket = true;
                            Serial.println("Got RTS (manual), ACKed + sent CTS");
                        }
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

                        // Silently REQ Str0 from the calc to get the query.
                        // The calc holds Str0 in memory; we pull it now before responding.
                        // Silent REQ sequence per protocol guide:
                        //   Computer sends REQ -> Calc ACKs -> Calc sends VAR header
                        //   -> Computer ACKs -> Computer sends CTS -> Calc ACKs
                        //   -> Calc sends DATA -> Computer ACKs
                        Serial.println("REQing Str0 from calc...");
                        memset(lastQuery, 0, sizeof(lastQuery));
                        {
                            uint8_t req[13] = {0};
                            req[2] = 0x04;   // String
                            req[3] = 0xAA;
                            req[4] = 0x09;   // Str0 (AA09)
                            // Use machine ID 0x03 — matches what calc uses for its own silent ops
                            sendPacket03(0xA2, req, 13);
                            delay(10);

                            uint8_t ra = recvShortPacketWait();
                            Serial.print("ACK to Str0 REQ: 0x"); Serial.println(ra, HEX);

                            if (ra == 0x56) {
                                // Read VAR header: 4-byte pkt hdr + 13-byte var hdr + 2-byte chk = 19
                                uint8_t varHdr[19] = {0};
                                for (int i = 0; i < 19; i++) varHdr[i] = getByte(i == 0);
                                Serial.println("Str0 VAR header received");

                                sendShortPacket03(0x56);  // ACK VAR header
                                delay(5);
                                sendShortPacket03(0x09);  // CTS
                                delay(5);

                                ra = recvShortPacketWait();   // Calc ACKs CTS
                                Serial.print("ACK to CTS: 0x"); Serial.println(ra, HEX);

                                // Calc sends DATA — wait then read full packet
                                uint32_t t = millis();
                                while (digitalRead(TIP) && digitalRead(RING)) {
                                    if (millis() - t > 3000) { Serial.println("Timeout on Str0 DATA"); break; }
                                }
                                uint8_t dh[4];
                                dh[0]=getByte(); dh[1]=getByte(); dh[2]=getByte(); dh[3]=getByte();
                                Serial.print("Str0 DATA cmd: 0x"); Serial.println(dh[1], HEX);

                                if (dh[1] == 0x15) {
                                    uint16_t dlen = dh[2] | (dh[3] << 8);
                                    uint8_t sd[256] = {0};
                                    for (uint16_t i = 0; i < dlen + 2 && i < sizeof(sd); i++) sd[i] = getByte();
                                    uint16_t qLen = sd[0] | (sd[1] << 8);
                                    for (uint16_t i = 0; i < qLen && i < 255; i++) {
                                        lastQuery[i] = (sd[2 + i] == 0x29) ? ' ' : sd[2 + i];
                                    }
                                    Serial.print("Str0 query: "); Serial.println(lastQuery);
                                    sendShortPacket03(0x56);  // ACK DATA
                                    delay(5);
                                }
                            } else {
                                Serial.println("No ACK to Str0 REQ");
                            }
                        }

                        char claudeResponse[64] = {0};
                        if (strlen(lastQuery) > 0) {
                            callClaude(lastQuery, claudeResponse, sizeof(claudeResponse));
                            memset(lastQuery, 0, sizeof(lastQuery));
                        } else {
                            strncpy(claudeResponse, "NO QUERY RECEIVED", sizeof(claudeResponse));
                        }

                        uint16_t strLen = strlen(claudeResponse);
                        uint16_t dataSize = 2 + strLen;

                        uint8_t header[13] = {0};
                        header[0] = dataSize & 0xFF;
                        header[1] = (dataSize >> 8) & 0xFF;
                        header[2] = 0x04;
                        header[3] = 0xAA;
                        header[4] = 0x00;   // Str1 (AA00)
                        sendPacketSilent(0x06, header, 13);
                        delay(10);

                        uint8_t r = recvShortPacketWait();
                        Serial.print("ACK to VAR header: 0x"); Serial.println(r, HEX);
                        if (r != 0x56) { Serial.println("Expected ACK, aborting"); break; }

                        r = recvShortPacketWait();
                        Serial.print("CTS: 0x"); Serial.println(r, HEX);
                        if (r != 0x09) { Serial.println("Expected CTS, aborting"); break; }

                        sendShortPacketSilent(0x56);
                        delay(5);

                        uint8_t payload[256];
                        payload[0] = strLen & 0xFF;
                        payload[1] = (strLen >> 8) & 0xFF;
                        for (uint16_t i = 0; i < strLen; i++) {
                            char cc = claudeResponse[i];
                            if (cc == ' ')  { payload[2 + i] = 0x29; continue; }
                            if (cc == '.')  { payload[2 + i] = 0x3A; continue; }
                            payload[2 + i] = cc;
                        }
                        sendPacketSilent(0x15, payload, dataSize);
                        delay(10);

                        r = recvShortPacketWait();
                        Serial.print("ACK to Str1 data: 0x"); Serial.println(r, HEX);
                        Serial.println("Str1 response sent!");

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
                    } else if (cmd == 0x56) {
                        Serial.println("Stray ACK, ignoring");
                    } else if (cmd == 0x92) {
                        sendShortPacket(0x56);
                        Serial.println("Stray EOT, ACKed");
                    } else if (cmd == 0x36) {
                        // SKIP/EXIT from calc — out of memory or variable conflict
                        sendShortPacket(0x56);  // ACK it
                        Serial.println("Got SKIP/EXIT (0x36) from calc — out of memory or conflict");
                    } else {
                        Serial.print("Unknown cmd: 0x");
                        Serial.println(cmd, HEX);
                    }
                    break;

                case WAIT_DATA:
                    if (cmd == 0x56) {
                        Serial.println("Got ACK to CTS, data incoming");
                    } else if (cmd == 0x09) {
                        // CTS from calc (BASIC Send() path) — ACK it, data follows
                        sendShortPacket(0x56);
                        Serial.println("Got CTS from calc in WAIT_DATA, ACKed");
                    } else if (cmd == 0x68) {
                        // Manual announce mid-transfer — ACK and stay in WAIT_DATA
                        sendShortPacket(0x56);
                        Serial.println("Got announce in WAIT_DATA, ACKed, staying");
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
                            Serial.print("Real bytes[5,6]: 0x");
                            Serial.print(recvBuf[5],HEX); Serial.print(" 0x");
                            Serial.println(recvBuf[6],HEX);
                            if (recvBuf[5] == 0x81 && recvBuf[6] == 0x69) {
                                Serial.println("Magic 69 — will send program after EOT.");
                                pendingSendProgram = true;
                            } else if (recvBuf[5] == 0x84 && recvBuf[6] == 0x69) {
                                Serial.println("Magic 69420 — will remote-send Str0 after EOT.");
                                pendingRemoteSend = true;
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
                    } else if (pendingRemoteSend) {
                        pendingRemoteSend = false;
                        delay(300);
                        remoteSendStr0();
                    } else if (strlen(lastQuery) > 0 && recvVarType == 0x04) {
                        // Str0 query received via manual send. Call Claude.
                        Serial.println("Query received, calling Claude...");
                        char claudeResponse[64] = {0};
                        callClaude(lastQuery, claudeResponse, sizeof(claudeResponse));
                        memset(lastQuery, 0, sizeof(lastQuery));
                        // Push Str1 back via manual send sequence.
                        // User has been prompted by BASIC program to set calc to receive mode.
                        Serial.println("Using remote control to put calc in receive mode...");
                        delay(300);
                        calcEnterReceiveMode();
                        sendStr1ToCalc(claudeResponse);
                        // Overwrite/rename/omit screen: send k2 (0x90) = option 2: Overwrite
                        delay(500);
                        Serial.println("Selecting Overwrite (option 2)...");
                        sendKey(0x90, true);    // k2 = 0x90
                        delay(300);
                        // Dismiss "Done" and launch SHOW
                        Serial.println("Dismissing Done screen and launching SHOW...");
                        sendKey(0x05, false);   // kEnter — dismiss Done
                        delay(300);
                        // Clear home screen, then type prgmTIGPT and run option 2
                        sendKey(0x09, false);   // kClear
                        delay(200);
                        sendKey(0x2D, false);   // kPrgm — open PRGM EXEC menu
                        delay(300);
                        // Type TIGPT to search/select the program
                        sendKey(0xAD, true);    // kCapT
                        sendKey(0xA2, true);    // kCapI
                        sendKey(0xA0, true);    // kCapG
                        sendKey(0xA9, true);    // kCapP
                        sendKey(0xAD, true);    // kCapT
                        delay(200);
                        sendKey(0x05, false);   // kEnter — paste prgmTIGPT to home screen
                        delay(300);
                        sendKey(0x05, false);   // kEnter — execute prgmTIGPT (run it)
                        delay(800);             // wait for Menu( to render
                        sendKey(0x90, true);    // k2 — select and run SHOW
                        delay(200);
                        Serial.println("SHOW launched via remote control");
                    }
                    break;
            }
        }
    }

    if (digitalRead(BOOT_PIN) == LOW) {
        delay(50);
        if (digitalRead(BOOT_PIN) == LOW) {
            remoteSendStr0();
            while (digitalRead(BOOT_PIN) == LOW) delay(10);
        }
    }
}