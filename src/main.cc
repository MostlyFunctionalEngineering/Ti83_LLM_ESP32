#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Preferences.h>

#define TI_TIP 4
#define TI_RING 5

#define WIFI_RESET_PIN 9
#define DEVICE_HOSTNAME "TI-83 Plus"

#define DEBUG 1

#if DEBUG
#define DBG(x) Serial.println(x)
#define DBGF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(x)
#define DBGF(...)
#endif

Preferences prefs;
String apiKey;
WiFiManager wm;

// -----------------------------
// TI Link helpers
// -----------------------------
inline void tipRelease() { pinMode(TI_TIP, INPUT_PULLUP); }
inline void tipLow() { pinMode(TI_TIP, OUTPUT); digitalWrite(TI_TIP, LOW); }

inline void ringRelease() { pinMode(TI_RING, INPUT_PULLUP); }
inline void ringLow() { pinMode(TI_RING, OUTPUT); digitalWrite(TI_RING, LOW); }

inline bool tipRead() { return digitalRead(TI_TIP); }
inline bool ringRead() { return digitalRead(TI_RING); }

void tiIdle() { tipRelease(); ringRelease(); }

// Wait helper to prevent deadlock
bool waitWhile(bool (*cond)(), uint32_t timeoutMicros = 200000) {
    uint32_t start = micros();
    while (cond()) {
        if (micros() - start > timeoutMicros)
            return false;
    }
    return true;
}

// -----------------------------
// TI Bit Send/Receive
// -----------------------------
bool tiSendBit(bool bit) {
    if (bit) {
        tipLow();
        if (!waitWhile(ringRead)) return false;
        tipRelease();
        if (!waitWhile([]() { return !ringRead(); })) return false;
    } else {
        ringLow();
        if (!waitWhile(tipRead)) return false;
        ringRelease();
        if (!waitWhile([]() { return !tipRead(); })) return false;
    }
    return true;
}

bool tiReceiveBit(bool &bit) {
    if (!waitWhile([]() { return tipRead() && ringRead(); }))
        return false;

    if (!tipRead()) {
        ringLow();
        if (!waitWhile([]() { return !tipRead(); })) return false;
        ringRelease();
        bit = true;
    } else {
        tipLow();
        if (!waitWhile([]() { return !ringRead(); })) return false;
        tipRelease();
        bit = false;
    }
    return true;
}

void tiSendByte(uint8_t value) {
    for (int i = 0; i < 8; i++) {
        if (!tiSendBit(value & 1)) return;
        value >>= 1;
    }
}

bool tiReceiveByte(uint8_t &value) {
    value = 0;
    for (int i = 0; i < 8; i++) {
        bool bit;
        if (!tiReceiveBit(bit)) return false;
        if (bit) value |= (1 << i);
    }
    return true;
}

// -----------------------------
String tiReceiveString() {
    uint8_t len;
    if (!tiReceiveByte(len)) return "";

    String s = "";
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b;
        if (!tiReceiveByte(b)) return s;
        s += (char)b;
    }
    return s;
}

void tiSendString(const String &s) {
    uint8_t len = s.length();
    if (len > 255) len = 255;

    tiSendByte(len);
    for (uint8_t i = 0; i < len; i++)
        tiSendByte(s[i]);
}

// -----------------------------
// Embedded TI Program (with menu suboptions)
// -----------------------------
const uint8_t gptProgram[] = {
    0xBB, 0x5D, 0x2A, 0x54, 0x49, 0x20, 0x47, 0x50, 0x54, 0x2A,
    0xBB, 0x5D, 0x2A, 0x41, 0x53, 0x4B, 0x2A, 0x2C, 0xAA,
    0xBB, 0xEF, 0xAA,
    0xBB, 0xE7, 0xAB,
    0xBB, 0x5D, 0xAB,
    0xBB, 0x6E, 0x32
};
const int gptProgramSize = sizeof(gptProgram);

void sendTIProgram() {
    const char name[8] = { 'G','P','T',0,0,0,0,0 };

    DBG("Sending program with menu...");

    tiSendByte(0x06);  // VAR packet
    tiSendByte(0x05);  // program type

    for (int i = 0; i < 8; i++)
        tiSendByte(name[i]);

    tiSendByte(gptProgramSize & 0xFF);
    tiSendByte(gptProgramSize >> 8);

    for (int i = 0; i < gptProgramSize; i++)
        tiSendByte(gptProgram[i]);

    String menu = "!install:Menu->Ask GPT, !wifi reset, !portal";
    tiSendString(menu);

    DBG("Program transfer complete with menu");
}

// -----------------------------
// ChatGPT request
// -----------------------------
String queryChatGPT(String prompt) {
    DBG("Sending OpenAI request");

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient https;

    https.begin(client, "https://api.openai.com/v1/chat/completions");
    https.addHeader("Content-Type", "application/json");
    https.addHeader("Authorization", "Bearer " + apiKey);

    StaticJsonDocument<1024> request;
    request["model"] = "gpt-4o-mini";
    request["max_tokens"] = 60;

    JsonArray messages = request.createNestedArray("messages");
    JsonObject sys = messages.createNestedObject();
    sys["role"] = "system";
    sys["content"] = "Reply as short as possible.";

    JsonObject msg = messages.createNestedObject();
    msg["role"] = "user";
    msg["content"] = prompt;

    String body;
    serializeJson(request, body);

    int httpCode = https.POST(body);
    if (httpCode != 200) {
        DBG("OpenAI HTTP error");
        https.end();
        return "HTTP error";
    }

    String response = https.getString();
    StaticJsonDocument<4096> doc;
    deserializeJson(doc, response);

    String reply = doc["choices"][0]["message"]["content"].as<String>();
    https.end();

    if (reply.length() > 255) reply = reply.substring(0, 255);

    return reply;
}

// -----------------------------
// WiFi Setup
// -----------------------------
void setupWifi() {
    DBG("Initializing WiFi");

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(DEVICE_HOSTNAME);

    wm.setDebugOutput(DEBUG);

    prefs.begin("config", false);
    String savedApi = prefs.getString("apikey", "");
    WiFiManagerParameter apiParam("apikey", "OpenAI API Key", savedApi.c_str(), 80);
    wm.addParameter(&apiParam);

    // Only autoConnect if credentials exist
    bool hasCreds = WiFi.SSID() != "" || WiFi.status() == WL_CONNECTED;
    bool res;

    if (prefs.getString("ssid", "").length() > 0) {
        res = wm.autoConnect("TI83-Plus");
    } else {
        DBG("No WiFi creds, starting AP portal");
        res = wm.startConfigPortal("TI83-Plus");
    }

    if (!res) {
        DBG("WiFi failed");
        ESP.restart();
    }

    apiKey = String(apiParam.getValue());
    prefs.putString("apikey", apiKey);
    prefs.end();

    DBG("WiFi connected");
    DBGF("SSID: %s\n", WiFi.SSID().c_str());
    DBGF("IP: %s\n", WiFi.localIP().toString().c_str());
}

// -----------------------------
// WiFi utilities
// -----------------------------
void resetWifi() {
    DBG("Resetting WiFi settings");
    wm.resetSettings();
    delay(2000);
    ESP.restart();
}

void startWifiPortal() {
    DBG("Starting config portal");
    wm.startConfigPortal("TI83-GPT");
    DBG("Portal closed");
}

// -----------------------------
// Setup
// -----------------------------
void setup() {
    Serial.begin(115200);
    delay(500); // allow serial to initialize
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);

    // Make TI lines idle, but non-blocking
    tipRelease();
    ringRelease();

    setupWifi();
}

// -----------------------------
// Loop
// -----------------------------
void loop() {
    if (digitalRead(WIFI_RESET_PIN) == LOW) {
        DBG("WiFi reset button held");
        delay(3000);
        if (digitalRead(WIFI_RESET_PIN) == LOW)
            resetWifi();
    }

    if (tipRead() == LOW || ringRead() == LOW) {
        DBG("TI connected, waiting for query...");
        String query = tiReceiveString();
        DBGF("Received: %s\n", query.c_str());

        if (query.startsWith("!install")) {
            sendTIProgram();

            if (query.indexOf("Ask GPT") >= 0) {
                String prompt = query.substring(query.indexOf("Ask GPT") + 8);
                String response = queryChatGPT(prompt);
                tiSendString(response);
            } else if (query.indexOf("wifi reset") >= 0) {
                resetWifi();
            } else if (query.indexOf("portal") >= 0) {
                startWifiPortal();
            }
        }
    } else {
        DBG("No TI connected, skipping receive");
        delay(500);
    }
}