#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Preferences.h>

#define TI_TIP 4
#define TI_RING 5

#define ANALOG_WAKE_PIN 2
#define ANALOG_THRESHOLD 1240

#define WIFI_RESET_PIN 9
#define DEVICE_HOSTNAME "TI-83 Plus"

#define SLEEP_SECONDS 60

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

bool wifiInitialized = false;

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

// -----------------------------
bool waitWhile(bool (*cond)(), uint32_t timeoutMicros = 200000) {
    uint32_t start = micros();

    while (cond()) {
        if (micros() - start > timeoutMicros)
            return false;

        delayMicroseconds(10);
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

    if (!tiReceiveByte(len))
        return "";

    String s = "";

    for (uint8_t i = 0; i < len; i++) {
        uint8_t b;

        if (!tiReceiveByte(b))
            return s;

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
// Embedded TI Program
// -----------------------------
const uint8_t gptProgram[] = {
    0xBB,0x5D,0x2A,0x54,0x49,0x20,0x47,0x50,0x54,0x2A,
    0xBB,0x5D,0x2A,0x41,0x53,0x4B,0x2A,0x2C,0xAA,
    0xBB,0xEF,0xAA,
    0xBB,0xE7,0xAB,
    0xBB,0x5D,0xAB,
    0xBB,0x6E,0x32
};

const int gptProgramSize = sizeof(gptProgram);

void sendTIProgram() {

    const char name[8] = {'G','P','T',0,0,0,0,0};

    DBG("Sending program");

    tiSendByte(0x06);
    tiSendByte(0x05);

    for(int i=0;i<8;i++)
        tiSendByte(name[i]);

    tiSendByte(gptProgramSize & 0xFF);
    tiSendByte(gptProgramSize >> 8);

    for(int i=0;i<gptProgramSize;i++)
        tiSendByte(gptProgram[i]);

    tiSendString("!install");

    DBG("Program sent");
}

// -----------------------------
// ChatGPT request
// -----------------------------
String queryChatGPT(String prompt) {

    DBG("OpenAI request");

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient https;

    https.begin(client,"https://api.openai.com/v1/chat/completions");

    https.addHeader("Content-Type","application/json");
    https.addHeader("Authorization","Bearer "+apiKey);

    StaticJsonDocument<1024> request;

    request["model"]="gpt-4o-mini";
    request["max_tokens"]=60;

    JsonArray messages=request.createNestedArray("messages");

    JsonObject sys=messages.createNestedObject();
    sys["role"]="system";
    sys["content"]="Reply as short as possible.";

    JsonObject msg=messages.createNestedObject();
    msg["role"]="user";
    msg["content"]=prompt;

    String body;
    serializeJson(request,body);

    int httpCode=https.POST(body);

    if(httpCode!=200){
        DBG("OpenAI error");
        https.end();
        return "HTTP error";
    }

    String response=https.getString();

    StaticJsonDocument<4096> doc;
    deserializeJson(doc,response);

    String reply=doc["choices"][0]["message"]["content"].as<String>();

    https.end();

    if(reply.length()>255)
        reply=reply.substring(0,255);

    return reply;
}

// -----------------------------
// WiFi
// -----------------------------
void setupWifi(){

    if(wifiInitialized) return;

    DBG("Initializing WiFi");

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(DEVICE_HOSTNAME);

    wm.setDebugOutput(DEBUG);

    prefs.begin("config",false);

    String savedApi=prefs.getString("apikey","");

    WiFiManagerParameter apiParam("apikey","OpenAI API Key",savedApi.c_str(),80);

    wm.addParameter(&apiParam);

    if(!wm.autoConnect("TI83-Plus")){
        DBG("WiFi failed");
        ESP.restart();
    }

    apiKey=String(apiParam.getValue());

    prefs.putString("apikey",apiKey);
    prefs.end();

    wifiInitialized=true;

    DBG("WiFi connected");
}

// -----------------------------
void goToSleep(){

    DBG("Entering deep sleep");

    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);

    Serial.flush();

    esp_deep_sleep_start();
}

// -----------------------------
bool analogTriggerActive(){

    int reading = analogRead(ANALOG_WAKE_PIN);

    DBGF("ADC reading: %d\n",reading);

    return reading > ANALOG_THRESHOLD;
}

// -----------------------------
// Setup
// -----------------------------
void setup(){

    Serial.begin(115200);
    delay(3000);

    Serial.println("Serial Beginning");

    pinMode(WIFI_RESET_PIN,INPUT_PULLUP);

    tipRelease();
    ringRelease();

    analogReadResolution(12);
    pinMode(ANALOG_WAKE_PIN,INPUT);

    DBG("Analog trigger active");

    setupWifi();
}

// -----------------------------
// Loop
// -----------------------------
void loop() {

    // Handle WiFi reset button
    if (digitalRead(WIFI_RESET_PIN) == LOW) {
        DBG("WiFi reset");
        delay(3000);
        if (digitalRead(WIFI_RESET_PIN) == LOW) {
            wm.resetSettings();
            ESP.restart();
        }
    }

    const int ANALOG_OFF_CYCLES = 20; // number of consecutive low readings to sleep
    int analogLowCount = 0;

    while (true) {
        int analogVal = analogRead(ANALOG_WAKE_PIN);
        DBGF("ADC reading: %d\n", analogVal);

        // TI connected?
        if (tipRead() == LOW || ringRead() == LOW) {
            DBG("TI connected");
            String query = tiReceiveString();
            DBGF("Received: %s\n", query.c_str());
            DBGF("Standard Serial: %s\n", Serial.printf("%02X ", query));
            if (query.startsWith("MFE")) {
                sendTIProgram();

                if (query.indexOf("Ask GPT") >= 0) {
                    String prompt = query.substring(query.indexOf("Ask GPT") + 8);
                    String response = queryChatGPT(prompt);
                    tiSendString(response);
                }
            }
            analogLowCount = 0; // reset counter if TI is active
        }

        // Analog line high → stay awake
        if (analogVal > ANALOG_THRESHOLD) {
            analogLowCount = 0;
        } else {
            analogLowCount++;
        }

        // If analog low for several cycles → go to deep sleep
        if (analogLowCount >= ANALOG_OFF_CYCLES) {
            DBG("Analog trigger low, entering deep sleep");
            delay(10); // small delay to finish serial prints
            goToSleep();
        }

        delay(200); // poll interval
    }
}