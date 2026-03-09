#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <secrets.h>

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* apiKey = OPENAI_API_KEY;

#define TI_TIP 18
#define TI_RING 19

// -----------------------------
// Link line helpers
// -----------------------------

inline void tipRelease()
{
    pinMode(TI_TIP, INPUT_PULLUP);
}

inline void tipLow()
{
    pinMode(TI_TIP, OUTPUT);
    digitalWrite(TI_TIP, LOW);
}

inline void ringRelease()
{
    pinMode(TI_RING, INPUT_PULLUP);
}

inline void ringLow()
{
    pinMode(TI_RING, OUTPUT);
    digitalWrite(TI_RING, LOW);
}

inline bool tipRead()
{
    return digitalRead(TI_TIP);
}

inline bool ringRead()
{
    return digitalRead(TI_RING);
}

void tiIdle()
{
    tipRelease();
    ringRelease();
}

// -----------------------------
// TI bit send
// -----------------------------

void tiSendBit(bool bit)
{
    if (bit)
    {
        tipLow();
        while (ringRead());
        tipRelease();
        while (!ringRead());
    }
    else
    {
        ringLow();
        while (tipRead());
        ringRelease();
        while (!tipRead());
    }
}

// -----------------------------
// TI bit receive
// -----------------------------

bool tiReceiveBit()
{
    while (tipRead() && ringRead());

    if (!tipRead())
    {
        ringLow();
        while (!tipRead());
        ringRelease();
        return true;
    }
    else
    {
        tipLow();
        while (!ringRead());
        tipRelease();
        return false;
    }
}

// -----------------------------
// Byte send
// -----------------------------

void tiSendByte(uint8_t value)
{
    for (int i = 0; i < 8; i++)
    {
        tiSendBit(value & 1);
        value >>= 1;
    }
}

// -----------------------------
// Byte receive
// -----------------------------

uint8_t tiReceiveByte()
{
    uint8_t value = 0;

    for (int i = 0; i < 8; i++)
    {
        if (tiReceiveBit())
            value |= (1 << i);
    }

    return value;
}

// -----------------------------
// String receive
// -----------------------------

String tiReceiveString()
{
    uint8_t len = tiReceiveByte();

    String s = "";

    for (int i = 0; i < len; i++)
    {
        s += (char)tiReceiveByte();
    }

    return s;
}

// -----------------------------
// String send
// -----------------------------

void tiSendString(const String &s)
{
    uint8_t len = s.length();

    if (len > 255)
        len = 255;

    tiSendByte(len);

    for (int i = 0; i < len; i++)
    {
        tiSendByte(s[i]);
    }
}

// -----------------------------
// ChatGPT request
// -----------------------------

String queryChatGPT(String prompt)
{
    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient https;

    https.begin(client, "https://api.openai.com/v1/chat/completions");

    https.addHeader("Content-Type", "application/json");
    https.addHeader("Authorization", "Bearer " + String(apiKey));

    StaticJsonDocument<1024> request;

    request["model"] = "gpt-4o-mini";

    JsonArray messages = request.createNestedArray("messages");

    JsonObject message = messages.createNestedObject();
    message["role"] = "user";
    message["content"] = prompt;

    String body;
    serializeJson(request, body);

    int httpCode = https.POST(body);

    if (httpCode != 200)
    {
        https.end();
        return "HTTP error";
    }

    String response = https.getString();

    StaticJsonDocument<4096> doc;
    deserializeJson(doc, response);

    String reply =
        doc["choices"][0]["message"]["content"].as<String>();

    https.end();

    if (reply.length() > 255)
        reply = reply.substring(0,255);

    return reply;
}

// -----------------------------
// Setup
// -----------------------------

void setup()
{
    Serial.begin(115200);

    tiIdle();

    Serial.println("Connecting WiFi...");

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
}

// -----------------------------
// Main loop
// -----------------------------

void loop()
{
    Serial.println("Waiting for TI query...");

    String query = tiReceiveString();

    Serial.println("Received:");
    Serial.println(query);

    String response = queryChatGPT(query);

    Serial.println("Response:");
    Serial.println(response);

    tiSendString(response);
}