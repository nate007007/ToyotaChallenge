//Copy this into an arduino sketch OR open in Platform io 
//ensure the wifi credentials are in the same folder

#include <WiFi.h>
#include "env.h"

//For the ESP32 Dev Module or ESP32C6 Dev Module

// ===== UART to PRIZM =====
//HardwareSerial PRIZM(2);
//#define RXD2 4
//#define TXD2 13

//Uncomment for ESP32C6 dev module (the esp32 with a usbc)
HardwareSerial PRIZM(1);
#define RXD1 4
#define TXD1 5

// ===== TCP Server =====
WiFiServer server(81);
WiFiClient client;

bool clientConnected = false;

const unsigned long WIFI_RETRY_WINDOW_MS = 20000;

void connectWifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_13dBm);

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("[ESP32] Connecting to WiFi SSID: ");
        Serial.println(ssid);

        WiFi.disconnect(true);
        delay(500);
        WiFi.begin(ssid, password);

        unsigned long startMs = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_RETRY_WINDOW_MS)
        {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.print("\n[ESP32] WiFi connect failed, status=");
            Serial.println(WiFi.status());
            Serial.println("[ESP32] Retrying. Check SSID/password, 2.4GHz hotspot, and that firmware was re-uploaded.");
        }
    }

    Serial.println("\n[ESP32] WiFi connected");
    Serial.print("[ESP32] IP: ");
    Serial.println(WiFi.localIP());
}

// ===== Setup =====
void setup()
{
    // INTERNAL SERIAL FOR DEBUGGING
    Serial.begin(115200);
    delay(500);

    // PRIZM MUST LOOK AT THIS EXACT BAUDRATE AND CONFIGURATION
    PRIZM.begin(38400, SERIAL_8N1, RXD1, TXD1);

    connectWifi();

    server.begin();
    Serial.println("[ESP32] TCP server started on port 81");
    Serial.println("[ESP32] Waiting for client...");
}

// ===== Main Loop =====
void loop()
{
    // Accept new client if needed
    if (!client || !client.connected())
    {
        WiFiClient newClient = server.available();
        if (newClient)
        {
            client = newClient;
            clientConnected = true;

            Serial.println("[ESP32] Client connected");

            // Send ready message
            client.println("{\"type\":\"esp32_ready\"}");
        }
    }

    // ===== TCP → PRIZM =====
    if (client && client.connected() && client.available())
    {
        while (client.available())
        {
            char c = client.read();
            PRIZM.write(c);

            // Debug
            Serial.write(c);
        }
    }

    // ===== PRIZM → TCP =====
    static String prizmBuffer = "";

    while (PRIZM.available())
    {
        char c = PRIZM.read();
        prizmBuffer += c;

        if (c == '\n')
        {
            prizmBuffer.trim();

            if (prizmBuffer.length() > 0)
            {
                Serial.print("[PRIZM → TCP] ");
                Serial.println(prizmBuffer);

                if (client && client.connected())
                {
                    client.println(prizmBuffer);
                }
            }

            prizmBuffer = "";
        }
    }

    // Small yield for stability
    delay(2);
}
