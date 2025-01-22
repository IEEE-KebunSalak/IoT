
/*
    LoRa Tester
*/

#include "Arduino.h"
#include <SPI.h>
// #include <DFRobot_SHT20.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <WebConfig.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <Update.h>

#define AP_SSID "testbed-32"
#define AP_PASSWORD "intuisiricat123!"
IPAddress local_IP(1, 1, 1, 1);     // Static IP
IPAddress gateway(1, 1, 1, 1);      // Gateway (same as IP for standalone AP)
IPAddress subnet(255, 255, 255, 0); // Subnet mask

#define RFM95_RST 15     // Reset pin for initial reset of module
#define RFM95_CS 5       // SPI channel select or slave select pin for
#define RFM95_INT 4      // SPI interrupt pin for data "ready" function
#define RFM95_FREQ 922.0 // Frequency Set function 915 MHz
// SCK, MOSI, MISO (18, 23, 19)
#define NODE_ID 102
#define SERVER_ADDRESS 2

typedef struct main
{
    uint8_t id;
    float temperature;
    float humidity;
    float lux;
    uint32_t tips;
} Payload;

// instances
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_ID);

String params = "["
                "{"
                "'name':'gw_id',"
                "'label':'Gateway ID',"
                "'type':" +
                String(INPUTNUMBER) + ","
                                      "'min':0,'max':20,"
                                      "'default':'2'"
                                      "},"
                                      "{"
                                      "'name':'node_id',"
                                      "'label':'Node ID',"
                                      "'type':" +
                String(INPUTNUMBER) + ","
                                      "'min':0,'max':100,"
                                      "'default':'1'"
                                      "}"
                                      "]";

WebServer server;
WebConfig conf;

// prototype
void sendPayload();

void handleRoot()
{
    conf.handleFormRequest(&server);
}

void setup()
{

    Serial.begin(9600);
    // setCpuFrequencyMhz(80);

    conf.setDescription(params);
    conf.readConfig();

    WiFi.softAP(AP_SSID, AP_PASSWORD);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    char dns[30];
    sprintf(dns, "%s.local", conf.getApName());
    if (MDNS.begin(dns))
    {
        Serial.println("MDNS responder gestartet");
    }
    server.on("/", handleRoot);
    server.begin(80);

    uint32_t lastOTAWait = millis();

    while (millis() - lastOTAWait < 20000 || WiFi.softAPgetStationNum() != 0)
    {
        server.handleClient();
        delay(10);
    }

    // cleanup
    server.stop();

    // turn off wifi
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    pinMode(LED_BUILTIN, OUTPUT);

    // LoRa setup
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if (!manager.init())
    {
        Serial.println("RFM95 radio init failed");
        while (1)
            ;
    }

    Serial.println("RFM95 radio init OK!");

    if (!rf95.setFrequency(RFM95_FREQ))
    {
        Serial.println("setFrequency failed");
        while (1)
            ;
    }

    rf95.setTxPower(23, false);
    // rf95.setHeaderFrom(NODE_ID);
    // rf95.setHeaderTo(SERVER_ADDRESS);
    // rf95.setModeTx();
    rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
    rf95.setPayloadCRC(true);
    manager.setRetries(0);
    manager.setThisAddress(conf.getInt("node_id"));

    Serial.println("LoRa Setup complete");

    digitalWrite(LED_BUILTIN, HIGH);
    sendPayload();
    Serial.println("[Payload]: SENT TO " + String(conf.getInt("gw_id")));
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
}

void sendPayload()
{

    Payload payload;

    for (size_t i = 1; i <= 50; i++)
    {
        payload.id = conf.getInt("node_id");
        payload.temperature = i * 1.87367;
        payload.humidity = i * 1.75689;
        payload.lux = i * 554.231;
        payload.tips = i;

        if (manager.sendto((uint8_t *)&payload, sizeof(payload), conf.getInt("gw_id")))
        {
            Serial.println("Message sent");
        }
        else
        {
            Serial.println("Message failed");
        }

        delay(5000);
    }
}