
/*
    Main Program
*/

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <DFRobot_SHT20.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// sleep thingies
#include "esp_sleep.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"

// ulp thingies
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "ulptool.h"

// gps module
#include <TinyGPS++.h>

// wifi
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <DNSServer.h>

#include <pinout.h>
#include <config.h>

// web config
#include <webconfig.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

typedef struct main
{
    uint8_t id;
    float temperature;
    float humidity;
    float lux;
    uint32_t tips;
} Payload;

typedef struct gpsPayload
{
    uint8_t id;
    double latitude;
    double longitude;
} GpsPayload;

typedef struct webConfig
{
    uint8_t gateway_id;
    uint8_t node_id;
    bool gps;
    bool halleffect;
} WebConfig;

// instances
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_ID);
TinyGPSPlus gps;
Payload payload;

// payloads
GpsPayload gpsPayload;

uint8_t buf[4] = {0};
uint16_t data;

RTC_DATA_ATTR bool firstBoot = true;
RTC_DATA_ATTR uint8_t gpsRetryCount = 0;
RTC_DATA_ATTR WebConfig webConfig = {
    .gateway_id = SERVER_ADDRESS, // default
    .node_id = NODE_ID,           // default
    .gps = false,
    .halleffect = false,
};

JsonDocument doc;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

void initSleep(uint64_t timeSleep);
static void initULP(void);
static uint32_t getPulseCount(void);
static uint32_t getShortestPulse(void);
uint8_t readReg(uint8_t reg, const void *pBuf, size_t size);
float gravity();
void sendPayload();
void setupCaptivePortal(DNSServer *dns, AsyncWebServer *ws);

void setup()
{
    // Serial.begin(9600);
    setCpuFrequencyMhz(80);

    // disable hold for switching power
    rtc_gpio_hold_dis(GPS_POWER);

    // add delay if first boot
    if (firstBoot)
    {
        if (SPIFFS.begin(true))
        {
            // open and try to read config
            File configFile = SPIFFS.open("/config.json", "r", true);
            if (configFile)
            {
                size_t size = configFile.size();
                std::unique_ptr<char[]> buf(new char[size]);
                configFile.readBytes(buf.get(), size);
                configFile.close();

                // parse json
                doc.clear();
                DeserializationError error = deserializeJson(doc, buf.get());
                if (error)
                {
                    Serial.println(F("Failed to read config file, using default config"));
                }
                else
                {
                    webConfig.gateway_id = doc["gateway_id"];
                    webConfig.node_id = doc["node_id"];
                    webConfig.gps = doc["gps"];
                    webConfig.halleffect = doc["halleffect"];
                }
            }

            // cleanup
            SPIFFS.end();
        }

        // if SPIFFS failed, we use default config

        // disable GPS on first boot
        rtc_gpio_deinit(GPS_POWER);
        rtc_gpio_init(GPS_POWER);
        rtc_gpio_pullup_dis(GPS_POWER);
        rtc_gpio_pulldown_en(GPS_POWER);
        rtc_gpio_set_direction(GPS_POWER, RTC_GPIO_MODE_INPUT_ONLY);

        AsyncWebServer server(80);
        DNSServer dnsServer; // DNS server to handle captive portal

        WiFi.softAP(AP_SSID, AP_PASSWORD);
        WiFi.softAPConfig(local_IP, gateway, subnet);

        setupCaptivePortal(&dnsServer, &server);

        ElegantOTA.setAutoReboot(true); // Enable auto-reboot after update
        ElegantOTA.begin(&server);      // Start ElegantOTA
        server.begin();

        uint32_t lastOTAWait = millis();

        // loop for OTA 60 seconds
        while (millis() - lastOTAWait < OTA_WAIT || WiFi.softAPgetStationNum() != 0)
        {
            dnsServer.processNextRequest();
            ElegantOTA.loop(); // ESP.restart() is called here if update is successful
        }

        // cleanup
        server.reset();
        server.end();
        dnsServer.stop();

        // turn off wifi
        WiFi.softAPdisconnect(true);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }

    initULP();

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    // debug yang ULP - matiin kalo udah bisa
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER || wakeup_reason == ESP_SLEEP_WAKEUP_ULP || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1 || firstBoot)
    {
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
            Serial.println(F("RFM95 radio init failed"));
            while (1)
                ;
        }
        Serial.println(F("RFM95 radio init OK!"));

        if (!rf95.setFrequency(RFM95_FREQ))
        {
            Serial.println(F("setFrequency failed"));
            while (1)
                ;
        }

        rf95.setTxPower(23, false);
        rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
        rf95.setPayloadCRC(true);
        manager.setRetries(0);
        manager.setThisAddress(webConfig.node_id);
        manager.setHeaderFrom(webConfig.node_id);

        Serial.println(F("LoRa Setup complete"));

        // if woke up from timer or ULP -- add first boot
        if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER || wakeup_reason == ESP_SLEEP_WAKEUP_ULP || firstBoot)
        {
            sht20.initSHT20();
            Wire.begin();

            // delay to ensure LoRa and all sensors are ready
            delay(2500);

            Serial.println(F("Sensor init complete"));

            sendPayload();
        }

        // if woke up from ext1 - antimaling trigger
        if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1 && gpsRetryCount < 3 && webConfig.gps)
        {
            // set LED_BUILTIN as output
            pinMode(LED_BUILTIN, OUTPUT);
            digitalWrite(LED_BUILTIN, HIGH);

            // power up GPS via TIP31C
            rtc_gpio_deinit(GPS_POWER);
            rtc_gpio_init(GPS_POWER);
            rtc_gpio_set_direction(GPS_POWER, RTC_GPIO_MODE_OUTPUT_ONLY);
            rtc_gpio_set_level(GPS_POWER, 1);

            Serial1.begin(9600, SERIAL_8N1, GPS_RXPIN, GPS_TXPIN);

            int count = 0;

            // try to read GPS and send it to gateway 3 times
            while (count < 3)
            {
                if (Serial1.available() <= 0)
                {
                    delay(10);
                    continue;
                }

                if (gps.encode(Serial1.read()) && gps.location.isValid())
                {
                    gpsPayload.id = webConfig.node_id;
                    gpsPayload.latitude = gps.location.lat();
                    gpsPayload.longitude = gps.location.lng();

                    // send distress signal to gateway
                    manager.sendto((uint8_t *)&gpsPayload, sizeof(gpsPayload), webConfig.gateway_id);

                    // sleep for 2 seconds for each iteration
                    delay(2000);
                    count++;
                }
            }

            digitalWrite(LED_BUILTIN, LOW);
        }

        rf95.sleep();
    }

    // disable GPS for deep sleep
    rtc_gpio_deinit(GPS_POWER);
    rtc_gpio_init(GPS_POWER);
    rtc_gpio_pullup_dis(GPS_POWER);
    rtc_gpio_pulldown_en(GPS_POWER);
    rtc_gpio_set_direction(GPS_POWER, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_hold_en(GPS_POWER);

    if (gpsRetryCount < 3 && webConfig.gps) // kalo udh 3x retry, matiin antimaling
    {
        // set interrupt for INTUISI_PIN on ext1
        esp_sleep_enable_ext1_wakeup(1ULL << INTUISI_PIN, webConfig.halleffect ? ESP_EXT1_WAKEUP_ANY_HIGH : ESP_EXT1_WAKEUP_ALL_LOW);
        rtc_gpio_pullup_dis(INTUISI_PIN);
        rtc_gpio_pulldown_en(INTUISI_PIN);

        // increase gpsRetryCount
        gpsRetryCount++;
    }

    if (firstBoot)
    {
        firstBoot = false;
    }

    initSleep(TIMESLEEP);
}

void loop()
{
    // unused loop cuz of deep sleep
}

// functions
float gravity()
{
    readReg(0x10, buf, 2);
    data = buf[0] << 8 | buf[1];
    return (((float)data) / 1.2);
}

uint8_t readReg(uint8_t reg, const void *pBuf, size_t size)
{
    if (pBuf == NULL)
    {
        Serial.println(F("pBuf ERROR!! : null pointer"));
    }
    uint8_t *_pBuf = (uint8_t *)pBuf;
    Wire.beginTransmission(GRAVITY_ADDRESS);
    Wire.write(&reg, 1);
    if (Wire.endTransmission() != 0)
    {
        return 0;
    }
    delay(20);
    Wire.requestFrom(GRAVITY_ADDRESS, size);
    for (uint16_t i = 0; i < size; i++)
    {
        _pBuf[i] = Wire.read();
    }
    return size;
}

void sendPayload()
{
    payload.id = webConfig.node_id;
    payload.temperature = sht20.readTemperature();
    payload.humidity = sht20.readHumidity();
    payload.lux = gravity();
    payload.tips = getPulseCount();

    // dummy get data
    getShortestPulse();

    Serial.println(F("Temperature: "));
    Serial.println(payload.temperature);
    Serial.println(F("Humidity: "));
    Serial.println(payload.humidity);
    Serial.println(F("Lux: "));
    Serial.println(payload.lux);
    Serial.println(F("Tips: "));
    Serial.println(payload.tips);

    if (manager.sendto((uint8_t *)&payload, sizeof(payload), webConfig.gateway_id))
    {
        Serial.println(F("Message sent"));
    }
    else
    {
        Serial.println(F("Message failed"));
    }
}

/****************\
|*    Sleep     *|
\****************/

void initSleep(uint64_t timeSleep)
{
    Serial.println(F("Preparing deep sleep now"));

    Serial.println("Set timer for sleep-wakeup every " + String(timeSleep) + " seconds");
    esp_sleep_enable_timer_wakeup(timeSleep * TIMEFACTOR);

    // biarin ulp jalan di deep sleep
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    Serial.println(F("Going into deep sleep now"));
    esp_deep_sleep_start();
}

/****************\
|*     ULP      *|
\****************/

static void initULP(void)
{
    if (!firstBoot)
        return;

    esp_err_t err_load = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err_load);

    /* GPIO used for pulse counting. */
    gpio_num_t gpio_num = PIN_RAIN;
    // assert(rtc_gpio_desc[gpio_num].reg && "GPIO used for pulse counting must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables. */
    ulp_debounce_counter = 5;
    ulp_debounce_max_count = 5;
    ulp_pulse_edge = 0;
    ulp_next_edge = 0;
    // ulp_io_number = rtc_gpio_desc[gpio_num].rtc_num; /* map from GPIO# to RTC_IO# */
    ulp_io_number = PIN_RAIN_RTC; /* map from GPIO# to RTC_IO# */

    /* Initialize selected GPIO as RTC IO, enable input, sets pullup and pulldown */
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_en(gpio_num);
    rtc_gpio_hold_en(gpio_num);

    /* Set ULP wake up period to T = 4ms.
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 24ms. */
    ulp_set_wakeup_period(0, ULPSLEEP);

    /* Start the program */
    esp_err_t err_run = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err_run);
}

static uint32_t getPulseCount(void)
{
    /* ULP program counts signal edges, convert that to the number of pulses */
    uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;

    /* In case of an odd number of edges, keep one until next time */
    ulp_edge_count = ulp_edge_count % 2;

    return pulse_count_from_ulp;
}

static uint32_t getShortestPulse(void)
{
    /* ULP program saves shortes pulse */
    uint32_t pulse_time_min = (ulp_pulse_min & UINT16_MAX) * ULPSLEEP;

    /* Reset shortest edge */
    ulp_pulse_min = 0;

    return pulse_time_min;
}

void setupCaptivePortal(DNSServer *dns, AsyncWebServer *ws)
{
    // Start DNS server to redirect all requests to the ESP32 IP
    dns->setTTL(3600);
    dns->start(53, "*", local_IP);
    // Web server routes
    ws->on("/", HTTP_GET, [](AsyncWebServerRequest *request)
           { request->redirect("/update"); }); // Redirect root to the update page

    // Required
    ws->on("/connecttest.txt", [](AsyncWebServerRequest *request)
           { request->redirect("http://logout.net"); }); // windows 11 captive portal workaround
    ws->on("/wpad.dat", [](AsyncWebServerRequest *request)
           { request->send(404); }); // Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

    // Background responses: Probably not all are Required, but some are. Others might speed things up?
    // A Tier (commonly used by modern systems)
    ws->on("/generate_204", [](AsyncWebServerRequest *request)
           { request->redirect(localIPUrl); }); // android captive portal redirect
    ws->on("/redirect", [](AsyncWebServerRequest *request)
           { request->redirect(localIPUrl); }); // microsoft redirect
    ws->on("/hotspot-detect.html", [](AsyncWebServerRequest *request)
           { request->redirect(localIPUrl); }); // apple call home
    ws->on("/canonical.html", [](AsyncWebServerRequest *request)
           { request->redirect(localIPUrl); }); // firefox captive portal call home
    ws->on("/success.txt", [](AsyncWebServerRequest *request)
           { request->send(200); }); // firefox captive portal call home
    ws->on("/ncsi.txt", [](AsyncWebServerRequest *request)
           { request->redirect(localIPUrl); }); // windows call home

    // return 404 to webpage icon
    ws->on("/favicon.ico", [](AsyncWebServerRequest *request)
           { request->send(404); }); // webpage icon

    ws->onNotFound([](AsyncWebServerRequest *request)
                   { request->redirect(localIPUrl); }); // redirect anything

    // serve html string
    ws->on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
           { request->send(200, "text/html", index_html); });

    ws->on("/saveconfig", [](AsyncWebServerRequest *request)
           {
        if (request->hasParam("gateway_id", true))
        {
            webConfig.gateway_id = request->getParam("gateway_id", true)->value().toInt();
        }
        if (request->hasParam("node_id", true))
        {
            webConfig.node_id = request->getParam("node_id", true)->value().toInt();
        }

            webConfig.gps = request->hasParam("gps", true);
            webConfig.halleffect = request->hasParam("halleffect", true);

        // save config to SPIFFS
        if (SPIFFS.begin(true))
        {
            File configFile = SPIFFS.open("/config.json", "w");
            if (configFile)
            {
                doc.clear();
                doc["gateway_id"] = webConfig.gateway_id;
                doc["node_id"] = webConfig.node_id;
                doc["gps"] = webConfig.gps;
                doc["halleffect"] = webConfig.halleffect;

                serializeJson(doc, configFile);
                configFile.close();
            }
            SPIFFS.end();
        }

        request->send(200, "text/plain", "Configuration saved..."); });

    // read current config /config/read
    ws->on("/readconfig", HTTP_GET, [](AsyncWebServerRequest *request)
           { request->send(200, "application/json", doc.as<String>()); });
}
