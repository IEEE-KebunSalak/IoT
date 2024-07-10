
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

#define RFM95_RST 15     // Reset pin for initial reset of module
#define RFM95_CS 5       // SPI channel select or slave select pin for
#define RFM95_INT 4      // SPI interrupt pin for data "ready" function
#define RFM95_FREQ 915.0 // Frequency Set function 915 MHz
#define SERVER_ADDRESS 2
// SCK, MOSI, MISO (18, 23, 19)
#define GRAVITY_ADDRESS 0x23
#define NODE_ID 10

// #define ULPSLEEP 4000      // amount in microseconds the ULP co-processor sleeps
#define ULPSLEEP 4000      // amount in microseconds the ULP co-processor sleeps
#define TIMEFACTOR 1000000 // factor between seconds and microseconds
#define TIMESLEEP 3600     // amount in seconds the ESP32 sleeps
#define PIN_RAIN GPIO_NUM_25
#define PIN_RAIN_RTC 6

typedef struct main
{
    uint8_t id;
    float temperature;
    float humidity;
    float lux;
    uint32_t tips;
} Payload;

// instances
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODE_ID);

uint8_t buf[4] = {0};
uint16_t data;

RTC_DATA_ATTR uint32_t bootCount = 0;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

void initSleep(uint64_t timeSleep);
static void initULP(void);
static uint32_t getPulseCount(void);
static uint32_t getShortestPulse(void);
uint8_t readReg(uint8_t reg, const void *pBuf, size_t size);
float gravity();
void sendPayload();

void setup()
{
    // Serial.begin(115200);
    setCpuFrequencyMhz(80);

    ++bootCount;

    initULP();

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    // debug yang ULP - matiin kalo udah bisa
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER || wakeup_reason == ESP_SLEEP_WAKEUP_ULP)
    {
        // LoRa setup
        pinMode(RFM95_RST, OUTPUT);
        digitalWrite(RFM95_RST, HIGH);
        delay(10);
        digitalWrite(RFM95_RST, LOW);
        delay(10);
        digitalWrite(RFM95_RST, HIGH);
        delay(10);

        if (!rf95.init())
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
        rf95.setThisAddress(NODE_ID);
        rf95.setHeaderFrom(NODE_ID);
        rf95.setHeaderTo(SERVER_ADDRESS);
        rf95.setModeTx();
        rf95.setModemConfig(RH_RF95::Bw125Cr45Sf2048);
        rf95.setPayloadCRC(true);
        manager.setRetries(0);

        Serial.println("LoRa Setup complete");

        sht20.initSHT20();
        Wire.begin();

        // delay to ensure LoRa and all sensors are ready
        delay(2500);

        Serial.println("Sensor init complete");

        sendPayload();

        rf95.sleep();
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
        Serial.println("pBuf ERROR!! : null pointer");
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
    Payload payload;
    payload.id = NODE_ID;
    payload.temperature = sht20.readTemperature();
    payload.humidity = sht20.readHumidity();
    payload.lux = gravity();
    payload.tips = getPulseCount();

    // dummy get data
    getShortestPulse();

    Serial.println("Temperature: ");
    Serial.println(payload.temperature);
    Serial.println("Humidity: ");
    Serial.println(payload.humidity);
    Serial.println("Lux: ");
    Serial.println(payload.lux);
    Serial.println("Tips: ");
    Serial.println(payload.tips);

    if (manager.sendtoWait((uint8_t *)&payload, sizeof(payload), SERVER_ADDRESS))
    {
        Serial.println("Message sent");
    }
    else
    {
        Serial.println("Message failed");
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
    if (bootCount > 1)
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
