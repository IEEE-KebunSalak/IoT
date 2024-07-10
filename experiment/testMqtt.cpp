#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// logging enum
enum class LogType
{
  INFO,
  WARNING,
  ERROR
};

// I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// rain sensor pin
#define PIN_RAIN_SENSOR 36

// interval for data logging in milliseconds
#define LOG_INTERVAL 5000

// WIFI
#define WIFI_SSID "smartroom"
#define WIFI_PASSWORD "smartroom519"

// MQTT
#define MQTT_SERVER "broker.emqx.io"
#define MQTT_PORT 1883
// #define MQTT_USERNAME "usermqtt"
// #define MQTT_PASSWORD "Testing123"

// previous millis
unsigned long previousMillis = 0;

// Wifi and MQTT client
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// setup functions
void s_Wifi();
void s_Mqtt();

// general functions
void log(LogType type, String message);
void publishData(uint16_t light, float temp, float humid, uint16_t rain);

// read sensor functions
uint16_t r_LightSensor();
std::tuple<float, float> r_TempHumidSensor();
uint16_t r_RainSensor();

void setup()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(9600);
  log(LogType::INFO, "I2C and Serial communication started.");

  s_Wifi();
  s_Mqtt();
}

void loop()
{
  client.loop();

  if (millis() - previousMillis >= LOG_INTERVAL)
  {
    previousMillis = millis();
    uint16_t light = r_LightSensor();
    log(LogType::INFO, "Light Sensor: " + String(light) + " lux");

    auto [temp, humid] = r_TempHumidSensor();
    log(LogType::INFO, "Temperature: " + String(temp) + " C");
    log(LogType::INFO, "Humidity: " + String(humid) + " %");

    uint16_t rain = r_RainSensor();
    log(LogType::INFO, "Rain Sensor: " + String(rain));

    publishData(light, temp, humid, rain);
  }
}

uint16_t r_LightSensor()
{
  uint16_t lux = 0;
  Wire.beginTransmission(0x23);
  Wire.write(0x10);
  if (Wire.endTransmission() == 0)
  {
    delay(180);
    Wire.requestFrom(0x23, 2);
    if (Wire.available() >= 2)
    {
      lux = Wire.read() << 8 | Wire.read();
    }
  }
  return lux;
}

std::tuple<float, float> r_TempHumidSensor()
{
  uint8_t data[4];
  float temp = 0.0, humid = 0.0;
  Wire.beginTransmission(0x40);
  Wire.write(0x00);
  if (Wire.endTransmission() == 0)
  {
    delay(50);
    Wire.requestFrom(0x40, 4);
    if (Wire.available() >= 4)
    {
      for (int i = 0; i < 4; i++)
      {
        data[i] = Wire.read();
      }
      temp = (data[0] * 256 + data[1]) / 65536.0 * 165.0 - 40.0;
      humid = (data[2] * 256 + data[3]) / 65536.0 * 100.0;
    }
  }
  return std::make_tuple(temp, humid);
}

uint16_t r_RainSensor()
{
  return analogRead(PIN_RAIN_SENSOR);
}

void log(LogType type, String message)
{
  switch (type)
  {
  case LogType::INFO:
    Serial.println("[INFO]: " + message);
    break;
  case LogType::WARNING:
    Serial.println("[WARNING]: " + message);
    break;
  case LogType::ERROR:
    Serial.println("[ERROR]: " + message);
    break;
  default:
    Serial.println("[ERROR]: " + message);
    break;
  }
}

void s_Wifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    log(LogType::INFO, "Connecting to WiFi...");
  }
  log(LogType::INFO, "Connected to WiFi.");
}

void s_Mqtt()
{
  client.setServer(MQTT_SERVER, MQTT_PORT);
  while (!client.connected())
  {
    log(LogType::INFO, "Connecting to MQTT...");
    if (client.connect("proto-device-1"))
    {
      log(LogType::INFO, "Connected to MQTT.");
    }
    else
    {
      log(LogType::ERROR, "Failed to connect to MQTT. Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void publishData(uint16_t light, float temp, float humid, uint16_t rain)
{
  String payload = "{\"light\": " + String(light) + ", \"temp\": " + String(temp) + ", \"humid\": " + String(humid) + ", \"rain\": " + String(rain) + "}";
  client.publish("salak", payload.c_str());
}