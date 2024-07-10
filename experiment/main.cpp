#include <Wire.h>
#include <DFRobot_B_LUX_V30B.h>
#include "DFRobot_SHT20.h"
#include <SPI.h>
#include <LoRa.h>

#define ss 4
#define rst 5
#define dio0 2

byte LocalAddress = 0x01; //--> address of this device (Slave 1).
// byte LocalAddress = 0x03;       //--> address of this device (Slave 2).

byte Destination_Master = 0x34; //--> destination to send to Master (ESP32).

DFRobot_B_LUX_V30B lux(34, 23, 4); // sEN, SCL, SDA
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

// Hall effect sensor
const int hallPin = 14;              // Hall effect sensor pin
volatile int tipCount = 0;           // Variable to count tips
float rainfallPerTip = 0.47;         // Rainfall per tip in millimeters
float totalRainfall = 0;             // Total rainfall in millimeters
unsigned long lastHourTimestamp = 0; // Timestamp to track last hour

void IRAM_ATTR hallInterrupt()
{
    tipCount++;
}

void sendMessage(String Outgoing, byte Destination_Master)
{
    LoRa.beginPacket();             //--> start packet
    LoRa.write(Destination_Master); //--> add destination address
    LoRa.write(LocalAddress);       //--> add sender address
    LoRa.write(Outgoing.length());  //--> add payload length
    LoRa.print(Outgoing);           //--> add payload
    LoRa.endPacket();               //--> finish packet and send it
}

/*void Processing_incoming_data() {
    if (Incoming == "SDS1") {
    Message = "ESP1," + String(h) + "," + String(t);

    Serial.println();
    Serial.print("Send message to Master : ");
    Serial.println(Message);
    sendMessage(Message, Destination_Master);
    }
}*/

void setup()
{
    lux.begin();
    sht20.initSHT20();
    sht20.checkSHT20();
    Serial.begin(9600);

    // Lora
    LoRa.setPins(ss, rst, dio0);
    Serial.println();
    Serial.println("Start LoRa init...");
    if (!LoRa.begin(433E6))
    { // initialize ratio at 915 or 433 MHz
        Serial.println("LoRa init failed. Check your connections.");
        while (true)
            ; // if failed, do nothing
    }
    Serial.println("LoRa init succeeded.");

    attachInterrupt(digitalPinToInterrupt(hallPin), hallInterrupt, FALLING); // Attach interrupt to the hall sensor pin
    lastHourTimestamp = millis();                                            // Initialize the timestamp
}

void loop()
{
    // Calculate time passed since last hour
    unsigned long currentMillis = millis();
    unsigned long elapsedTime = currentMillis - lastHourTimestamp;

    // If an hour has passed, calculate rainfall
    if (elapsedTime >= 3600000)
    { // 3600000 milliseconds = 1 hour
        totalRainfall = tipCount * rainfallPerTip;
        Serial.print("Total Rainfall (1 hour) : ");
        Serial.println(totalRainfall);

        // Reset tip count and update timestamp
        tipCount = 0;
        lastHourTimestamp = currentMillis;
    }

    float luxValue = lux.lightStrengthLux();
    Serial.print("Lux: ");
    Serial.println(luxValue);

    float temp = sht20.readTemperature();
    float humid = sht20.readHumidity();
    Serial.print("Temperature: ");
    Serial.println(temp);
    Serial.print("Humidity: ");
    Serial.println(humid);

    // String Message = "ESP1" + "Brightness: " + String(luxValue) + " | " + "Temperature: " + String(temp) + " | " + "Humidity: " + String(humid) + " | " + "Rainfall: " + String(totalRainfall);
    String Message = "test";
    Serial.println();
    Serial.print("Send message to Master : ");
    Serial.println(Message);
    sendMessage(Message, Destination_Master);

    delay(2000); // Delay for other sensor readings
}
