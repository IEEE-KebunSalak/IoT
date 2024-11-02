#define NODE_ID 101
#define SERVER_ADDRESS 100

#define ULPSLEEP 4000      // amount in microseconds the ULP co-processor sleeps
#define TIMEFACTOR 1000000 // factor between seconds and microseconds
// #define TIMESLEEP 3600     // amount in seconds the ESP32 sleeps
#define TIMESLEEP 15 // amount in seconds the ESP32 sleeps

#define AP_SSID "NODE-OTA"
#define AP_PASSWORD "kulonuwun"
#define OTA_WAIT 60000

// Define static IP configuration
IPAddress local_IP(1, 1, 1, 1);     // Static IP
IPAddress gateway(1, 1, 1, 1);      // Gateway (same as IP for standalone AP)
IPAddress subnet(255, 255, 255, 0); // Subnet mask

const String localIPUrl = "http://1.1.1.1";