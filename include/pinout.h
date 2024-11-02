#define RFM95_RST 15     // Reset pin for initial reset of module
#define RFM95_CS 5       // SPI channel select or slave select pin for
#define RFM95_INT 4      // SPI interrupt pin for data "ready" function
#define RFM95_FREQ 922.0 // Frequency Set function 922 MHz
// SCK, MOSI, MISO (18, 23, 19)
#define GRAVITY_ADDRESS 0x23

#define PIN_RAIN GPIO_NUM_25
#define PIN_RAIN_RTC 6

#define GPS_RXPIN 16
#define GPS_TXPIN 17
#define GPS_POWER GPIO_NUM_14

#define INTUISI_PIN GPIO_NUM_35