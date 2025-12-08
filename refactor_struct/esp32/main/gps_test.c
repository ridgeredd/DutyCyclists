#include "Sparkfun_ublox_GNSS_esp32.h"

#define GNSS_RX_PIN 16  // ESP32 GPIO 16 (connects to GNSS TX)
#define GNSS_TX_PIN 17  // ESP32 GPIO 17 (connects to GNSS RX)
#define GNSS_BAUD 9600  // Default u-blox baud rate (can also use 38400 or 115200)
SFE_UBLOX_GNSS_UART myGNSS;

int main(void) {
    setup();
    while(1) {
        loop();
    }

    return 0;
}

void setup()
{
  // Initialize USB Serial for debug output
  Serial.begin(115200);
  delay(1000); // Give Serial time to initialize
  Serial.println("SparkFun u-blox ESP32 UART Example");
  Serial.println("==================================");

  // Initialize GNSS on Serial2 with custom ESP32 pins
  // Serial2 is commonly used for external devices on ESP32
  Serial.println("Initializing GNSS module on UART...");
  Serial.print("RX Pin: GPIO");
  Serial.println(GNSS_RX_PIN);
  Serial.print("TX Pin: GPIO");
  Serial.println(GNSS_TX_PIN);
  Serial.print("Baud Rate: ");
  Serial.println(GNSS_BAUD);

  // Begin GNSS communication using ESP32 UART with custom pins
  if (myGNSS.begin(Serial2, GNSS_BAUD, GNSS_RX_PIN, GNSS_TX_PIN) == false)
  {
    Serial.println(F("u-blox GNSS module not detected on UART. Please check wiring."));
    Serial.println(F("Verify:"));
    Serial.println(F("  - GNSS TX is connected to ESP32 GPIO 16"));
    Serial.println(F("  - GNSS RX is connected to ESP32 GPIO 17"));
    Serial.println(F("  - Power and ground are connected"));
    Serial.println(F("  - Baud rate matches module setting"));
    Serial.println(F("Freezing..."));
    while (1)
    {
      delay(1000);
    }
  }

  Serial.println(F("GNSS module detected!"));
  Serial.println();

  // Pipe all NMEA sentences to the USB Serial port so we can see them
  myGNSS.setNMEAOutputPort(Serial);
  
  Serial.println(F("NMEA sentences will appear below:"));
  Serial.println(F("----------------------------------"));
}

void loop()
{
  // Check if new data is available and process bytes as they come in
  myGNSS.checkUblox();

  // Small delay to prevent overwhelming the UART bus
  delay(250);
}