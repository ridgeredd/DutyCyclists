/*
  ESP32-optimized UART version of SparkFun u-blox GNSS Arduino Library
  
  This version is specifically designed for ESP32 UART communication with u-blox GNSS modules
  Supports ESP32's multiple hardware serial ports with configurable RX/TX pins
  
  Based on SparkFun_u-blox_GNSS_v3 by Paul Clark @ SparkFun Electronics
  https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  MIT License - Copyright (c) 2018 SparkFun Electronics
*/

#pragma once

#ifdef ESP32
  #include <HardwareSerial.h>
#endif

#include "u-blox_GNSS.h"
#include "u-blox_external_typedefs.h"
#include "sfe_bus.h"

// Primary ESP32 UART-based GNSS class
class SFE_UBLOX_GNSS_UART : public DevUBLOXGNSS
{
public:
  SFE_UBLOX_GNSS_UART() { _commType = COMM_TYPE_SERIAL; }

  ///////////////////////////////////////////////////////////////////////
  // begin() - ESP32 UART/Serial initialization
  //
  // Initialize the GNSS library and connect via UART/Serial port.
  // This method must be called before any other interaction with the device.
  //
  // Returns true if "signs of life" detected: any valid UBX packet or NMEA header
  //
  //  Parameter     Description
  //  -----------   -----------------------------------------------------
  //  serialPort    Reference to HardwareSerial (Serial, Serial1, Serial2)
  //  baudRate      UART baud rate (default: 9600, common: 38400, 115200)
  //  rxPin         ESP32 GPIO pin for RX (default: -1, uses default pin)
  //  txPin         ESP32 GPIO pin for TX (default: -1, uses default pin)
  //  maxWait       Maximum wait time in ms for response (default: 1100)
  //  assumeSuccess Skip detection check if true (default: false)
  //  retval        true on success, false on startup failure
  //
  ///////////////////////////////////////////////////////////////////////

#ifdef ESP32
  // Version 1: ESP32 with custom RX/TX pins
  bool begin(HardwareSerial &serialPort, uint32_t baudRate, int8_t rxPin, int8_t txPin,
             uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Initialize the ESP32 serial port with custom pins
    // ESP32 allows remapping UART pins to almost any GPIO
    serialPort.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    
    // Small delay to allow UART to stabilize
    delay(100);
    
    // Setup Serial object and pass into the superclass
    setCommunicationBus(_serialBus);

    // Initialize the Serial bus class
    _serialBus.init(serialPort);

    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

  // Version 2: ESP32 with default pins (uses hardware defaults)
  bool begin(HardwareSerial &serialPort, uint32_t baudRate = 9600,
             uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Initialize the serial port at specified baud rate with default pins
    serialPort.begin(baudRate);
    
    // Small delay to allow UART to stabilize
    delay(100);
    
    // Setup Serial object and pass into the superclass
    setCommunicationBus(_serialBus);

    // Initialize the Serial bus class
    _serialBus.init(serialPort);

    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }
#else
  // Non-ESP32 version: Basic UART initialization
  bool begin(HardwareSerial &serialPort, uint32_t baudRate = 9600, 
             uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    serialPort.begin(baudRate);
    delay(100);
    
    setCommunicationBus(_serialBus);
    _serialBus.init(serialPort);
    
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }
#endif

  // Version 3: Use pre-initialized serial port (for custom configurations)
  bool begin(Stream &serialPort, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, 
             bool assumeSuccess = false)
  {
    // Setup Serial object and pass into the superclass
    setCommunicationBus(_serialBus);

    // Initialize the Serial bus class
    _serialBus.init(serialPort);

    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

#ifdef ESP32
  ///////////////////////////////////////////////////////////////////////
  // Utility function to change UART baud rate on ESP32
  // Can optionally remap pins during the change
  ///////////////////////////////////////////////////////////////////////
  bool changeBaudRate(HardwareSerial &serialPort, uint32_t newBaudRate, 
                      int8_t rxPin = -1, int8_t txPin = -1)
  {
    serialPort.end();
    delay(100);
    
    if (rxPin >= 0 && txPin >= 0)
    {
      serialPort.begin(newBaudRate, SERIAL_8N1, rxPin, txPin);
    }
    else
    {
      serialPort.begin(newBaudRate);
    }
    
    delay(100);
    return true;
  }
#else
  bool changeBaudRate(HardwareSerial &serialPort, uint32_t newBaudRate)
  {
    serialPort.end();
    delay(100);
    serialPort.begin(newBaudRate);
    delay(100);
    return true;
  }
#endif

private:
  // Serial bus class
  SparkFun_UBLOX_GNSS::SfeSerial _serialBus;
};

// Backward compatibility alias
using SFE_UBLOX_GNSS_SERIAL = SFE_UBLOX_GNSS_UART;

// Simplified ESP32 UART-only class (no I2C/SPI options)
class SFE_UBLOX_GNSS : public DevUBLOXGNSS
{
public:
  SFE_UBLOX_GNSS() { _commType = COMM_TYPE_SERIAL; }

#ifdef ESP32
  // ESP32: UART with custom RX/TX pins
  bool begin(HardwareSerial &serialPort, uint32_t baudRate, int8_t rxPin, int8_t txPin,
             uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    serialPort.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    delay(100);
    
    setCommunicationBus(_serialBus);
    _serialBus.init(serialPort);
    
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

  // ESP32: UART with default pins
  bool begin(HardwareSerial &serialPort, uint32_t baudRate = 9600, 
             uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    serialPort.begin(baudRate);
    delay(100);
    
    setCommunicationBus(_serialBus);
    _serialBus.init(serialPort);
    
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }
#else
  // Non-ESP32: Simple UART begin
  bool begin(HardwareSerial &serialPort, uint32_t baudRate = 9600, 
             uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    serialPort.begin(baudRate);
    delay(100);
    
    setCommunicationBus(_serialBus);
    _serialBus.init(serialPort);
    
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }
#endif

  // Use pre-configured Stream
  bool begin(Stream &serialPort, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, 
             bool assumeSuccess = false)
  {
    setCommunicationBus(_serialBus);
    _serialBus.init(serialPort);
    
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

private:
  SparkFun_UBLOX_GNSS::SfeSerial _serialBus;
};

/*
  ESP32 USAGE EXAMPLES:
  
  // Example 1: ESP32 with custom GPIO pins (Serial2)
  // Common setup: RX=GPIO16, TX=GPIO17
  SFE_UBLOX_GNSS_UART myGNSS;
  if (myGNSS.begin(Serial2, 38400, 16, 17)) {
    Serial.println("GNSS connected on GPIO16/17!");
  }
  
  // Example 2: ESP32 Serial1 with custom pins
  // Common setup: RX=GPIO9, TX=GPIO10
  SFE_UBLOX_GNSS myGNSS;
  if (myGNSS.begin(Serial1, 9600, 9, 10)) {
    Serial.println("GNSS connected on GPIO9/10!");
  }
  
  // Example 3: Using default pins (hardware UART defaults)
  SFE_UBLOX_GNSS_UART myGNSS;
  if (myGNSS.begin(Serial2, 115200)) {
    Serial.println("GNSS connected with default pins!");
  }
  
  // Example 4: ESP32-S3/C3 with different GPIO assignments
  SFE_UBLOX_GNSS myGNSS;
  if (myGNSS.begin(Serial1, 38400, 18, 17)) {
    Serial.println("GNSS on ESP32-S3 GPIO18/17!");
  }
  
  // Example 5: Change baud rate and remap pins on the fly
  SFE_UBLOX_GNSS_UART myGNSS;
  myGNSS.begin(Serial2, 9600, 16, 17);
  // Later, switch to higher speed
  myGNSS.changeBaudRate(Serial2, 115200, 16, 17);
  
  // Common ESP32 UART Pin Configurations:
  // --------------------------------------
  // ESP32 (Original):
  //   Serial0 (USB): RX=3,  TX=1   (used for programming/debug)
  //   Serial1:       RX=9,  TX=10  (can be remapped)
  //   Serial2:       RX=16, TX=17  (can be remapped)
  //
  // ESP32-S3:
  //   Serial0 (USB): RX=44, TX=43  (USB-Serial-JTAG)
  //   Serial1:       Any GPIO (flexible)
  //   Serial2:       Any GPIO (flexible)
  //
  // ESP32-C3:
  //   Serial0 (USB): RX=20, TX=21  (USB-Serial-JTAG)
  //   Serial1:       Any GPIO (flexible)
  //
  // Note: ESP32 allows most GPIOs to be used for UART RX/TX
  //       Avoid using GPIO 6-11 on original ESP32 (flash pins)
*/