#ifndef DEVICE_SETUP_H
#define DEVICE_SETUP_H

// This header file defines the pin assignments and configurations for the project.

/////////////////////////// I2C ///////////////////////////////////////////////
// I2C pins used for communication between the ESP32 and the ESP8266.
// Modify these pins based on the target platform (ESP32 or ESP8266).
// For ESP32:
#if defined(ARDUINO_ARCH_ESP32)
#define SDA_PIN 21       // GPIO pin used for I2C SDA (data)
#define SCL_PIN 22       // GPIO pin used for I2C SCL (clock)
#define INTERRUPT_PIN GPIO_NUM_2   // GPIO pin used for I2C interrupt
#endif
// For ESP8266:
#if defined(ARDUINO_ARCH_ESP8266)
#define SDA_PIN D2       // GPIO pin used for I2C SDA (data)
#define SCL_PIN D1       // GPIO pin used for I2C SCL (clock)
#define INTERRUPT_PIN D3 // GPIO pin used for I2C interrupt
#endif

// I2C address configuration for the ESP32 and ESP8266 in the I2C communication.
#define I2C_SlaveAddress 0x04   // I2C slave address
#define I2C_MasterAddress 0x01  // I2C master address
#define Standard 100000        // I2C Standard speed
#define FastMode 400000         // I2C Fast Mode speed

/////////////////////////// RBDDIMMER ///////////////////////////////////////////////
// Pin used to control the dimmer for the lamps.
// Modify these pins based on the target platform (ESP32 or ESP8266).
// For ESP32:
#if defined(ARDUINO_ARCH_ESP32)
#define CHANNEL_L1 15   // GPIO pin used for dimmer control of lamp 1
#define CHANNEL_L2 18   // GPIO pin used for dimmer control of lamp 2
#define ZEROCROSS 4     // GPIO pin used for zero-crossing detection
#endif
// For ESP8266:
#if defined(ARDUINO_ARCH_ESP8266)
#define CHANNEL_L1 D10  // GPIO pin used for dimmer control of lamp 1
#define CHANNEL_L2 D4   // GPIO pin used for dimmer control of lamp 2
#define ZEROCROSS D7    // GPIO pin used for zero-crossing detection
#endif

/////////////////////////// MAX6675 ///////////////////////////////////////////////
#if defined(ARDUINO_ARCH_ESP32)
int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;
#endif
#if defined(ARDUINO_ARCH_ESP8266)
int thermoDO = D6;
int thermoCS = D8;
int thermoCLK = D5;
#endif


#endif // DEVICE_SETUP_H
