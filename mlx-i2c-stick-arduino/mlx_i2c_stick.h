#ifndef __MLX_I2C_STICK_H__
#define __MLX_I2C_STICK_H__

#include <Arduino.h>
#ifdef USE_TINYUSB
  #include <Adafruit_TinyUSB.h>
#endif
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// auto config
// ***********
//
// for a limited set of architectures only.
#ifdef ARDUINO_ARCH_RP2040
  #define HAS_EEPROM_H
  // no need for CORE1 yet!
  // #define HAS_CORE1
  #define BUFFER_COMMAND_ENABLE
  #ifdef USE_TINYUSB
    #define ENABLE_USB_MSC
  #endif // USB_TINYUSB
#endif // ARDUINO_ARCH_RP2040

// Some boards have Wire1 hooked up to the QWIIC connector...
#ifdef ARDUINO_ADAFRUIT_QTPY_RP2040
#define USE_WIRE1
#endif


// USERS Configuration
// *******************
//
// enable/disable modules:
#define DEVICE_PT100_ADS122C_ENABLE
//#undef DEVICE_PT100_ADS122C_ENABLE

//#define DEVICE_MLX90640_TEST_ENABLE
//#undef DEVICE_MLX90640_TEST_ENABLE

//#define BUFFER_COMMAND_ENABLE
#undef BUFFER_COMMAND_ENABLE


// board information
#define xstr(s) str(s)
#define str(s) #s

#if defined(__AVR_DEVICE_NAME__) and defined(USB_PRODUCT)
const char BOARD_INFO[] PROGMEM = xstr(__AVR_DEVICE_NAME__) "|" USB_PRODUCT;
#elif defined(USB_MANUFACTURER) and defined(USB_PRODUCT)
const char BOARD_INFO[] PROGMEM = USB_MANUFACTURER "|" USB_PRODUCT;
#elif defined(USB_PRODUCT)
const char BOARD_INFO[] PROGMEM = USB_PRODUCT;
#elif defined(ARDUINO_TEENSY40)
const char BOARD_INFO[] PROGMEM = "Teensy 4.0";
#elif defined(ARDUINO_TEENSY41)
const char BOARD_INFO[] PROGMEM = "Teensy 4.1";
#elif defined(CONFIG_IDF_TARGET) // for ESP32
const char BOARD_INFO[] PROGMEM = CONFIG_IDF_TARGET;
#else
const char BOARD_INFO[] PROGMEM = "Unknown";
#endif

// device configurations

// device id  constants (0..15) used to assign a fw driver and stored in 4 LSB of g_device_list.
#define DEVICE_NONE  0
#define DEVICE_90614 1
#define DEVICE_9064x 2 /* this drv figures out if it is a 90640 or a 90641 at init and then alter to it */
#define DEVICE_90640 3
#define DEVICE_90641 4
#define DEVICE_90632 5
#define DEVICE_90393 6
#define DEVICE_PT100_ADS122C 15 /* non mlx sensor starting from the upper range */

// define the name of each device.
const char DEVICE_NAMES[16][16] PROGMEM = {"No driver",    // DEVICE =  0
                                           "MLX90614",     // DEVICE =  1
                                           "MLX9064x",     // DEVICE =  2
                                           "MLX90640",     // DEVICE =  3
                                           "MLX90641",     // DEVICE =  4
                                           "MLX90632",     // DEVICE =  5
                                           "MLX90393",     // DEVICE =  6
                                           "",             // DEVICE =  7
                                           "",             // DEVICE =  8
                                           "",             // DEVICE =  9
                                           "",             // DEVICE = 10
                                           "",             // DEVICE = 11
                                           "",             // DEVICE = 12
                                           "",             // DEVICE = 13
                                           "",             // DEVICE = 14
                                           "PT100-ADS122C" // DEVICE = 15
                                         };

// HOST config
// 4 bit format field (nibble 0)
#define HOST_CFG_FORMAT_DEC  0
#define HOST_CFG_FORMAT_HEX  1
#define HOST_CFG_FORMAT_BIN  2
// 4 bit i2c field (nibble 1)
#define HOST_CFG_I2C_F100k   0
#define HOST_CFG_I2C_F400k   1
#define HOST_CFG_I2C_F1M     2
#define HOST_CFG_I2C_F50k    3
#define HOST_CFG_I2C_F20k    4
#define HOST_CFG_I2C_F10k    5


// application identifiers
#define APP_NONE              0

// mode demo operating mode (stored in g_mode)
#define MODE_INTERACTIVE 0
#define MODE_CONTINUOUS 1

// channel for external communication
#define CHANNEL_UART          0
#define CHANNEL_ETHERNET      1
#define CHANNEL_WIFI          2
#define CHANNEL_BLUETOOTH     3
#define CHANNEL_BUFFER        4

#define BUFFER_CHANNEL_SIZE (2*1024)

// global variables
extern int8_t g_mode;
extern uint16_t g_config_host;
extern int8_t g_device_list[128];
extern int8_t g_active_slave;

#ifdef BUFFER_COMMAND_ENABLE
  extern uint32_t g_channel_buffer_pos;
  extern char g_channel_buffer[BUFFER_CHANNEL_SIZE];
#endif // BUFFER_COMMAND_ENABLE

// Configure QWIIC I2C Bus:
// ************************
// By default use Wire
// Some boards have Wire1 hooked up to the QWIIC connector...
#include <Wire.h>

#ifdef USE_WIRE1
  extern TwoWire Wire1;
  #define WIRE Wire1
#else
  #define WIRE Wire
#endif

#ifdef WIRE_BUFFER_SIZE
#define READ_BUFFER_SIZE WIRE_BUFFER_SIZE
#elif defined(SERIAL_BUFFER_SIZE)
#define READ_BUFFER_SIZE SERIAL_BUFFER_SIZE
#else
#define READ_BUFFER_SIZE 32
#endif


const char *handle_cmd(uint8_t channel_mask, const char *cmd);
void send_broadcast_message(const char *msg);
void send_answer_chunk(uint8_t channel_mask, const char *answer, uint8_t terminate);

#ifdef __cplusplus
}
#endif

#endif // __MLX_I2C_STICK_H__
