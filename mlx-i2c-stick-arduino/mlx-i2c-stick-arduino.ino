#include "mlx_i2c_stick.h"

#ifdef HAS_EEPROM_H
#include <EEPROM.h>
#endif

#ifdef ENABLE_USB_MSC // Mass Storage device Class
  #include <SPI.h>
  #include <SdFat.h>
  #include <Adafruit_SPIFlash.h>
  #include <Adafruit_TinyUSB.h>
#endif // ENABLE_USB_MSC // Mass Storage device Class

#include "mlx_i2c_stick_cmd.h"
#include <stdlib.h>

#include "mlx90632_hal.h"

const char FW_VERSION[] PROGMEM = "V1.1.0";


#ifdef ENABLE_USB_MSC // Mass Storage device Class
  // Un-comment to run example with custom SPI and SS e.g with FRAM breakout
  // #define CUSTOM_CS   A5
  // #define CUSTOM_SPI  SPI

  #if defined(CUSTOM_CS) && defined(CUSTOM_SPI)
    Adafruit_FlashTransport_SPI flashTransport(CUSTOM_CS, CUSTOM_SPI);

  #elif defined(ARDUINO_ARCH_ESP32)
    // ESP32 use same flash device that store code.
    // Therefore there is no need to specify the SPI and SS
    Adafruit_FlashTransport_ESP32 flashTransport;

  #elif defined(ARDUINO_ARCH_RP2040)
    // RP2040 use same flash device that store code.
    // Therefore there is no need to specify the SPI and SS
    // Use default (no-args) constructor to be compatible with CircuitPython partition scheme
    Adafruit_FlashTransport_RP2040 flashTransport;

    // For generic usage:
    //    Adafruit_FlashTransport_RP2040 flashTransport(start_address, size)
    // If start_address and size are both 0, value that match filesystem setting in
    // 'Tools->Flash Size' menu selection will be used

  #else
    // On-board external flash (QSPI or SPI) macros should already
    // defined in your board variant if supported
    // - EXTERNAL_FLASH_USE_QSPI
    // - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
    #if defined(EXTERNAL_FLASH_USE_QSPI)
      Adafruit_FlashTransport_QSPI flashTransport;

    #elif defined(EXTERNAL_FLASH_USE_SPI)
      Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);

    #else
      #error No QSPI/SPI flash are defined on your board variant.h !
    #endif
  #endif


  Adafruit_SPIFlash flash(&flashTransport);

  // file system object from SdFat
  FatFileSystem fatfs;

  FatFile root;
  FatFile file;

  // USB Mass Storage object
  Adafruit_USBD_MSC usb_msc;

  // Check if flash is formatted
  bool fs_formatted;

  // Set to true when PC write to flash
  bool fs_changed;

#endif // ENABLE_USB_MSC // Mass Storage device Class



// global variables
int8_t g_mode;
int8_t g_device_list[128];
int8_t g_active_slave;
uint16_t g_config_host;

int8_t g_app_id = APP_NONE;

int8_t g_state = 0;
uint8_t g_channel_mask = 
    (1U<<CHANNEL_UART)
  | (1U<<CHANNEL_WIFI)
  | (1U<<CHANNEL_ETHERNET)
  | (1U<<CHANNEL_BLUETOOTH)
#ifdef BUFFER_COMMAND_ENABLE
  | (1U<<CHANNEL_BUFFER)
#endif
  ;

#ifdef BUFFER_COMMAND_ENABLE
  uint32_t g_channel_buffer_pos;
  char g_channel_buffer[BUFFER_CHANNEL_SIZE];
#endif 

void uart_welcome()
{
  Serial.println("mlx-i2c-stick booted: '?' for help");
  g_channel_mask |= (1U<<CHANNEL_UART);
  g_state = 1;
}


#ifdef HAS_CORE1
void main_core1()
{
  while(true)
  {}
}
#endif // HAS_CORE1


void
setup()
{
#ifdef ENABLE_USB_MSC

  flash.begin();

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Melexis", "MLX I2C STICK", "1.0");

  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size()/512, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  usb_msc.begin();

  // Init file system on the flash
  fs_formatted = fatfs.begin(&flash);
#endif // ENABLE_USB_MSC


#ifdef HAS_CORE1
  multicore_launch_core1(main_core1);
  if (rp2040.fifo.available())
  {}
#endif // HAS_CORE1

  Serial.begin(1000000);
  g_channel_mask &= ~(1U<<CHANNEL_UART);

  WIRE.setClock(100000); // RP2040 requires first to set the clock
  WIRE.begin();
  WIRE.setClock(100000); // NRF52840 requires first begin, then set clock.

  // default link slave address vs device driver!
  memset(g_device_list, 0, sizeof(g_device_list));

#ifdef HAS_EEPROM_H
  EEPROM.begin(512);
  for (uint8_t i=0; i<128; i+=2)
  {
    uint8_t d = EEPROM.read(i>>1);
    g_device_list[i] = d & 0x0F;
    g_device_list[i+1] = (d >> 4) & 0x0F;
  }
#endif

  // defaults will ALWAYS be defaults after boot!
  g_device_list[0x5A] = DEVICE_90614;
  g_device_list[0x3E] = DEVICE_90614; // mlx90616
  g_device_list[0x33] = DEVICE_9064x; // could be either 90640 or 90641
  g_device_list[0x3A] = DEVICE_90632;

  g_device_list[0x40] = DEVICE_PT100_ADS122C; // A0=GND & A1=GND
  g_device_list[0x41] = DEVICE_PT100_ADS122C; // A0=VDD & A1=GND
  //g_device_list[0x44] = DEVICE_PT100_ADS122C; // A0=GND & A1=VDD
  g_device_list[0x45] = DEVICE_PT100_ADS122C; // A0=VDD & A1=VDD default w/o soldering

  uint8_t channel_mask = 0; // run quitely...
  handle_cmd(channel_mask, "scan"); // scan at startup!
  if (Serial) uart_welcome();
}


void
loop()
{
  if (g_state == 0)
  {
    if (Serial)
    {
      uart_welcome();
    }
  } else
  {
    int_uart();
  }
  //int_ethernet();
  //int_ble();
  //int_clue_button_display();
  if (g_mode == MODE_CONTINUOUS)
  {
    handle_continuous_mode();
  }
  handle_applications();
}


#ifdef ENABLE_USB_MSC
// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  // digitalWrite(LED_BUILTIN, HIGH);

  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  fs_changed = true;

  // digitalWrite(LED_BUILTIN, LOW);
}

#endif // ENABLE_USB_MSC


const char *
handle_cmd(uint8_t channel_mask, const char *cmd)
{
  const char *this_cmd;

  this_cmd = "mlx"; // MeLeXis test command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    send_answer_chunk(channel_mask, "mlx:MLX I2C STICK", 1);
    send_answer_chunk(channel_mask, "mlx:=============", 1);
    send_answer_chunk(channel_mask, "mlx:", 1);
    send_answer_chunk(channel_mask, "mlx:Melexis Inspired Engineering", 1);
    send_answer_chunk(channel_mask, "mlx:", 1);
    send_answer_chunk(channel_mask, "mlx:hit '?' for help", 1);
    return NULL;
  }

  this_cmd = "fv"; // get Firmware Version
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    char buf[16]; memset(buf, 0, sizeof(buf));
    send_answer_chunk(channel_mask, "fv:", 0);
    // read back chars from FLASH
    for (byte k = 0; k < strlen_P(FW_VERSION); k++)
    {
      buf[k] = pgm_read_byte_near(FW_VERSION + k);
    }
    send_answer_chunk(channel_mask, buf, 1);
    return NULL;
  }

  this_cmd = "bi"; // Board Information command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    char buf[64]; memset(buf, 0, sizeof(buf));
    send_answer_chunk(channel_mask, "bi:", 0);
    // read back chars from FLASH
    for (byte k = 0; k < strlen_P(BOARD_INFO); k++)
    {
      buf[k] = pgm_read_byte_near(BOARD_INFO + k);
    }
    send_answer_chunk(channel_mask, buf, 1);
    return NULL;
  }

  this_cmd = "ch"; // Config Hub command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    handle_cmd_ch(channel_mask, cmd+strlen(this_cmd));
    return NULL;
  }

  this_cmd = "+ch:"; // Config Hub command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    handle_cmd_ch_write(channel_mask, cmd+strlen(this_cmd));
    return NULL;
  }

  this_cmd = "help"; // help command, same as task ?
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    return handle_task(channel_mask, '?');
  }

  this_cmd = "sos"; // SOS command, get more help on a specific command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    handle_command_sos(channel_mask, cmd+strlen(this_cmd));
    return NULL;
  }

#ifdef HAS_EEPROM_H
  this_cmd = "store"; // store hub cfg to EEPROM
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    send_answer_chunk(channel_mask, "store:", 0);
    for (uint8_t i=0; i<128; i+=2)
    {
      uint8_t d;
      d = g_device_list[i] & 0x0F;
      d |= ((g_device_list[i+1] << 4) & 0xF0);
      EEPROM.write(i>>1, d);
    }

    EEPROM.commit();
    send_answer_chunk(channel_mask, "OK", 1);
    return NULL;
  }
#endif // HAS_EEPROM_H

#ifdef BUFFER_COMMAND_ENABLE
  this_cmd = "buf"; // buffer command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    send_answer_chunk(channel_mask, g_channel_buffer, 1);
    return NULL;
  }
#endif // BUFFER_COMMAND_ENABLE

  this_cmd = "i2c:"; // raw I2C command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    char buf[128]; memset(buf, 0, sizeof(buf));
    send_answer_chunk(channel_mask, this_cmd, 0);

    int16_t sa = atohex8(cmd+strlen(this_cmd));
    if ((sa < 0) || (sa >= 0x80) || (cmd[6] != ':'))
    {
      send_answer_chunk(channel_mask, "ERROR:syntax error in slave address", 1);
      return NULL;
    }

    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, ":", 0);

    if (cmd[7] == 'R')
    {
      // example: read 1 byte from slave address 0x33 (sa is 7bit)
      // cmd: i2c:33:R1
      // cmd: 012345678
      send_answer_chunk(channel_mask, "R:", 0);
      uint16_t read_n_bytes = atoi(&cmd[8]);

      // do the real thing!
      WIRE.endTransmission();
      delayMicroseconds(5);

      WIRE.beginTransmission((uint8_t)sa);
      WIRE.requestFrom((uint8_t)sa, uint8_t(read_n_bytes), uint8_t(true));

      for (uint8_t i=0; i<read_n_bytes; i++)
      {
        uint8_t data = WIRE.read();

        uint8_to_hex(buf, data);
        send_answer_chunk(channel_mask, buf, 0);
      }

      byte result = WIRE.endTransmission();     // stop transmitting
#ifdef ARDUINO_ARCH_RP2040
      if (result == 4) result = 0; // ignore error=4 ('other error', but I can't seem to find anything wrong; only on this MCU platform)
#endif
      if (result == 0) // SUCCESS
      {
        send_answer_chunk(channel_mask, ":OK", 1);
      } else
      {
        send_answer_chunk(channel_mask, ":FAIL:", 0);
        uint8_to_hex(buf, result);
        send_answer_chunk(channel_mask, buf, 1);
      }
    } else if (cmd[7] == 'W')
    {
      // example: write 4 bytes to slave address 0x3A (sa is 7bit)
      // cmd: i2c:3A:W30045A69
      send_answer_chunk(channel_mask, "W", 0);
      WIRE.endTransmission();
      delayMicroseconds(5);

      WIRE.beginTransmission((uint8_t)sa);

      uint8_t i=0;
      for(; i<255; i++)
      {
        int16_t data = atohex8(cmd+8+i*2);
        if (data < 0) break;
        WIRE.write(data);
        uint8_to_hex(buf, data);
        send_answer_chunk(channel_mask, buf, 0);
      }
      if (cmd[8+i*2] != 'R')
      {
        byte result = WIRE.endTransmission(true);     // normal stop condition.
#ifdef ARDUINO_ARCH_RP2040
        if (result == 4) result = 0; // ignore error=4 ('other error', but I can't seem to find anything wrong; only on this MCU platform)
#endif
        if (result == 0) // SUCCESS
        {
          send_answer_chunk(channel_mask, ":OK", 1);
        } else
        {
          send_answer_chunk(channel_mask, ":FAIL:", 1);
          uint8_to_hex(buf, result);
          send_answer_chunk(channel_mask, buf, 0);
        }
        return NULL;
      } else // addressed read!
      {
        send_answer_chunk(channel_mask, ":R:", 0);
        byte result = WIRE.endTransmission(false);     // repeated start
#ifdef ARDUINO_ARCH_RP2040
        if (result == 4) result = 0; // ignore error=4 ('other error', but I can't seem to find anything wrong; only on this MCU platform)
#endif
        if (result != 0)
        {
          send_answer_chunk(channel_mask, ":FAIL:", 0);
          uint8_to_hex(buf, result);
          send_answer_chunk(channel_mask, buf, 1);
          return NULL;
        }

        uint16_t read_n_bytes = atoi(&cmd[8+i*2+1]);
        WIRE.requestFrom((uint8_t)sa, uint8_t(read_n_bytes), uint8_t(true));
        for (uint8_t i=0; i<read_n_bytes; i++)
        {
          uint8_t data = WIRE.read();
          uint8_to_hex(buf, data);
          send_answer_chunk(channel_mask, buf, 0);
        }

        result = WIRE.endTransmission();     // stop transmitting
#ifdef ARDUINO_ARCH_RP2040
        if (result == 4) result = 0; // ignore error=4 ('other error', but I can't seem to find anything wrong; only on this MCU platform)
#endif
        if (result == 0) // SUCCESS
        {
          send_answer_chunk(channel_mask, ":OK", 1);
        } else
        {
          send_answer_chunk(channel_mask, ":FAIL:", 0);
          uint8_to_hex(buf, result);
          send_answer_chunk(channel_mask, buf, 1);
        }
      }
    } else
    {
      send_answer_chunk(channel_mask, "ERROR:No R|W info found in cmd", 1);
      return NULL;
    }

    return NULL;
  }

  this_cmd = "scan"; // SCAN i2c bus command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    WIRE.endTransmission();
    delayMicroseconds(5);

    cmd_tear_down(255); // tear down all current drivers; if any.

    uint8_t count_slaves = 0;

    for (uint8_t sa = 1; sa<128; sa++)
    {
      g_device_list[sa] &= uint8_t(~0x80); // clear the found on i2c bus bit!
      WIRE.beginTransmission(sa);
      byte result = WIRE.endTransmission();     // stop transmitting
#ifdef ARDUINO_ARCH_RP2040
      if (result == 4) result = 0; // ignore error=4 ('other error', but I can't seem to find anything wrong; only on this MCU platform)
#endif

      if (result == 0) // SUCCESS
      {
        count_slaves++;
        g_device_list[sa] |= uint8_t(0x80); // set the found on i2c bus bit!
        char buf[16]; memset(buf, 0, sizeof(buf));
        send_answer_chunk(channel_mask, this_cmd, 0);
        send_answer_chunk(channel_mask, ":", 0);
        uint8_to_hex(buf, sa);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ":", 0);
        uint8_to_hex(buf, g_device_list[sa] & 0x0F);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ",", 0);
        uint8_to_hex(buf, g_device_list[sa] & 0x40 ? 1 : 0);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ",", 0);
        uint8_to_hex(buf, g_device_list[sa] & 0x20 ? 1 : 0);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ",", 0);

        for (byte k = 0; k < 16; k++)
        {
          buf[k] = pgm_read_byte_near(DEVICE_NAMES[g_device_list[sa] & 0x0F]+k);
        }
        send_answer_chunk(channel_mask, buf, 1);
      }
    }

    if (count_slaves == 0)
    {
      send_answer_chunk(channel_mask, this_cmd, 0);
      send_answer_chunk(channel_mask, ":no slaves found", 1);
      g_active_slave = 0;
    } else
    {
      if (g_active_slave == 0)
      {
        send_answer_chunk(channel_mask, "", 1);
        handle_task_next(channel_mask);
      }

      if ((g_device_list[g_active_slave] & 0x0080) == 0) // old active slave is not found on the bus anymore.
      { // thus select next slave on the bus
        send_answer_chunk(channel_mask, "", 1);
        handle_task_next(channel_mask);
      }
    }
    return NULL;
  }

  this_cmd = "ls"; // List Slave command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    uint8_t count_slaves = 0;

    for (uint8_t sa = 1; sa<128; sa++)
    {
      if (g_device_list[sa] & uint8_t(0x80))
      {
        count_slaves++;
        char buf[16]; memset(buf, 0, sizeof(buf));
        send_answer_chunk(channel_mask, this_cmd, 0);
        send_answer_chunk(channel_mask, ":", 0);
        uint8_to_hex(buf, sa);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ":", 0);
        uint8_to_hex(buf, g_device_list[sa] & 0x0F);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ",", 0);
        uint8_to_hex(buf, g_device_list[sa] & 0x40 ? 1 : 0);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ",", 0);
        uint8_to_hex(buf, g_device_list[sa] & 0x20 ? 1 : 0);
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ",", 0);

        for (byte k = 0; k < 16; k++)
        {
          buf[k] = pgm_read_byte_near(DEVICE_NAMES[g_device_list[sa] & 0x0F]+k);
        }
        send_answer_chunk(channel_mask, buf, 1);
      }
    }

    if (count_slaves == 0)
    {
      send_answer_chunk(channel_mask, this_cmd, 0);
      send_answer_chunk(channel_mask, ":no slaves listed", 1);
    }
    return NULL;
  }

  this_cmd = "dis"; // DIsable Slave command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    int16_t i = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
      if (*(cmd+strlen(this_cmd)+3) == ':')
      {
        i = strlen(this_cmd)+4;
      }
    }
    if (sa < 0)
    {
      sa = g_active_slave;
    }
    uint8_t disable = 1;
    if (i > 0)
    {
      disable = atoi(cmd+i);
    }

    char buf[16]; memset(buf, 0, sizeof(buf));
    send_answer_chunk(channel_mask, this_cmd, 0);
    send_answer_chunk(channel_mask, ":", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, ":", 0);

    if (g_device_list[sa] & uint8_t(0x80)) // only allow disable of discovered slaves
    {
      if (disable > 0)
      {
        g_device_list[sa] |= uint8_t(0x20);
        uint8_to_hex(buf, 1);
      } else
      {
        g_device_list[sa] &= uint8_t(~0x20);
        uint8_to_hex(buf, 0);
      }
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, ":", 0);
      send_answer_chunk(channel_mask, "OK [i2c-stick register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, "FAIL: slave not seen by scan; try scan again", 1);
    }
    return NULL;
  }

  this_cmd = "mv"; // Measure (sensor) Value command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
    }
    if (sa < 0)
    {
      sa = g_active_slave;
    }

    handle_cmd_mv(sa, channel_mask);
    return NULL;
  }

  this_cmd = "nd"; // New Data command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
    }
    if (sa < 0)
    {
      sa = g_active_slave;
    }

    handle_cmd_nd(sa, channel_mask);
    return NULL;
  }

  this_cmd = "as"; // Active Slave command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    send_answer_chunk(channel_mask, "as:", 0);
    char buf[16]; memset(buf, 0, sizeof(buf));
    send_answer_chunk(channel_mask, bytetohex(g_active_slave), 0);
    send_answer_chunk(channel_mask, ":", 0);
    send_answer_chunk(channel_mask, bytetostr(g_device_list[g_active_slave] & 0x0F), 0);
    send_answer_chunk(channel_mask, ",", 0);

    for (byte k = 0; k < 16; k++)
    {
      buf[k] = pgm_read_byte_near(DEVICE_NAMES[g_device_list[g_active_slave] & 0x0F]+k);
    }
    send_answer_chunk(channel_mask, buf, 1);
    return NULL;
  }

  this_cmd = "sn"; // Serial Number command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
    }
    if (sa < 0)
    {
      sa = g_active_slave;
    }

    handle_cmd_sn(sa, channel_mask);
    return NULL;
  }

  this_cmd = "cs"; // Config Slave command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
    }
    uint8_t i = 0;
    if (sa < 0)
    {
      sa = g_active_slave;
    }
    for (; i<strlen(cmd); i++)
    {
      if (cmd[i] == ':')
      {
        i++;
        break;
      }
    }

    handle_cmd_cs(sa, channel_mask, cmd+i);
    return NULL;
  }

  this_cmd = "+cs:"; // Config Slave command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = atohex8(cmd+strlen(this_cmd));
    if (sa < 0)
    {
      sa = g_active_slave;
    }
    uint8_t i = strlen(this_cmd);
    for (; i<strlen(cmd); i++)
    {
      if (cmd[i] == ':')
      {
        i++;
        break;
      }
    }

    handle_cmd_cs_write(sa, channel_mask, cmd+i);
    return NULL;
  }

  this_cmd = "ee"; // EEPROM read command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
    }
    uint8_t i = 0;
    if (sa < 0)
    {
      sa = g_active_slave;
    }
    for (; i<strlen(cmd); i++)
    {
      if (cmd[i] == ':')
      {
        i++;
        break;
      }
    }

    handle_cmd_ee(sa, channel_mask);
    return NULL;
  }

  this_cmd = "raw"; // read raw sensor data command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
    }
    uint8_t i = 0;
    if (sa < 0)
    {
      sa = g_active_slave;
    }
    for (; i<strlen(cmd); i++)
    {
      if (cmd[i] == ':')
      {
        i++;
        break;
      }
    }

    handle_cmd_raw(sa, channel_mask);
    return NULL;
  }

  this_cmd = "test"; // Test command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t sa = -1;
    if (cmd[strlen(this_cmd)] == ':')
    {
      sa = atohex8(cmd+strlen(this_cmd)+1);
    }
    uint8_t i = 0;
    if (sa < 0)
    {
      sa = g_active_slave;
    }
    for (; i<strlen(cmd); i++)
    {
      if (cmd[i] == ':')
      {
        i++;
        break;
      }
    }

    handle_cmd_test(sa, channel_mask, cmd+i);
    return NULL;
  }

  this_cmd = "ca"; // Config Application command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t app_id = atohex8(cmd+strlen(this_cmd));
    if (app_id < 0)
    {
      app_id = g_app_id;
    }
    uint8_t i = 0;
    for (; i<strlen(cmd); i++)
    {
      if (cmd[i] == ':')
      {
        i++;
        break;
      }
    }

    cmd_ca(app_id, channel_mask, cmd+i);
    return NULL;
  }

  this_cmd = "+ca:"; // Config Application command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t app_id = atohex8(cmd+strlen(this_cmd));
    uint8_t i = 0;
    if (app_id < 0)
    {
      app_id = g_app_id;
    }
    for (; i<strlen(cmd); i++)
    {
      if (cmd[i] == ':')
      {
        i++;
        break;
      }
    }

    cmd_ca_write(app_id, channel_mask, cmd+i);
    return NULL;
  }

  this_cmd = "app"; // APPlication command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    send_answer_chunk(channel_mask, "app:", 0);
    char buf[8]; memset(buf, 0, sizeof(buf));
    uint8_to_hex(buf, g_app_id);
    send_answer_chunk(channel_mask, buf, 1);
    return NULL;
  }

  this_cmd = "+app:"; // APPlication command
  if (!strncmp(this_cmd, cmd, strlen(this_cmd)))
  {
    int16_t app_id = atohex8(cmd+strlen(this_cmd));

    send_answer_chunk(channel_mask, "+app:", 0);

    if (app_id < 0)
    {
      send_answer_chunk(channel_mask, "FAILED", 1);
    } else
    {
      g_app_id = app_id;
      char buf[8]; memset(buf, 0, sizeof(buf));
      uint8_to_hex(buf, app_id);
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, ":STARTED", 1);
    }
    return NULL;
  }

  return cmd;
}


void
handle_cmd_mv(uint8_t sa, uint8_t channel_mask)
{
  float mv_list[768+1];
  uint16_t mv_count = sizeof(mv_list)/sizeof(mv_list[0]);
  char buf[16];
  const char *error_message = NULL;
  uint32_t time_stamp = millis();

  send_answer_chunk(channel_mask, "mv:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":", 0);

  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    uint32_to_dec(buf, time_stamp, 8);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, ":", 0);
    send_answer_chunk(channel_mask, "FAIL: Slave not found; try scan command!", 1);
    return;
  }

  if (cmd_mv(sa, mv_list, &mv_count, &error_message) == 0)
  {
    uint32_to_dec(buf, time_stamp, 8);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, ":", 0);
    send_answer_chunk(channel_mask, "FAIL: no device driver assigned", 1);
    return;
  }
  time_stamp = millis(); // update timestamp when data is available.
  uint32_to_dec(buf, time_stamp, 8);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":", 0);
  if (error_message != NULL)
  {
    send_answer_chunk(channel_mask, "FAIL: ", 0);
    send_answer_chunk(channel_mask, error_message, 1);
    return;
  }
  if (mv_count == 0)
  {
    send_answer_chunk(channel_mask, "FAIL: local buffer not big enough", 1);
    return;
  }

  if ((g_config_host & 0x000F) == HOST_CFG_FORMAT_DEC)
  {
    for (int16_t i=0; i<mv_count; i++)
    {
      memset(buf, 0, sizeof(buf));
      const char *p = my_dtostrf(mv_list[i], 10, 2, buf);
      while (*p == ' ') p++; // remove leading space
      if ((p - buf) > 10) p = "NaN"; // should never ever happen!

      if (i < (mv_count - 1))
      { // not yet last element...
        send_answer_chunk(channel_mask, p, 0);
        send_answer_chunk(channel_mask, ",", 0);
      } else
      { // last element...
        send_answer_chunk(channel_mask, p, 1);
      }
    }
  }

  if ((g_config_host & 0x000F) == HOST_CFG_FORMAT_HEX)
  {
    char buf[16]; memset(buf, 0, sizeof(buf));

    sprintf(buf, "HEX:%d:", 32);
    send_answer_chunk(channel_mask, buf, 0);

    for (int16_t i=0; i<mv_count; i++)
    {
      memset(buf, 0, sizeof(buf));
      uint16_t int_value = int(mv_list[i] * 32) & 0x0FFFF;
      uint16_to_hex(buf, int_value);
      if (i < (mv_count - 1))
      { // not yet last element...
        send_answer_chunk(channel_mask, buf, 0);
      } else
      { // last element...
        send_answer_chunk(channel_mask, buf, 1);
      }
    }
  }

  if ((g_config_host & 0x000F) == HOST_CFG_FORMAT_BIN)
  {
    // send in text format the length of the byte-stream in binary format.
    char buf[16]; memset(buf, 0, sizeof(buf));
    sprintf(buf, "BIN:%d:%d", 32, (mv_count)*2);
    send_answer_chunk(channel_mask, buf, 1);

    for (int16_t i=0; i<mv_count; i++)
    {
      uint16_t int_value = int(mv_list[i] * 32) & 0x0FFFF;
      Serial.write( (uint8_t *) &int_value, sizeof(int_value));
    }
    Serial.flush();
  }
}


void
handle_cmd_raw(uint8_t sa, uint8_t channel_mask)
{
  uint16_t raw_list[834]; memset(raw_list, 0, sizeof(raw_list));
  uint16_t raw_count = sizeof(raw_list)/sizeof(raw_list[0]);
  char buf[16];
  const char *error_message = NULL;

  send_answer_chunk(channel_mask, "raw:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":", 0);

  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    send_answer_chunk(channel_mask, "FAIL: Slave[", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, "] not found; try scan command!", 1);
    return;
  }

  if (cmd_raw(sa, raw_list, &raw_count, &error_message) == 0)
  {
    send_answer_chunk(channel_mask, "FAIL: no device driver assigned", 1);
    return;
  }

  if (error_message != NULL)
  {
    send_answer_chunk(channel_mask, "FAIL: ", 0);
    send_answer_chunk(channel_mask, error_message, 1);
    return;
  }

  if (raw_count == 0)
  {
    send_answer_chunk(channel_mask, "FAIL: local buffer not big enough", 1);
    return;
  }

  for (int16_t i=0; i<raw_count; i++)
  {
    uint16_to_hex(buf, raw_list[i]);

    if (i < (raw_count - 1))
    { // not yet last element...
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, ",", 0);
    } else
    { // last element...
      send_answer_chunk(channel_mask, buf, 1);
    }
  }
}


void
handle_cmd_test(uint8_t sa, uint8_t channel_mask, const char *input)
{
  char buf[8]; memset(buf, 0, sizeof(buf));
  const char *error_message = NULL;
  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    send_answer_chunk(channel_mask, "test:FAIL: Slave[", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, "] not found; try scan command!", 1);
    return;
  }
  if (cmd_test(sa, channel_mask, input, &error_message) == 0)
  {
    send_answer_chunk(channel_mask, "test:FAIL: no device driver assigned", 1);
  }

  if (error_message != NULL)
  {
    send_answer_chunk(channel_mask, "FAIL: ", 0);
    send_answer_chunk(channel_mask, error_message, 1);
    return;
  }
}


void
handle_cmd_nd(uint8_t sa, uint8_t channel_mask)
{
  uint8_t nd;
  char buf[16]; memset(buf, 0, sizeof(buf));
  const char *error_message = NULL;

  send_answer_chunk(channel_mask, "nd:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":", 0);

  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    send_answer_chunk(channel_mask, "FAIL: Slave[", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, "] not found; try scan command!", 1);
    return;
  }

  if (cmd_nd(sa, &nd, &error_message) == 0)
  {
    send_answer_chunk(channel_mask, "FAIL: no device driver assigned", 1);
    return;
  }

  if (error_message != NULL)
  {
    send_answer_chunk(channel_mask, "FAIL: ", 0);
    send_answer_chunk(channel_mask, error_message, 1);
    return;
  }

  send_answer_chunk(channel_mask, nd ? "1" : "0", 1);
}


void
handle_cmd_sn(uint8_t sa, uint8_t channel_mask)
{
  uint16_t sn_list[4]; memset(sn_list, 0, sizeof(sn_list));
  uint16_t sn_count = sizeof(sn_list)/sizeof(sn_list[0]);
  char buf[16];
  const char *error_message = NULL;

  send_answer_chunk(channel_mask, "sn:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":", 0);

  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    send_answer_chunk(channel_mask, "FAIL: Slave[", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, "] not found; try scan command!", 1);
    return;
  }

  if (cmd_sn(sa, sn_list, &sn_count, &error_message) == 0)
  {
    send_answer_chunk(channel_mask, "FAIL: no device driver assigned", 1);
    return;
  }

  if (error_message != NULL)
  {
    send_answer_chunk(channel_mask, "FAIL: ", 0);
    send_answer_chunk(channel_mask, error_message, 1);
    return;
  }

  if (sn_count == 0)
  {
    send_answer_chunk(channel_mask, "FAIL: local buffer not big enough", 1);
    return;
  }

  for (int16_t i=0; i<sn_count; i++)
  {
    uint16_to_hex(buf, sn_list[i]);

    if (i < (sn_count - 1))
    { // not yet last element...
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, "-", 0);
    } else
    { // last element...
      send_answer_chunk(channel_mask, buf, 1);
    }
  }
}


void
handle_cmd_ch(uint8_t channel_mask, const char *input)
{
  if (cmd_ch(channel_mask, input) == 0)
  {
    send_answer_chunk(channel_mask, "ch:ERROR", 1);
  }
}


void
handle_cmd_ch_write(uint8_t channel_mask, const char *input)
{
  if (cmd_ch_write(channel_mask, input) == 0)
  {
    send_answer_chunk(channel_mask, "+ch:ERROR", 1);
  }
}


void
handle_cmd_cs(uint8_t sa, uint8_t channel_mask, const char *input)
{
  if (cmd_cs(sa, channel_mask, input) == 0)
  {
    send_answer_chunk(channel_mask, "cs:FAIL: no device driver assigned", 1);
  }
}


void
handle_cmd_cs_write(uint8_t sa, uint8_t channel_mask, const char *input)
{
  if (cmd_cs_write(sa, channel_mask, input) == 0)
  {
    send_answer_chunk(channel_mask, "+cs:FAIL: no device driver assigned", 1);
  }
}


void
handle_cmd_ee(uint8_t sa, uint8_t channel_mask)
{
  uint16_t ee_list[832];
  uint16_t ee_count = sizeof(ee_list)/sizeof(ee_list[0]);
  uint16_t ee_start_address = 0;
  char buf[16];
  const char *error_message = NULL;

  send_answer_chunk(channel_mask, "ee:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":", 0);

  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    send_answer_chunk(channel_mask, "FAIL: Slave[", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, "] not found; try scan command!", 1);
    return;
  }

  if (cmd_ee(sa, ee_list, &ee_count, &ee_start_address, &error_message) == 0)
  {
    send_answer_chunk(channel_mask, "FAIL: no device driver assigned", 1);
    return;
  }

  if (error_message != NULL)
  {
    send_answer_chunk(channel_mask, "FAIL: ", 0);
    send_answer_chunk(channel_mask, error_message, 1);
    return;
  }

  uint16_to_hex(buf, ee_start_address);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ",10,01,", 0); // 10h = 16 bit per data element, 01h = address increments by 1 each data element (each 16 bit).
  uint16_to_hex(buf, ee_count);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ",DATA,", 0);

  if (ee_count == 0)
  {
    send_answer_chunk(channel_mask, "no data", 1);
  } else
  {
    for (uint16_t i=0; i<ee_count; i++)
    {
      uint16_to_hex(buf, ee_list[i]);

      if (i < (ee_count - 1))
      { // not yet last element...
        send_answer_chunk(channel_mask, buf, 0);
        send_answer_chunk(channel_mask, ",", 0);
      } else
      { // last element...
        send_answer_chunk(channel_mask, buf, 1);
      }
    }
  }
}


const char *
handle_task(uint8_t channel_mask, char task)
{
  if (task == ';') // go-task
  {
    g_mode = MODE_CONTINUOUS;
    send_answer_chunk(channel_mask, ";:continuous mode", 1);
    return "";
  }
  if (task == '!') // halt-task
  {
    g_mode = MODE_INTERACTIVE;
    send_answer_chunk(channel_mask, "!:interactive mode", 1);
    return "";
  }
  if (task == '>') // next-sensor-task
  {
    handle_task_next(channel_mask);
    return "";
  }
  if (task == '<') // previous-sensor-task
  {
    uint8_t old_active_slave = g_active_slave;
    for (int8_t sa=old_active_slave-1; sa>=0; sa--)
    {
      if ((g_device_list[sa] & 0x80) && ((g_device_list[sa] & 0x0F) > 0))
      {
        g_active_slave = sa;
        break;
      }
    }
    if (g_active_slave == old_active_slave)
    {
      for (uint8_t sa=127; sa>old_active_slave; sa--)
      {
        if ((g_device_list[sa] & 0x80) && ((g_device_list[sa] & 0x0F) > 0))
        {
          g_active_slave = sa;
          break;
        }
      }
    }
    char buf[32]; memset(buf, 0, sizeof(buf));
    strcpy(buf, "<:");
    strcpy(buf+strlen(buf), bytetohex(g_active_slave));
    strcpy(buf+strlen(buf), ":");
    strcpy(buf+strlen(buf), bytetostr(g_device_list[g_active_slave] & 0x0F));
    strcpy(buf+strlen(buf), ",");

    char *dev_name = buf + strlen(buf);
    for (byte k = 0; k < 16; k++)
    {
      dev_name[k] = pgm_read_byte_near(DEVICE_NAMES[g_device_list[g_active_slave] & 0x0F]+k);
    }
    send_answer_chunk(channel_mask, buf, 1);
    return "";
  }
  if ((task == '?') || (task == '1')) // help-task
  {
    send_answer_chunk(channel_mask, "Usage: Melexis I2C STICK", 1);
    send_answer_chunk(channel_mask, "========================", 1);
    handle_task_help(channel_mask);
    g_mode = MODE_INTERACTIVE;
    return "";
  }
  if (task == '5') // refresh task ==> redirect to scan command.
  {
    handle_cmd(channel_mask, "scan");
    return "";
  }
  return NULL; // no task!
}


void
handle_task_next(uint8_t channel_mask)
{
  uint8_t old_active_slave = g_active_slave;
  for (uint8_t sa=old_active_slave+1; sa<128; sa++)
  {
    if ((g_device_list[sa] & 0x80) && ((g_device_list[sa] & 0x0F) > 0))
    {
      g_active_slave = sa;
      break;
    }
  }
  if (g_active_slave == old_active_slave)
  {
    for (uint8_t sa=0; sa<old_active_slave; sa++)
    {
      if ((g_device_list[sa] & 0x80) && ((g_device_list[sa] & 0x0F) > 0))
      {
        g_active_slave = sa;
        break;
      }
    }
  }
  char buf[32]; memset(buf, 0, sizeof(buf));
  strcpy(buf, ">:");
  strcpy(buf+strlen(buf), bytetohex(g_active_slave));
  strcpy(buf+strlen(buf), ":");
  strcpy(buf+strlen(buf), bytetostr(g_device_list[g_active_slave] & 0x0F));
  strcpy(buf+strlen(buf), ",");

  char *dev_name = buf+strlen(buf);
  for (byte k = 0; k < 16; k++)
  {
    dev_name[k] = pgm_read_byte_near(DEVICE_NAMES[g_device_list[g_active_slave] & 0x0F]+k);
  }
  send_answer_chunk(channel_mask, buf, 1);
}


uint16_t
int_uart()
{
  uint8_t channel_mask = 1U<<CHANNEL_UART;
  static char cmd[32];
  static char *p_cmd = cmd;
  if (!Serial.available()) return 1; // no new data => return
  while (Serial.available())
  {
    int8_t ch = Serial.read();

    if (p_cmd == cmd)
    { // first char might be a single char task!
      const char *p_answer = handle_task(channel_mask, ch);
      if (p_answer)
      {
        return 0;
      }
    }

    if ((ch == '\n') || (ch == '\r') || // end of cmd character.
        ((p_cmd - cmd + 1) >= (int)(sizeof(cmd)))) // cmd buffer full! lets try to handle the command; some command will read more bytes later if needed....
    {
      if (p_cmd == cmd) return 0; // empty command; likely only <LF> was sent!
      const char *p_answer = handle_cmd(channel_mask, cmd);
      if (p_answer) Serial.println(p_answer);

      memset(cmd, 0, sizeof(cmd));
      p_cmd = cmd;
      return 0;
    }
    *p_cmd = ch;
    p_cmd++;
  }
  return 0; // 0 => success!
}


void
send_answer_chunk(uint8_t channel_mask, const char *answer, uint8_t terminate)
{ // we currently have only UART/Serial,
  if (channel_mask & (1U<<CHANNEL_UART))
  {
    while (Serial.availableForWrite() < 200)
    {
      // wait!!
    }

    if (terminate)
    {
      Serial.println(answer);
      Serial.flush();
    } else
    {
      Serial.print(answer);
    }
  }
  if (channel_mask & (1U<<CHANNEL_ETHERNET))
  { // todo
  }
  if (channel_mask & (1U<<CHANNEL_WIFI))
  { // todo
  }
  if (channel_mask & (1U<<CHANNEL_BLUETOOTH))
  { // todo
  }
#ifdef BUFFER_COMMAND_ENABLE
  if (channel_mask & (1U<<CHANNEL_BUFFER))
  {
    uint32_t answer_len = strlen(answer);
    if ((g_channel_buffer_pos + answer_len) < (BUFFER_CHANNEL_SIZE - 3))
    {
      strcpy(g_channel_buffer+g_channel_buffer_pos, answer);
      g_channel_buffer_pos += answer_len;
      if (terminate)
      {
        strcpy(g_channel_buffer+g_channel_buffer_pos, "\r\n");
        g_channel_buffer_pos += 2;
      }
    }
  }
#endif // BUFFER_COMMAND_ENABLE
}


void
send_broadcast_message(const char *msg)
{// currently we do only UART
  Serial.println(msg);
}


void
handle_continuous_mode()
{
  uint8_t old_active_slave = g_active_slave;
  const char *error_message = NULL;
  for (uint8_t sa=0; sa<128; sa++)
  {
    if ((g_device_list[sa] & 0x80) && (!(g_device_list[sa] & 0x20)) && ((g_device_list[sa] & 0x0F) > 0))
    {
      g_active_slave = sa;
      uint8_t nd = 0; // new data
      cmd_nd(sa, &nd, &error_message);
      if (nd > 0)
      {
        char buf[32];
        memset(buf, 0, sizeof(buf));
        char *p = buf;
        *p = '@'; p++;
        uint8_to_hex(p, sa); p += 2;
        *p = ':'; p++;
        uint8_to_hex(p, g_device_list[sa] & 0x0F); p += 2;
        *p = ':'; p++;
        // Serial.println(Serial.availableForWrite());
        send_answer_chunk(g_channel_mask, buf, 0);
        handle_cmd_mv(sa, g_channel_mask);
        if (g_device_list[sa] & 0x40)
        {
          send_answer_chunk(g_channel_mask, buf, 0);
          handle_cmd_raw(sa, g_channel_mask);
        }
      }
    }
  }

  g_active_slave = old_active_slave;
}


void
handle_applications()
{
  uint8_t channel_mask = 0xFF;
}


void
handle_task_help(uint8_t channel_mask)
{
  send_answer_chunk(channel_mask, "Tasks: single character tasks; no new line required", 1);
  send_answer_chunk(channel_mask, "- ;  ==>  enter continuous mode", 1);
  send_answer_chunk(channel_mask, "- !  ==>  quit continuous mode", 1);
  send_answer_chunk(channel_mask, "- >  ==>  next active slave", 1);
  send_answer_chunk(channel_mask, "- <  ==>  previous active slave", 1);
  send_answer_chunk(channel_mask, "- ?  ==>  this help", 1);
  send_answer_chunk(channel_mask, "- 1  ==>  this help", 1);
  send_answer_chunk(channel_mask, "- 5  ==>  scan command alias", 1);
  send_answer_chunk(channel_mask, "", 1);
  send_answer_chunk(channel_mask, "Commands: 2+ character commands with new line required", 1);
  send_answer_chunk(channel_mask, "- mlx  ==>  test uplink communication", 1);
  send_answer_chunk(channel_mask, "- help ==>  this help!", 1);
  send_answer_chunk(channel_mask, "- sos  ==>  more detailed help!", 1);
  send_answer_chunk(channel_mask, "- fv   ==>  Firmware Version", 1);
  send_answer_chunk(channel_mask, "- bi   ==>  Board Info", 1);
  send_answer_chunk(channel_mask, "- scan ==>  SCAN I2C bus for slaves", 1);
  send_answer_chunk(channel_mask, "- ls   ==>  List Slaves (already discovered with 'scan')", 1);
  send_answer_chunk(channel_mask, "- dis  ==>  DIsable Slave (for continuous dump mode)", 1);
  send_answer_chunk(channel_mask, "- i2c  ==>  low level I2C", 1);
  send_answer_chunk(channel_mask, "- ch   ==>  Configuration of Host (I2C freq, output format)", 1);  
#ifdef BUFFER_COMMAND_ENABLE
  send_answer_chunk(channel_mask, "- buf  ==>  buffer command", 1);
#endif // BUFFER_COMMAND_ENABLE
  send_answer_chunk(channel_mask, "", 1);
  send_answer_chunk(channel_mask, "- as   ==>  Active Slave", 1);
  send_answer_chunk(channel_mask, "- mv   ==>  Measure Value of the sensor", 1);
  send_answer_chunk(channel_mask, "- sn   ==>  Serial Number of slave", 1);
  send_answer_chunk(channel_mask, "- cs   ==>  Configuration of Slave", 1);
  send_answer_chunk(channel_mask, "- nd   ==>  New Data available", 1);
  send_answer_chunk(channel_mask, "- ee   ==>  EEprom dump", 1);
  send_answer_chunk(channel_mask, "- raw  ==>  RAW sensor data dump", 1);
  send_answer_chunk(channel_mask, "", 1);
  send_answer_chunk(channel_mask, "- app  ==>  APPlication id", 1);
  send_answer_chunk(channel_mask, "- ca   ==>  Configuration of Application", 1);
  send_answer_chunk(channel_mask, "", 1);
  send_answer_chunk(channel_mask, "more at https://github.com/melexis/mlx-i2c-stick", 1);
}


void
handle_command_sos(uint8_t channel_mask, const char *input)
{
  if (*input == ':')
  {
    const char *this_cmd = ":i2c";
    if (!strncmp(this_cmd, input, strlen(this_cmd)))
    {
      send_answer_chunk(channel_mask, "I2C command format:", 1);
      send_answer_chunk(channel_mask, "1] WRITE         : 'i2c:<sa>:W<byte#0><byte#1>...' bytes in hex format", 1);
      send_answer_chunk(channel_mask, "2] READ          : 'i2c:<sa>:R<amount of bytes to read>' amount in decimal format", 1);
      send_answer_chunk(channel_mask, "3] ADDRESSED READ: 'i2c:<sa>:W<byte#0><byte#1>R<amount of bytes to read>...' bytes in hex format, amount in decimal", 1);
      send_answer_chunk(channel_mask, "<sa> Slave Address in hex format (7-bit only)", 1);
      return;
    }
    this_cmd = ":ch";
    if (!strncmp(this_cmd, input, strlen(this_cmd)))
    {
      send_answer_chunk(channel_mask, "Configure Host command format:", 1);
      send_answer_chunk(channel_mask, "1] set the output format:", 1);
      send_answer_chunk(channel_mask, "    - +ch:FORMAT=DEC", 1);
      send_answer_chunk(channel_mask, "    - +ch:FORMAT=HEX", 1);
      send_answer_chunk(channel_mask, "    - +ch:FORMAT=BIN", 1);
      send_answer_chunk(channel_mask, "2] set the I2C frequency:", 1);
      send_answer_chunk(channel_mask, "    - +ch:I2C_FREQ=F100k", 1);
      send_answer_chunk(channel_mask, "    - +ch:I2C_FREQ=F400k", 1);
      send_answer_chunk(channel_mask, "    - +ch:I2C_FREQ=F1M", 1);
      send_answer_chunk(channel_mask, "    - +ch:I2C_FREQ=F50k", 1);
      send_answer_chunk(channel_mask, "    - +ch:I2C_FREQ=F20k", 1);
      send_answer_chunk(channel_mask, "    - +ch:I2C_FREQ=F10k", 1);
      return;
    }
    this_cmd = ":dis";
    if (!strncmp(this_cmd, input, strlen(this_cmd)))
    {
      send_answer_chunk(channel_mask, "disable a slave:", 1);
      send_answer_chunk(channel_mask, "  - dis   (disable active slave)", 1);
      send_answer_chunk(channel_mask, "  - dis:33", 1);
      send_answer_chunk(channel_mask, "  - dis:33:1", 1);
      send_answer_chunk(channel_mask, "enable a slave:", 1);
      send_answer_chunk(channel_mask, "  - dis:33:0", 1);
      return;
    }
    send_answer_chunk(channel_mask, "no SOS available for command '", 0);
    send_answer_chunk(channel_mask, input, 0);
    send_answer_chunk(channel_mask, "'", 1);
    return;
  }


  send_answer_chunk(channel_mask, "sos:<cmd>", 1);
  send_answer_chunk(channel_mask, "", 1);
  send_answer_chunk(channel_mask, "example: 'sos:i2c'", 1);
}
