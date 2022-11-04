#include <Arduino.h>
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"


#include "mlx90614_cmd.h"
#include "mlx90632_cmd.h"
#include "mlx90640_cmd.h"
#include "mlx90641_cmd.h"
#include "mlx9064x_cmd.h"
#include "mlx90393_cmd.h"

#ifdef DEVICE_PT100_ADS122C_ENABLE
#include "pt100_ads122c_cmd.h"
#endif //DEVICE_PT100_ADS122C_ENABLE

#ifdef DEVICE_MLX90640_TEST_ENABLE
#include "mlx90640_test.h"
#endif // DEVICE_MLX90640_TEST_ENABLE

#ifdef __cplusplus
extern "C" {
#endif


// supporting functions

char nibble_to_hex(uint8_t nibble) {  // convert a 4-bit nibble to a hexadecimal character
  nibble &= 0xF;
  return nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
}

void
uint8_to_hex(char *hex, uint8_t dec)
{
  hex[2] = '\0';
  hex[1] = nibble_to_hex(dec); dec >>= 4;
  hex[0] = nibble_to_hex(dec);
}


void
uint16_to_hex(char *hex, uint16_t dec)
{
  hex[4] = '\0';
  hex[3] = nibble_to_hex(dec); dec >>= 4;
  hex[2] = nibble_to_hex(dec); dec >>= 4;
  hex[1] = nibble_to_hex(dec); dec >>= 4;
  hex[0] = nibble_to_hex(dec);
}


void
uint32_to_dec(char *str, uint32_t dec, int8_t digits)
{
  str[digits] = '\0';
  if (digits <= 0) return;
  for (int8_t d=digits-1; d>=0; d--)
  {
    str[d] = '0' + (dec % 10);
    dec /= 10;
  }
}


int16_t atohex8(const char *in)
{
   uint8_t c, h;

   c = in[0];

   if (c <= '9' && c >= '0') {  c -= '0'; }
   else if (c <= 'f' && c >= 'a') { c -= ('a' - 0x0a); }
   else if (c <= 'F' && c >= 'A') { c -= ('A' - 0x0a); }
   else return(-1);

   h = c;

   c = in[1];

   if (c <= '9' && c >= '0') {  c -= '0'; }
   else if (c <= 'f' && c >= 'a') { c -= ('a' - 0x0a); }
   else if (c <= 'F' && c >= 'A') { c -= ('A' - 0x0a); }
   else return(-1);

   return ( h<<4 | c);
}


int32_t atohex16(const char *in)
{
  uint8_t c;
  uint16_t h = 0;

  for (uint8_t i=0; ; i++)
  {
    c = in[i];
    if (c <= '9' && c >= '0') {  c -= '0'; }
    else if (c <= 'f' && c >= 'a') { c -= ('a' - 0x0a); }
    else if (c <= 'F' && c >= 'A') { c -= ('A' - 0x0a); }
    else return(-1);
    h |= c;
    if (i >= 3) break;
    h <<= 4;
  }

  return int32_t(h);
}


const char *
bytetohex(uint8_t dec)
{
  static char hex[3];
  uint8_t c = dec / 16;
  if (c < 10)
  {
    hex[0] = '0' + c;
  } else
  {
    hex[0] = 'A' + c - 10;
  }
  c = dec % 16;
  if (c < 10)
  {
    hex[1] = '0' + c;
  } else
  {
    hex[1] = 'A' + c - 10;
  }
  return hex;
}


const char *
bytetostr(uint8_t dec)
{
  static char str[4];
  memset(str, 0, sizeof(str));
  uint8_t c = dec / 100;
  uint8_t i = 0;

  str[i] = '0' + c;
  if (c > 0) i++;

  c = (dec / 10) % 10;
  str[i] = '0' + c;
  if (c > 0) i++;

  c = dec % 10;
  str[i] = '0' + c;
  return str;
}

// https://github.com/pmalasp/dtostrf/blob/master/dtostrf.c
char *my_dtostrf( float val,  int8_t char_num, uint8_t precision, char *chr_buffer)
{
  int       right_j;
  int       i, j ;
  float     r_val;
  long      i_val;
  char      c, c_sign;


  // check the sign
  if (val < 0.0) {
    // print the - sign
    c_sign = '-';

    // process the absolute value
    val = - val;
  } else {
    // put a space for positive numbers
    c_sign = ' ';

  }

  // check the left-right justification
  if (char_num < 0)
  {
    // set the flag
    right_j = 1;

    // make the number positive
    char_num = -char_num;

  } else {
    right_j = 0;
  }


  // no native exponential function for int
  j=1;
  for(i=0; i < (char_num - precision - 3 );i++) j *= 10;

  // Hackish fail-fast behavior for larger-than-what-can-be-printed values, countig the precision + sign ('-') +'.' + '\0'
  if (val >= (float)(j))
  {
    // not enough space
    // strcpy(chr_buffer, "ovf"); - this is very byte consuming (388 bytes) , so we go for the cheap array
    chr_buffer[0] = 'o';
    chr_buffer[1] = 'v';
    chr_buffer[2] = 'f';
    chr_buffer[3] = '\0';

    // finish here
    return chr_buffer;
  }



  // Simplistic rounding strategy so that e.g. print(1.999, 2)
  // prints as "2.00"
  r_val = 0.5;
  for ( i = 0; i < precision; i++) {
      // debug
      // Serial.println(r_val, 6);
      r_val /= 10.0;
  }
  val += r_val;

  // Extract the integer and decimal part of the number and print it
  i_val = (long) val;
  r_val = val - (float) i_val;


  // print the integral part ... but it is in reverse order ... so leaves the space for  '.' and the decimal part (and remember that array indexes start from 0
  i = char_num - precision - 2;
  do
  {
      // debug
      // Serial.println(i_val);

      chr_buffer[i] = '0' + ( i_val % 10);
      i_val /= 10;
      i--;

  }   while ( i_val > 0) ;

  // add the sign char
  chr_buffer[i] = c_sign;

  // prepare for the decimal part
  j = char_num - precision - 1;

  // Print the decimal point, but only if there are digits beyond
  if (precision > 0) {
    chr_buffer[j] = '.';
    j++;

    // Extract digits from the remainder one at a time
    while (precision > 0) {
      // prepare the data
      r_val *= 10.0;
      i_val  = (int)    r_val;
      r_val -= (float)  i_val;

      // update the string
      chr_buffer[j] = '0' + ( i_val );
      j++;

      // use precision as the counter
      precision--;
    }
  }

  // terminate the string
  chr_buffer[j] = '\0';

  // check the justification direction
  if (right_j)
  {
    // pad the string with leading ' '
    while (i > 0)
    {
      i--;

      chr_buffer[i] = ' ';
    }

  }

 // return the pointer to the first char of the prepared string
 return ( &chr_buffer[i]);
}


// end supporting functions


// host only commands

uint8_t
cmd_ch(uint8_t channel_mask, const char *input)
{
  char buf[16]; memset(buf, 0, sizeof(buf));
  const char *value = NULL;

  send_answer_chunk(channel_mask, "ch:FORMAT=", 0);
  uint8_t fmt = g_config_host & 0x000F;
  switch(fmt)
  {
    case HOST_CFG_FORMAT_DEC:
      value = "DEC";
      break;
    case HOST_CFG_FORMAT_HEX:
      value = "HEX";
      break;
    case HOST_CFG_FORMAT_BIN:
      value = "BIN";
      break;
    default:
      value = "Unknown";
  }
  sprintf(buf, "%s(%d)", value, fmt);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "ch:I2C_FREQ=", 0);
  uint8_t freq = (g_config_host & 0x00F0) >> 4;
  switch(freq)
  {
    case HOST_CFG_I2C_F100k:
      value = "100kHz";
      break;
    case HOST_CFG_I2C_F400k:
      value = "400kHz";
      break;
    case HOST_CFG_I2C_F1M:
      value = "1MHz";
      break;
    case HOST_CFG_I2C_F50k:
      value = "50kHz";
      break;
    case HOST_CFG_I2C_F20k:
      value = "20kHz";
      break;
    case HOST_CFG_I2C_F10k:
      value = "10kHz";
      break;
    default:
      value = "Unknown";
  }
  sprintf(buf, "%s(%d)", value, freq);
  send_answer_chunk(channel_mask, buf, 1);


  return 1;
}


uint8_t
cmd_ch_write(uint8_t channel_mask, const char *input)
{
  char buf[16]; memset(buf, 0, sizeof(buf));

  const char *var_name = "FORMAT=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    const char *p = input+strlen(var_name);
    uint16_t value = 0;
    uint8_t valid = false;

    if (('0' <= p[0]) && (p[0] <= '9'))
    {
      value = atoi(p);
      if (value < 16)
      {
        valid = true;
      }
    }
    if (!valid)
    {
      if (!strcmp(p, "DEC"))
      {
        value = HOST_CFG_FORMAT_DEC;
        valid = true;
      }
      else if (!strcmp(p, "HEX"))
      {
        value = HOST_CFG_FORMAT_HEX;
        valid = true;
      }
      else if (!strcmp(p, "BIN"))
      {
        value = HOST_CFG_FORMAT_BIN;
        valid = true;
      }
    }
    if (!valid)
    {
      send_answer_chunk(channel_mask, "+ch:", 0);
      send_answer_chunk(channel_mask, input, 0);
      send_answer_chunk(channel_mask, ":ERROR: Invalid value", 1);
      return 0;
    }
    g_config_host &= ~0x000F;
    g_config_host |= (value & 0x000F);
    send_answer_chunk(channel_mask, "+ch:OK [host-register]", 1);
    return 1;
  }

  var_name = "I2C_FREQ=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    const char *p = input+strlen(var_name);
    uint16_t value = 0;
    uint8_t valid = false;

    if (('0' <= p[0]) && (p[0] <= '9'))
    {
      value = atoi(p);
      if (value < 16)
      {
        valid = true;
      }
    }
    if (!valid)
    {
      if (!strcmp(p, "F100k"))
      {
        value = HOST_CFG_I2C_F100k;
        valid = true;
      }
      else if (!strcmp(p, "F400k"))
      {
        value = HOST_CFG_I2C_F400k;
        valid = true;
      }
      else if (!strcmp(p, "F1M"))
      {
        value = HOST_CFG_I2C_F1M;
        valid = true;
      }
      else if (!strcmp(p, "F50k"))
      {
        value = HOST_CFG_I2C_F50k;
        valid = true;
      }
      else if (!strcmp(p, "F20k"))
      {
        value = HOST_CFG_I2C_F20k;
        valid = true;
      }
      else if (!strcmp(p, "F10k"))
      {
        value = HOST_CFG_I2C_F10k;
        valid = true;
      }
    }
    if (!valid)
    {
      send_answer_chunk(channel_mask, "+ch:", 0);
      send_answer_chunk(channel_mask, input, 0);
      send_answer_chunk(channel_mask, ":ERROR: Invalid value", 1);
      return 0;
    }
    g_config_host &= ~0x00F0;
    g_config_host |= ((value << 4) & 0x00F0);

    // apply the configuration
    uint32_t f = 100000;
    switch (value)
    {
      case HOST_CFG_I2C_F100k:
        f = 100000;
        break;
      case HOST_CFG_I2C_F400k:
        f = 400000;
        break;
      case HOST_CFG_I2C_F1M:
        f = 1000000;
        break;
      case HOST_CFG_I2C_F50k:
        f = 50000;
        break;
      case HOST_CFG_I2C_F20k:
        f = 20000;
        break;
      case HOST_CFG_I2C_F10k:
        f = 10000;
        break;
      default:
        f = 100000;
    }
    WIRE.end();
    WIRE.setClock(f); // RP2040 requires first to set the clock
    WIRE.begin();
    WIRE.setClock(f); // NRF52840 requires first begin, then set clock.

    send_answer_chunk(channel_mask, "+ch:OK [host-register]", 1);
    return 1;
  }

  return 0;
}

// end of host only commands

// command functions.

uint8_t
cmd_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message)
{
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;
  float temp;
  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_mv(sa, mv_list, mv_count, error_message);
      break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_mv(sa, mv_list, mv_count, error_message);
      }
      break;
    case DEVICE_90640:
      cmd_90640_mv(sa, mv_list, mv_count, error_message);
      break;
    case DEVICE_90641:
      cmd_90641_mv(sa, mv_list, mv_count, error_message);
      break;
    case DEVICE_90632:
      cmd_90632_mv(sa, mv_list, mv_count, error_message);
      break;
    case DEVICE_90393:
      cmd_90393_mv(sa, mv_list, mv_count, error_message);
      break;
#ifdef DEVICE_PT100_ADS122C_ENABLE
    case DEVICE_PT100_ADS122C:
      cmd_pt100_ads122c_mv(sa, mv_list, mv_count, error_message);
      break;
#endif //DEVICE_PT100_ADS122C_ENABLE
    default:
      return 0;
  }
  return dev;
}


uint8_t
cmd_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message)
{
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;
  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_raw(sa, raw_list, raw_count, error_message);
      break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_raw(sa, raw_list, raw_count, error_message);
      }
      break;
    case DEVICE_90640:
      cmd_90640_raw(sa, raw_list, raw_count, error_message);
      break;
    case DEVICE_90641:
      cmd_90641_raw(sa, raw_list, raw_count, error_message);
      break;
    case DEVICE_90632:
      cmd_90632_raw(sa, raw_list, raw_count, error_message);
      break;
    case DEVICE_90393:
      cmd_90393_raw(sa, raw_list, raw_count, error_message);
      break;
    default:
      return 0;
  }
  return dev;
}


uint8_t
cmd_nd(uint8_t sa, uint8_t *nd, char const **error_message)
{
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;
  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    *nd = 0;
    return dev;
  }
  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_nd(sa, nd, error_message);
      break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_nd(sa, nd, error_message);
      }
      break;
    case DEVICE_90640:
      cmd_90640_nd(sa, nd, error_message);
      break;
    case DEVICE_90641:
      cmd_90641_nd(sa, nd, error_message);
      break;
    case DEVICE_90632:
      cmd_90632_nd(sa, nd, error_message);
      break;
    case DEVICE_90393:
      cmd_90393_nd(sa, nd, error_message);
      break;
#ifdef DEVICE_PT100_ADS122C_ENABLE
    case DEVICE_PT100_ADS122C:
      cmd_pt100_ads122c_nd(sa, nd, error_message);
      break;
#endif //DEVICE_PT100_ADS122C_ENABLE
    default:
      return 0;
  }
  return dev;
}


uint8_t
cmd_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message)
{
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;
  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_sn(sa, sn_list, sn_count, error_message);
      break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_sn(sa, sn_list, sn_count, error_message);
      }
      break;
    case DEVICE_90640:
      cmd_90640_sn(sa, sn_list, sn_count, error_message);
      break;
    case DEVICE_90641:
      cmd_90641_sn(sa, sn_list, sn_count, error_message);
      break;
    case DEVICE_90632:
      cmd_90632_sn(sa, sn_list, sn_count, error_message);
      break;
    case DEVICE_90393:
      cmd_90393_sn(sa, sn_list, sn_count, error_message);
      break;
    default:
      *sn_count = 0;
      return 0;
  }
  return dev;
}


uint8_t
cmd_cs(uint8_t sa, uint8_t channel_mask, const char *input)
{
  char buf[16]; memset(buf, 0, sizeof(buf));
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;
  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    send_answer_chunk(channel_mask, "cs:FAIL: Slave[", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, "] not found; try scan command!", 1);
    return dev;
  }
  if (dev == DEVICE_9064x)
  {
    if (mlx9064x_cmd_auto_detect(sa))
    {
      return cmd_cs(sa, channel_mask, input);
    }
    return 0;
  }

  // handle here: driver link and raw data flag
  uint8_t raw = 0;
  if (g_device_list[sa] & 0x40) raw = 1;

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":DRV=", 0);
  uint8_to_hex(buf, dev);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RAW=", 0);
  uint8_to_hex(buf, raw);
  send_answer_chunk(channel_mask, buf, 1);

  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_cs(sa, channel_mask, input);
      break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_cs(sa, channel_mask, input);
      }
      break;
    case DEVICE_90640:
      cmd_90640_cs(sa, channel_mask, input);
      break;
    case DEVICE_90641:
      cmd_90641_cs(sa, channel_mask, input);
      break;
    case DEVICE_90632:
      cmd_90632_cs(sa, channel_mask, input);
      break;
    case DEVICE_90393:
      cmd_90393_cs(sa, channel_mask, input);
      break;
#ifdef DEVICE_PT100_ADS122C_ENABLE
    case DEVICE_PT100_ADS122C:
      cmd_pt100_ads122c_cs(sa, channel_mask, input);
      break;
#endif //DEVICE_PT100_ADS122C_ENABLE
    default:
      return 0;
  }
  return dev;
}


uint8_t
cmd_cs_write(uint8_t sa, uint8_t channel_mask, const char *input)
{
  // 1. handle here: assign driver
  // 2. the rest is specific for each driver

  char buf[16]; memset(buf, 0, sizeof(buf));
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;

  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    send_answer_chunk(channel_mask, "cs:FAIL: Slave[", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, "] not found; try scan command!", 1);
    return dev;
  }

  // 1. handle here: assign driver
  const char *var_name = "DRV=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    // 1.1. try to find if the desired value is a 'driver-name'
    uint8_t dev = 0;
    for (uint8_t dev_i = 0; dev_i < 16; dev_i++)
    {
      char dev_name[16];
      for (byte k = 0; k < 16; k++)
      {
        dev_name[k] = pgm_read_byte_near(DEVICE_NAMES[dev_i]+k);
      }
      if (!strcasecmp(dev_name, input+strlen(var_name)))
      {
        dev = dev_i;
      }
    }
    // 1.2. if not => try to convert to a device-driver-code.
    if (dev == 0)
    {
      int16_t value = atoi(input+strlen(var_name));
      if ((value > 0) && (value < 16))
      {
        dev = value;
      }
    }
    // 1.3. store the device-driver-code.
    g_device_list[sa & 0x7F] &= 0xF0; // clear the dev-bits
    g_device_list[sa & 0x7F] |= dev;  // set the dev-bits
    if (g_active_slave == 0)
    {// when previous active slave was not set, make the new sa active!
      g_active_slave = sa & 0x7F;
    }
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, ":DRV=", 0);
    uint8_to_hex(buf, dev);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, ":OK [host-register]", 1);
    return dev;
  }

  var_name = "RAW=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    // 1.1. try to find if the desired value is a 'driver-name'
    int16_t value = atoi(input+strlen(var_name));
    if ((value >= 0) && (value <= 1))
    {
      if (value == 1)
      {
        g_device_list[sa] |= 0x40;
      } else
      {
        g_device_list[sa] &= ~0x40;
      }
      send_answer_chunk(channel_mask, "+cs:", 0);
      uint8_to_hex(buf, sa);
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, ":RAW=", 0);
      uint8_to_hex(buf, value);
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, ":OK [host-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, "+cs:", 0);
      uint8_to_hex(buf, sa);
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, ":RAW= FAIL; value not valid", 1);
    }
    return dev;
  }

  if (dev == DEVICE_9064x)
  {
    if (mlx9064x_cmd_auto_detect(sa))
    {
      return cmd_cs_write(sa, channel_mask, input);
    }
    return 0;
  }


  // 2. the rest is specific for each driver
  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_cs_write(sa, channel_mask, input);
      break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_cs_write(sa, channel_mask, input);
      }
      break;
    case DEVICE_90640:
      cmd_90640_cs_write(sa, channel_mask, input);
      break;
    case DEVICE_90641:
      cmd_90641_cs_write(sa, channel_mask, input);
      break;
    case DEVICE_90632:
      cmd_90632_cs_write(sa, channel_mask, input);
      break;
    case DEVICE_90393:
      cmd_90393_cs_write(sa, channel_mask, input);
      break;
#ifdef DEVICE_PT100_ADS122C_ENABLE
    case DEVICE_PT100_ADS122C:
      cmd_pt100_ads122c_cs_write(sa, channel_mask, input);
      break;
#endif //DEVICE_PT100_ADS122C_ENABLE
    default:
      return 0;
  }
  return dev;
}


uint8_t
cmd_ee(uint8_t sa, uint16_t *ee_list, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message)
{
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;

  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_ee(sa, ee_list, ee_count, ee_start_address, error_message);
      break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_ee(sa, ee_list, ee_count, ee_start_address, error_message);
      }
      break;
    case DEVICE_90640:
      cmd_90640_ee(sa, ee_list, ee_count, ee_start_address, error_message);
      break;
    case DEVICE_90641:
      cmd_90641_ee(sa, ee_list, ee_count, ee_start_address, error_message);
      break;
    case DEVICE_90632:
      cmd_90632_ee(sa, ee_list, ee_count, ee_start_address, error_message);
      break;
    case DEVICE_90393:
      cmd_90393_ee(sa, ee_list, ee_count, ee_start_address, error_message);
      break;
    default:
      return 0;
  }

  return dev;
}


uint8_t
cmd_test(uint8_t sa, uint8_t channel_mask, const char *input, char const **error_message)
{
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;
  // 2. the rest is specific for each driver
  if (!(g_device_list[sa & 0x7F] & 0x80))
  { // not found!
    return dev;
  }

  switch(dev)
  {
    // case DEVICE_90614:
    //   cmd_90614_test(sa, channel_mask, input);
    //   break;
    case DEVICE_9064x:
      dev = 0;
      if (mlx9064x_cmd_auto_detect(sa))
      {
        dev = cmd_test(sa, channel_mask, input, error_message);
      }
      break;
#ifdef DEVICE_MLX90640_TEST_ENABLE
    case DEVICE_90640:
      cmd_90640_test(sa, channel_mask, input);
      break;
#endif // DEVICE_MLX90640_TEST_ENABLE
    // case DEVICE_90641:
    //   cmd_90641_test(sa, channel_mask, input);
    //   break;
    // case DEVICE_90632:
    //   cmd_90632_test(sa, channel_mask, input);
    //   break;
    default:
      return 0;
  }
  return dev;
}


uint8_t
cmd_ca(uint8_t app_id, uint8_t channel_mask, const char *input)
{
  switch(app_id)
  {
    case APP_NONE:
      break;
    default:
      return 0;
  }
  return app_id;
}


uint8_t
cmd_ca_write(uint8_t app_id, uint8_t channel_mask, const char *input)
{
  // 1. all is specific for each application
  switch(app_id)
  {
    case APP_NONE:
      break;
    default:
      return 0;
  }
  return app_id;
}


uint8_t
cmd_tear_down(uint8_t sa)
{
  if (sa == 255)
  {
    for (sa = 0; sa <= 127; sa++)
    {
      cmd_tear_down(sa);
    }
    return sa;
  }
  uint8_t dev = g_device_list[sa & 0x7F] & 0x0F;
  switch(dev)
  {
    case DEVICE_90614:
      cmd_90614_tear_down(sa);
      break;
    case DEVICE_90640:
      cmd_90640_tear_down(sa);
      break;
    case DEVICE_90641:
      cmd_90641_tear_down(sa);
      break;
    case DEVICE_90632:
      cmd_90632_tear_down(sa);
      break;
    case DEVICE_90393:
      cmd_90393_tear_down(sa);
      break;
#ifdef DEVICE_PT100_ADS122C_ENABLE
    case DEVICE_PT100_ADS122C:
      cmd_pt100_ads122c_tear_down(sa);
      break;
#endif //DEVICE_PT100_ADS122C_ENABLE
    default:
      return 0;
  }
  return dev;
}

#ifdef __cplusplus
}
#endif
