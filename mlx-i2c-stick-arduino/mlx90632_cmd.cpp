#include "mlx90632_api.h"
#include "mlx90632_advanced.h"
#include "mlx90632_hal.h"
#include "mlx90632_cmd.h"
#include <string.h>
#include <Arduino.h>
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_MLX90632_SLAVES
#define MAX_MLX90632_SLAVES 8
#endif // MAX_MLX90632_SLAVES

#define MLX90632_ERROR_NO_FREE_HANDLE "No free handle; pls recompile firmware with higher 'MAX_MLX90632_SLAVES'"
#define MLX90632_ERROR_BUFFER_TOO_SMALL "Buffer too small"
#define MLX90632_ERROR_COMMUNICATION "Communication error"

static Mlx90632Device *g_mlx90632_device_list[MAX_MLX90632_SLAVES];


Mlx90632Device *
cmd_90632_get_handle(uint8_t sa)
{
  if (sa >= 128)
  {
    return NULL;
  }

  for (uint8_t i=0; i<MAX_MLX90632_SLAVES; i++)
  {
    if (!g_mlx90632_device_list[i])    
    {
      continue; // allow empty spots!
    }
    if (g_mlx90632_device_list[i]->slave_address_ == sa)
    { // found!
      return g_mlx90632_device_list[i];
    }
  }

  // not found => try to find a handle with slave address zero (not yet initialized)!
  for (uint8_t i=0; i<MAX_MLX90632_SLAVES; i++)
  {
    if (!g_mlx90632_device_list[i])
    {
      continue; // allow empty spots!
    }
    if (g_mlx90632_device_list[i]->slave_address_ == 0)
    { // found!
      return g_mlx90632_device_list[i];
    }
  }
  // not found => use first free spot!
  uint8_t i=0;
  for (; i<MAX_MLX90632_SLAVES; i++)
  {
    if (g_mlx90632_device_list[i] == NULL)    
    {
      g_mlx90632_device_list[i] = new Mlx90632Device;
      memset(g_mlx90632_device_list[i], 0, sizeof(Mlx90632Device));
      return g_mlx90632_device_list[i];
    }
  }

  return NULL; // no free spot available
}


static void
delete_handle(uint8_t sa)
{
  for (uint8_t i=0; i<MAX_MLX90632_SLAVES; i++)
  {
    if (!g_mlx90632_device_list[i])
    {
      continue; // allow empty spots!
    }
    if (g_mlx90632_device_list[i]->slave_address_ == sa)
    { // found!
      delete g_mlx90632_device_list[i];
      g_mlx90632_device_list[i] = NULL;
    }
  }
}


void
cmd_90632_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message)
{
  Mlx90632Device *mlx = cmd_90632_get_handle(sa);
  if (mlx == NULL)
  { // failed to get handle!
    *mv_count = 0;
    *error_message = MLX90632_ERROR_NO_FREE_HANDLE;
    return;
  }
  
  if (*mv_count < 2)
  {
    *mv_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90632_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *mv_count = 2;

  if (mlx->slave_address_ == 0) // only the case of a new handle!
  {
    _mlx90632_initialize(mlx, sa);
  }

  if (_mlx90632_measure_degc(mlx, &mv_list[0], &mv_list[1]) != 0)
  {
    *mv_count = 0;
    *error_message = MLX90632_ERROR_COMMUNICATION;
    return;
  }
}


void 
cmd_90632_nd(uint8_t sa, uint8_t *nd, char const **error_message)
{
  Mlx90632Device *mlx = cmd_90632_get_handle(sa);
  *nd = 0;
  if (mlx == NULL)
  { // failed to get handle!
    *error_message = MLX90632_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ == 0) // only the case of a new handle!
  {
    _mlx90632_initialize(mlx, sa);
  }

  uint16_t reg_status = 0, cycle_pos = 0;
  if (_mlx90632_i2c_read(mlx->slave_address_, MLX90632_REG_STATUS, &reg_status) != 0)
  {
    *error_message = MLX90632_ERROR_COMMUNICATION;
    return;
  }
  cycle_pos = (reg_status & MLX90632_STATUS_CYCLE_POSITION) >> 2;

  if (reg_status & MLX90632_STATUS_NEW_DATA) // new data bit is set!
  {
    if (mlx->adc_data_.RAM_6_ == 0) /* first time */
    {
      if (cycle_pos == 2) // check we have a full dataset!
      {
        *nd = 1;
      }
    } else
    {
      *nd = 1;
    }
  }
}


void
cmd_90632_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message)
{
  Mlx90632Device *mlx = cmd_90632_get_handle(sa);
  if (mlx == NULL)
  { // failed to get handle!
    *error_message = MLX90632_ERROR_NO_FREE_HANDLE;
    *sn_count = 0;
    return;
  }

  if (*sn_count < 4)
  {
    *sn_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90632_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *sn_count = 4;

  if (_mlx90632_i2c_read_block(sa, 0x2405, sn_list, 4) != 0)
  {
    *sn_count = 0;
    *error_message = MLX90632_ERROR_COMMUNICATION;
    return;
  }
}


void
cmd_90632_cs(uint8_t sa, uint8_t channel_mask, const char *input)
{//calib param - emissivity - refresh rate - ..
  Mlx90632Device *mlx = cmd_90632_get_handle(sa);
  if (mlx == NULL)
  { // failed to get handle!
    return;
  }

  if (mlx->slave_address_ == 0) // only the case of a new handle!
  {
    _mlx90632_initialize(mlx, sa);
  }

  MLX90632_RefreshRate rr;
  MLX90632_Reg_Mode mode;
  float ha = (1.0f / (1<<14)) * mlx->calib_data_.Ha_;
  float hb = (1.0f / (1<<10)) * mlx->calib_data_.Hb_;
  float emissivity = _mlx90632_drv_get_emissivity(mlx);
  _mlx90632_ee_read_refresh_rate(mlx, &rr);
  _mlx90632_reg_read_mode(mlx, &mode);


  char buf[16]; memset(buf, 0, sizeof(buf));
  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":SA=", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RR=", 0);
  uint8_to_hex(buf, rr);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":EM=", 0);
  const char *p = my_dtostrf(emissivity, 10, 3, buf);
  while (*p == ' ') p++; // remove leading space
  send_answer_chunk(channel_mask, p, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":Ha=", 0);
  p = my_dtostrf(ha, 10, 3, buf);
  while (*p == ' ') p++; // remove leading space
  send_answer_chunk(channel_mask, p, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":Hb=", 0);
  p = my_dtostrf(hb, 10, 3, buf);
  while (*p == ' ') p++; // remove leading space
  send_answer_chunk(channel_mask, p, 1);

  p = "(Unknown)";
  if (mode == MLX90632_REG_MODE_HALT) p = "(HALT)";
  if (mode == MLX90632_REG_MODE_SLEEPING_STEP) p = "(SLEEPING_STEP)";
  if (mode == MLX90632_REG_MODE_STEP) p = "(STEP)";
  if (mode == MLX90632_REG_MODE_CONTINIOUS) p = "(CONTINIOUS)";

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":MODE=", 0);
  uint8_to_hex(buf, mode);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, p, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_HEADER=TA,TO", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_UNIT=DegC,DegC", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_RES=" xstr(MLX90632_LSB_C) "," xstr(MLX90632_LSB_C), 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:EE_VERSION=0x", 0);
  uint16_to_hex(buf, mlx->calib_data_.eeprom_version_);
  send_answer_chunk(channel_mask, buf, 1);


  uint16_t data = 0;
  _mlx90632_i2c_read(sa, 0x2409, &data); // PRODUCT CODE
  data &= 0x0F; // ACC
  p = "Unknown";
  if (data == 1) p = "Medical";
  if (data == 2) p = "Commercial";
  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:ACC=", 0);
  send_answer_chunk(channel_mask, p, 1);

  data = 0;
  _mlx90632_i2c_read(sa, 0x2404, &data); // trick to read I2C LVL config
  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  if (data & 0x0008)
  {
    send_answer_chunk(channel_mask, ":RO:I2C=1(1V8)", 1);
  } else
  {
    send_answer_chunk(channel_mask, ":RO:I2C=0(3V3)", 1);
  }
}


void
cmd_90632_cs_write(uint8_t sa, uint8_t channel_mask, const char *input)
{ // EM/RR/Ha/Hb/MODE/SA
  char buf[16]; memset(buf, 0, sizeof(buf));
  Mlx90632Device *mlx = cmd_90632_get_handle(sa);
  if (mlx == NULL)
  { // failed to get handle!
    return;
  }

  if (mlx->slave_address_ == 0) // only the case of a new handle!
  {
    _mlx90632_initialize(mlx, sa);
  }

  const char *var_name = "EM=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    float em = atof(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((em > 0.1) && (em <= 1.0))
    {
      _mlx90632_drv_set_emissivity(mlx, em);
      send_answer_chunk(channel_mask, ":EM=OK [non-EE]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":EM=FAIL; outbound", 1);
    }
    return;
  }

  var_name = "RR=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t rr = atoi(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((rr >= 0) && (rr <= 7))
    {
      _mlx90632_ee_write_refresh_rate(mlx, MLX90632_RefreshRate(rr));
      send_answer_chunk(channel_mask, ":RR=OK [mlx-EE]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":RR=FAIL; outbound", 1);
    }
    return;
  }

  var_name = "Ha=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    const char *value = input+strlen(var_name);
    // search for '.' => interpret as float; if not hex!
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    uint16_t ha_ee = 0;
    for (uint8_t i=0; i<strlen(value); i++)
    {
      if (value[i] == '.')
      {
        ha_ee = 1;
      }
    }
    if (ha_ee)
    {
      float ha = atof(value);
      int32_t code = int32_t((ha * (1UL << 14)) + 0.5f);
      if ( (code < 0) || (code >= (1<<16)) )
      {// overflow!
        send_answer_chunk(channel_mask, ":Ha=FAIL; float outbound", 1);
        return;
      } else
      {
        ha_ee = uint16_t(code & 0x0000FFFF);
      }
    } else
    {
      int32_t v = atohex16(value);
      if (v < 0)
      {
        send_answer_chunk(channel_mask, ":Ha=FAIL; hex parse error", 1);
        return;
      } else
      {
        ha_ee = uint16_t(v);
      }
    }
    // ok now we have ha_ee; let's store into to the EEPROM.
    _mlx90632_ee_write(mlx, 0x2481, ha_ee);
    // re-read the (updated) calibration parameters from EE
    _mlx90632_read_calib_parameters(mlx);

    send_answer_chunk(channel_mask, ":Ha=OK [mlx-EE]", 1);
    return;
  }

  var_name = "Hb=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    const char *value = input+strlen(var_name);
    // search for '.' => interpret as float; if not hex!
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    uint16_t hb_ee = 0;
    for (uint8_t i=0; i<strlen(value); i++)
    {
      if (value[i] == '.')
      {
        hb_ee = 1;
      }
    }
    if (hb_ee)
    {
      float hb = atof(value);
      int32_t code = int32_t((hb * (1 << 10)) + 0.5f);
      if ((code > ((1<<15) - 1)) || (code < -(1<<15)))
      {// overflow!
        send_answer_chunk(channel_mask, ":Hb=FAIL; float outbound", 1);
        return;
      } else
      {
        hb_ee = uint16_t(code & 0x0000FFFF);
      }
    } else
    {
      int32_t v = atohex16(value);
      if (v < 0)
      {
        send_answer_chunk(channel_mask, ":Hb=FAIL; hex parse error", 1);
        return;
      } else
      {
        hb_ee = uint16_t(v);
      }
    }
    // ok now we have ha_ee; let's store into to the EEPROM.
    _mlx90632_ee_write(mlx, 0x2482, hb_ee);
    // re-read the (updated) calibration parameters from EE
    _mlx90632_read_calib_parameters(mlx);

    send_answer_chunk(channel_mask, ":Hb=OK [mlx-EE]", 1);
    return;
  }

  var_name = "MODE=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t mode = atoi(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((mode >= 0) && (mode <= 3))
    {
      _mlx90632_reg_write_mode(mlx, MLX90632_Reg_Mode(mode));
      send_answer_chunk(channel_mask, ":MODE=OK [mlx-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":MODE=FAIL; outbound", 1);
    }
    return;
  }

  var_name = "SA=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t new_sa = atohex8(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);

    if (new_sa == sa)
    {
      send_answer_chunk(channel_mask, ":SA=same; not updated", 1);
      return;
    }

    WIRE.endTransmission();
    delayMicroseconds(5);

    WIRE.beginTransmission(new_sa);
    byte result = WIRE.endTransmission();     // stop transmitting

    if (result != 2)
    {
      send_answer_chunk(channel_mask, ":SA=FAIL '", 0);
      uint8_to_hex(buf, new_sa);
      send_answer_chunk(channel_mask, buf, 0);
      send_answer_chunk(channel_mask, "' is in use; not updated", 1);
      return;
    }

    if ((new_sa >= 3) && (new_sa <= 126))
    { // trick to update the SA (Slave Address) in the EEPROM
      uint16_t old_value = 0;
      _mlx90632_i2c_read(sa, 0x24D5, &old_value);
      _mlx90632_i2c_write(sa, 0x24D5, 0);
      uint16_t new_value = old_value & ~0x003F;
      new_value |= (new_sa >> 1);
      _mlx90632_i2c_write(sa, 0x24D5, new_value);

      _mlx90632_soft_reset(mlx); // activate the new slave address!
      cmd_90632_tear_down(sa); // as new SA, tear down this one!

      if (g_active_slave == sa)
      {
        g_active_slave = new_sa;
      }
      // we assume that the new SA will use the same driver!
      g_device_list[new_sa] = g_device_list[sa];

      send_answer_chunk(channel_mask, ":SA=OK! [mlx-EE] (use 'scan' to discover the new SA)", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":SA=FAIL; outbound", 1);
    }
    return;
  }

  send_answer_chunk(channel_mask, "+cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":FAIL; unknown variable", 1);
}


void
cmd_90632_ee(uint8_t sa, uint16_t *ee_data, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message)
{ // read all the EEPROM!
  *ee_start_address = 0x2400;
  Mlx90632Device *mlx = cmd_90632_get_handle(sa);
  if (mlx == NULL)
  { // failed to get handle!
    *ee_count = 0;
    *error_message = MLX90632_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (*ee_count < 256)
  {
    *ee_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90632_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *ee_count = 256;

  if (_mlx90632_i2c_read_block(sa, *ee_start_address, ee_data, 256) != 0)
  {
    *ee_count = 0;
    *error_message = MLX90632_ERROR_COMMUNICATION;
    return;
  }
}


void
cmd_90632_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message)
{
  Mlx90632Device *mlx = cmd_90632_get_handle(sa);
  if (mlx == NULL)
  { // failed to get handle!
    *raw_count = 0;
    *error_message = MLX90632_ERROR_NO_FREE_HANDLE;
    return;
  }

  if (mlx->slave_address_ == 0) // only the case of a new handle!
  {
    _mlx90632_initialize(mlx, sa);
  }

  if (*raw_count < 6)
  {
    *raw_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90632_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *raw_count = 6;

  int16_t e = _mlx90632_read_adc(mlx);
  if (e != 0)
  {
    *raw_count = 0;
    *error_message = MLX90632_ERROR_COMMUNICATION;
    return;
  }
  uint16_t *pointer = (uint16_t *)&mlx->adc_data_;

  for (int16_t i=0; i<6; i++)
  {
    raw_list[i] = pointer[i];
  }
}


void
cmd_90632_tear_down(uint8_t sa)
{
  delete_handle(sa);
}

#ifdef __cplusplus
}
#endif
