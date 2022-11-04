#include "mlx90641_api.h"
#include "mlx90641_i2c_driver.h"
#include "mlx90641_cmd.h"
#include <string.h>
#include <stdint.h>
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

void cmd_90641_iir_filter(float *to_list, float *iir, uint8_t depth, float threshold);

#ifndef MAX_MLX90641_SLAVES
#define MAX_MLX90641_SLAVES 8
#endif // MAX_MLX90641_SLAVES

#define MLX90641_ERROR_BUFFER_TOO_SMALL "Buffer too small"
#define MLX90641_ERROR_COMMUNICATION "Communication error"
#define MLX90641_ERROR_NEW_DATA_SET "new_data bit set during read"
#define MLX90641_ERROR_NO_FREE_HANDLE "No free handle; pls recompile firmware with higher 'MAX_MLX90641_SLAVES'"

static MLX90641_t *g_mlx90641_list[MAX_MLX90641_SLAVES];


MLX90641_t *
cmd_90641_get_handle(uint8_t sa)
{
  if (sa >= 128)
  {
    return NULL;
  }

  for (uint8_t i=0; i<MAX_MLX90641_SLAVES; i++)
  {
    if (!g_mlx90641_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_mlx90641_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      return g_mlx90641_list[i];
    }
  }

  // not found => try to find a handle with slave address zero (not yet initialized)!
  for (uint8_t i=0; i<MAX_MLX90641_SLAVES; i++)
  {
    if (!g_mlx90641_list[i])
    {
      continue; // allow empty spots!
    }
    if (g_mlx90641_list[i]->slave_address_ == 0)
    { // found!
      return g_mlx90641_list[i];
    }
  }

  // not found => use first free spot!
  uint8_t i=0;
  for (; i<MAX_MLX90641_SLAVES; i++)
  {
    if (g_mlx90641_list[i] == NULL)
    {
      g_mlx90641_list[i] = (MLX90641_t *)malloc(sizeof(MLX90641_t));;
      memset(g_mlx90641_list[i], 0, sizeof(MLX90641_t));
      g_mlx90641_list[i]->slave_address_ = 0x80 | sa;
      return g_mlx90641_list[i];
    }
  }

  return NULL; // no free spot available
}


static void
delete_handle(uint8_t sa)
{
  for (uint8_t i=0; i<MAX_MLX90641_SLAVES; i++)
  {
    if (!g_mlx90641_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_mlx90641_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      if (g_mlx90641_list[i]->iir_ != NULL)
      {
        free(g_mlx90641_list[i]->iir_);
      }
      memset(g_mlx90641_list[i], 0, sizeof(MLX90641_t));
      free(g_mlx90641_list[i]);
      g_mlx90641_list[i] = NULL;
    }
  }
}


void
cmd_90641_init(uint8_t sa)
{
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  mlx->emissivity_ = 0.95f;
  mlx->t_room_ = 25.0f;
  mlx->flags_ |= (1U<<MLX90641_CMD_FLAG_BROKEN_PIXELS);
  mlx->flags_ |= (1U<<MLX90641_CMD_FLAG_IIR_FILTER);

  uint16_t ee_data[832];

  MLX90641_I2CInit();
  MLX90641_DumpEE(sa, ee_data);
  MLX90641_ExtractParameters(ee_data, &mlx->mlx90641_);
  MLX90641_SetRefreshRate(sa, 5);
  mlx->slave_address_ &= 0x7F;
}


void
cmd_90641_tear_down(uint8_t sa)
{ // nothing special to do, just release all associated memory
  delete_handle(sa);
}


void
cmd_90641_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message)
{
  uint16_t frame_data[242];
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    *mv_count = 0;
    *error_message = MLX90641_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90641_init(sa);
  }

  if (*mv_count < 192+1)
  {
    *error_message = MLX90641_ERROR_BUFFER_TOO_SMALL;
    *mv_count = 0;
    return;
  }
  *mv_count = 192+1;

  int e = MLX90641_GetFrameData(sa, frame_data);

  if (e < 0)
  {
    // allow one retry!
    MLX90641_I2CWrite(sa, 0x8000, 0x0030); // clear new data flag...
    e = MLX90641_GetFrameData(sa, frame_data);
    if (e < 0)
    {
      *mv_count = 0;
      *error_message = MLX90641_ERROR_COMMUNICATION;
      if (e == -16)
      {
        *error_message = MLX90641_ERROR_NEW_DATA_SET;
      }
      return;
    }
  }

  MLX90641_CalculateTo(frame_data, &mlx->mlx90641_, mlx->emissivity_, mlx->t_room_, &mv_list[1]);
  mv_list[0] = MLX90641_GetTa(frame_data, &mlx->mlx90641_);

  if (mlx->flags_ & (1U<<MLX90641_CMD_FLAG_BROKEN_PIXELS))
  {
    MLX90641_BadPixelsCorrection(mlx->mlx90641_.brokenPixel, &mv_list[1]);
  }

  if (mlx->flags_ & (1U<<MLX90641_CMD_FLAG_IIR_FILTER))
  {
    if (mlx->iir_ == NULL)
    {
      mlx->iir_ = (float *)malloc(sizeof(float) * 192);
      for (uint16_t pix=0; pix<192; pix++)
      {
        if ((-100 < mv_list[pix+1]) && (mv_list[pix+1] < 1000))
        {
          mlx->iir_[pix] = mv_list[pix+1];
        } else
        {
          mlx->iir_[pix] = mv_list[0];
        }
      }
    }
    cmd_90641_iir_filter(&mv_list[1], mlx->iir_, 8, 2.5);
    for (uint16_t pix=0; pix<192; pix++)
    {
      mv_list[pix+1] = mlx->iir_[pix];
    }
  }
}


void
cmd_90641_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message)
{
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    *raw_count = 0;
    *error_message = MLX90641_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90641_init(sa);
  }

  if (*raw_count < 242)
  {
    *raw_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90641_ERROR_BUFFER_TOO_SMALL;
    return; 
  }
  *raw_count = 242;

  int e = MLX90641_GetFrameData(sa, raw_list);
  if (e < 0)
  {
    *error_message = MLX90641_ERROR_COMMUNICATION;
    if (e == -16)
    {
      *error_message = MLX90641_ERROR_NEW_DATA_SET;
    }
    return;
  }
}


void
cmd_90641_nd(uint8_t sa, uint8_t *nd, char const **error_message)
{
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    *error_message = MLX90641_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90641_init(sa);
  }
  *nd = 0;

  uint16_t statusRegister;
  int error = MLX90641_I2CRead(sa, 0x8000, 1, &statusRegister);
  if(error != 0)
  {
    *error_message = MLX90641_ERROR_COMMUNICATION;
    return;
  }
  *nd = ((statusRegister & 0x0008) == 0x0008) ? 1 : 0;
}


void
cmd_90641_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message)
{
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    *sn_count = 0;
    *error_message = MLX90641_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90641_init(sa);
  }
  if (*sn_count < 3)
  {
    *sn_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90641_ERROR_BUFFER_TOO_SMALL;
    return; 
  }
  *sn_count = 3;
  if (MLX90641_I2CRead(sa, 0x2407, 3, sn_list))
  {
    *error_message = MLX90641_ERROR_COMMUNICATION;
    return;
  }
}


void
cmd_90641_cs(uint8_t sa, uint8_t channel_mask, const char *input)
{//emissivity - TR - RR - RES - FLAGS
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90641_init(sa);
  }
  uint8_t rr = MLX90641_GetRefreshRate(sa);
  uint8_t res = MLX90641_GetCurResolution(sa);

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
  send_answer_chunk(channel_mask, ":EM=", 0);
  const char *p = my_dtostrf(mlx->emissivity_, 10, 3, buf);
  while (*p == ' ') p++; // remove leading space
  send_answer_chunk(channel_mask, p, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":TR=", 0);
  p = my_dtostrf(mlx->t_room_, 10, 1, buf);
  while (*p == ' ') p++; // remove leading space
  send_answer_chunk(channel_mask, p, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RR=", 0);
  uint8_to_hex(buf, rr);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RES=", 0);
  uint8_to_hex(buf, res);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":FLAGS=", 0);
  uint8_to_hex(buf, mlx->flags_);
  send_answer_chunk(channel_mask, buf, 0);

  uint8_t is_first_flag = 1;
  

  if (mlx->flags_ & (1U<<MLX90641_CMD_FLAG_BROKEN_PIXELS))
  {
    if (is_first_flag) 
    { 
      is_first_flag = 0; 
      send_answer_chunk(channel_mask, "(", 0); 
    } else
    {
      send_answer_chunk(channel_mask, ",", 0);       
    }
    send_answer_chunk(channel_mask, "CORRECT_BROKEN_PIXELS", 0);
  }
  if (mlx->flags_ & (1U<<MLX90641_CMD_FLAG_IIR_FILTER))
  {
    if (is_first_flag) 
    { 
      is_first_flag = 0; 
      send_answer_chunk(channel_mask, "(", 0); 
    } else
    {
      send_answer_chunk(channel_mask, ",", 0);       
    }
    send_answer_chunk(channel_mask, "IIR_FILTER", 0);
  }
  send_answer_chunk(channel_mask, (is_first_flag == 0) ? ")" : "", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_HEADER=TA,TO_[192]", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_UNIT=DegC,DegC[192]", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_RES=" xstr(MLX90641_LSB_C) "," xstr(MLX90641_LSB_C) "[192]", 1);
}


void
cmd_90641_cs_write(uint8_t sa, uint8_t channel_mask, const char *input)
{//emissivity - TR - RR - RES - FLAGS
  char buf[16]; memset(buf, 0, sizeof(buf));
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90641_init(sa);
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
      mlx->emissivity_ = em;
      send_answer_chunk(channel_mask, ":EM=OK [hub-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":EM=FAIL; outbound", 1);
    }
    return;
  }
  var_name = "TR=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    float t_room = atof(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((t_room >= -60) && (t_room <= 100))
    {
      mlx->t_room_ = t_room;
      send_answer_chunk(channel_mask, ":TR=OK [hub-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":TR=FAIL; outbound", 1);
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
      MLX90641_SetRefreshRate(sa, rr);
      send_answer_chunk(channel_mask, ":RR=OK [mlx-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":RR=FAIL; outbound", 1);
    }
    return;
  }
  var_name = "RES=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t res = atoi(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((res >= 0) && (res <= 7))
    {
      MLX90641_SetResolution(sa, res);
      send_answer_chunk(channel_mask, ":RES=OK [mlx-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":RES=FAIL; outbound", 1);
    }
    return;
  }
  var_name = "FLAGS=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if (!strcmp(input+strlen(var_name), "+BROKEN"))
    {
      mlx->flags_ |= (1U<<MLX90641_CMD_FLAG_BROKEN_PIXELS);
      send_answer_chunk(channel_mask, ":FLAGS=OK [hub-register]", 1);
    }
    else if (!strcmp(input+strlen(var_name), "-BROKEN"))
    {
      mlx->flags_ &= ~(1U<<MLX90641_CMD_FLAG_BROKEN_PIXELS);
      send_answer_chunk(channel_mask, ":FLAGS=OK [hub-register]", 1);
    }
    else if (!strcmp(input+strlen(var_name), "+IIR"))
    {
      mlx->flags_ |= (1U<<MLX90641_CMD_FLAG_IIR_FILTER);
      send_answer_chunk(channel_mask, ":FLAGS=OK [hub-register]", 1);
    }
    else if (!strcmp(input+strlen(var_name), "-IIR"))
    {
      mlx->flags_ &= ~(1U<<MLX90641_CMD_FLAG_IIR_FILTER);
      send_answer_chunk(channel_mask, ":FLAGS=OK [hub-register]", 1);
    }
    else
    {
      send_answer_chunk(channel_mask, ":FLAGS=FAIL; unknown value '", 0);
      send_answer_chunk(channel_mask, input+strlen(var_name), 0);
      send_answer_chunk(channel_mask, "'", 1);
    }
    return;
  }
  var_name = "SA=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    int16_t new_sa = atohex8(input+strlen(var_name));

    if (new_sa == sa)
    {
      send_answer_chunk(channel_mask, ":SA=same; not updated", 1);
      return;
    }

    WIRE.endTransmission();
    delayMicroseconds(5);

    // check if new address already is in use on the bus...
    WIRE.beginTransmission(new_sa);
    byte result = WIRE.endTransmission();     // stop transmitting
#ifdef ARDUINO_ADAFRUIT_QTPY_RP2040
    if (result == 4) result = 0; // ignore error=4 ('other error', but I can't seem to find anything wrong; only on this MCU platform)
#endif

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
      uint16_t value = 0;
      int16_t error = MLX90641_I2CRead(sa, 0x240F, 1, &value);
      value &= 0xFF00; // keep MSB
      value |= new_sa; // use new_sa as LSB (LSB = slave address)

      if (error == 0)
      {
        MLX90641_I2CWrite(sa, 0x240F, 0x0000); // erase EE
        MLX90641_I2CWrite(sa, 0x240F, value); // write to EE
        MLX90641_I2CWrite(sa, 0x8010, value); // write to RAM (new_sa is now active!)
      }

      // keep same slave active after adress update.
      if (g_active_slave == sa)
      {
        g_active_slave = new_sa;
      }

      // disconnect old-SA.
      cmd_90641_tear_down(sa);
      g_device_list[sa] &= ~0x80; // reset 'found' bit on 'old'-SA.

      // we assume that the new SA will use the same driver!
      g_device_list[new_sa] = g_device_list[sa];
      g_device_list[new_sa] |= 0x80; // set 'found' bit on 'new'-SA.


      send_answer_chunk(channel_mask, ":SA=OK! [mlx-EE]", 1);
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
cmd_90641_ee(uint8_t sa, uint16_t *ee_data, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message)
{ // read all the EEPROM!
  *ee_start_address = 0x2400;
  MLX90641_t *mlx = cmd_90641_get_handle(sa);
  if (mlx == NULL)
  {
    *ee_count = 0;
    *error_message = MLX90641_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90641_init(sa);
  }
  if (*ee_count < 832)
  {
    *ee_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90641_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *ee_count = 832;

  if (MLX90641_DumpEE(sa, ee_data) < 0)
  {
    *error_message = MLX90641_ERROR_COMMUNICATION;
    return;
  }
}


void cmd_90641_iir_filter(float *to_list, float *iir, uint8_t depth, float threshold)
{
  for (uint16_t pix=0; pix<192; pix++)
  {
    if ((-100 <= to_list[pix]) && (to_list[pix] <= 1000))
    {
      if (fabs(to_list[pix] - iir[pix]) >= threshold)
      {
        iir[pix] += ((to_list[pix] - iir[pix]) * 0.9f); // take 90% of the change.
      } else
      {
        iir[pix] = (to_list[pix] + (iir[pix] * (depth - 1))) / depth;
      }
    }
  }
}


#ifdef __cplusplus
}
#endif
