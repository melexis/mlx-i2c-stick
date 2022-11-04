#include "mlx90614_api.h"
#include "mlx90614_smbus_driver.h"
#include "mlx90614_cmd.h"
#include <string.h>
#include <Arduino.h>
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_MLX90614_SLAVES
#define MAX_MLX90614_SLAVES 8
#endif // MAX_MLX90614_SLAVES

#define MLX90614_ERROR_BUFFER_TOO_SMALL "Buffer too small"
#define MLX90614_ERROR_COMMUNICATION "Communication error"
#define MLX90614_ERROR_NO_FREE_HANDLE "No free handle; pls recompile firmware with higher 'MAX_MLX90614_SLAVES'"


static MLX90614_t *g_mlx90614_list[MAX_MLX90614_SLAVES];


int16_t atohex8(const char *in);


MLX90614_t *
cmd_90614_get_handle(uint8_t sa)
{
  if (sa >= 128)
  {
    return NULL;
  }

  for (uint8_t i=0; i<MAX_MLX90614_SLAVES; i++)
  {
    if (!g_mlx90614_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_mlx90614_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      return g_mlx90614_list[i];
    }
  }

  // not found => try to find a handle with slave address zero (not yet initialized)!
  for (uint8_t i=0; i<MAX_MLX90614_SLAVES; i++)
  {
    if (!g_mlx90614_list[i])
    {
      continue; // allow empty spots!
    }
    if (g_mlx90614_list[i]->slave_address_ == 0)
    { // found!
      return g_mlx90614_list[i];
    }
  }

  // not found => use first free spot!
  uint8_t i=0;
  for (; i<MAX_MLX90614_SLAVES; i++)
  {
    if (g_mlx90614_list[i] == NULL)
    {
      g_mlx90614_list[i] = (MLX90614_t *)malloc(sizeof(MLX90614_t));;
      memset(g_mlx90614_list[i], 0, sizeof(MLX90614_t));
      g_mlx90614_list[i]->slave_address_ = 0x80 | sa;
      return g_mlx90614_list[i];
    }
  }

  return NULL; // no free spot available
}


static void
delete_handle(uint8_t sa)
{
  for (uint8_t i=0; i<MAX_MLX90614_SLAVES; i++)
  {
    if (!g_mlx90614_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_mlx90614_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      memset(g_mlx90614_list[i], 0, sizeof(MLX90614_t));
      free(g_mlx90614_list[i]);
      g_mlx90614_list[i] = NULL;
    }
  }
}


void
cmd_90614_init(uint8_t sa)
{
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  // init functions goes here

  // turn off bit7, to indicate other routines this slave has been init
  mlx->slave_address_ &= 0x7F;
}


void
cmd_90614_tear_down(uint8_t sa)
{ // nothing special to do, just release all associated memory
  delete_handle(sa);
}


void
cmd_90614_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message)
{
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    *mv_count = 0;
    *error_message = MLX90614_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90614_init(sa);
  }

  if (*mv_count <= 2)
  {
    *mv_count = 0;
    *error_message = MLX90614_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *mv_count = 2;
  int ret = MLX90614_GetTa(sa, &mv_list[0]);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
    return;
  }
  ret = MLX90614_GetTo(sa, &mv_list[1]);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
    return;
  }
}


void
cmd_90614_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message)
{
  int16_t ir_data1;
  int16_t ir_data2;
  uint16_t data;
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    *raw_count = 0;
    *error_message = MLX90614_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90614_init(sa);
  }

  if (*raw_count < 3)
  {
    *raw_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90614_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *raw_count = 3;

  int ret = MLX90614_SMBusRead(sa, 0x06, &raw_list[0]); // TA
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
  }
  ret = MLX90614_GetIRdata1(sa, &data);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
  }
  ir_data1 = data;
  if (data & 0x8000) ir_data1 = -(data & ~0x8000);
  raw_list[1] = (uint16_t)(ir_data1);
  ret = MLX90614_GetIRdata2(sa, &data);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
  }
  ir_data2 = data;
  if (data & 0x8000) ir_data2 = -(data & ~0x8000);
  raw_list[2] = (uint16_t)(ir_data1);
}


void
cmd_90614_nd(uint8_t sa, uint8_t *nd, char const **error_message)
{
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    *error_message = MLX90614_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90614_init(sa);
  }

  if (mlx->nd_timer_ == 0)
  { // first time
    mlx->nd_timer_ = millis();
    *nd = 1; // first time there is new data
    return;
  }
  *nd = 0;
  if ((millis() - mlx->nd_timer_) > 200)
  {
    mlx->nd_timer_ = millis();
    *nd = 1;
  }
}


void
cmd_90614_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message)
{
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    *sn_count = 0;
    *error_message = MLX90614_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90614_init(sa);
  }

  if (*sn_count < 4)
  {
    *sn_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90614_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *sn_count = 4;

  int ret = 0;
  ret = MLX90614_SMBusRead(sa, 0x3C, &sn_list[0]);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
  }
  ret = MLX90614_SMBusRead(sa, 0x3D, &sn_list[1]);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
  }
  ret = MLX90614_SMBusRead(sa, 0x3E, &sn_list[2]);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
  }
  ret = MLX90614_SMBusRead(sa, 0x3F, &sn_list[3]);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
  }
}


void
cmd_90614_cs(uint8_t sa, uint8_t channel_mask, const char *input)
{//emissivity - IIR - FIR - ND
  float em = 1;
  uint8_t fir = 255;
  uint8_t iir = 255;
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90614_init(sa);
  }

  MLX90614_GetEmissivity(sa, &em);
  MLX90614_GetFIR(sa, &fir);
  MLX90614_GetIIR(sa, &iir);

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
  const char *p = my_dtostrf(em, 10, 3, buf);
  while (*p == ' ') p++; // remove leading space
  send_answer_chunk(channel_mask, p, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":FIR=", 0);
  uint8_to_hex(buf, fir);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":IIR=", 0);
  uint8_to_hex(buf, iir);
  send_answer_chunk(channel_mask, buf, 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":ND=", 0);
  uint32_to_dec(buf, mlx->nd_timer_, 8);
  send_answer_chunk(channel_mask, buf, 1);

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
  send_answer_chunk(channel_mask, ":RO:MV_RES=" xstr(MLX90614_LSB_C) "," xstr(MLX90614_LSB_C), 1);
}


void
cmd_90614_cs_write(uint8_t sa, uint8_t channel_mask, const char *input)
{//emissivity - IIR - FIR + new:SA - ND
  char buf[16]; memset(buf, 0, sizeof(buf));
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90614_init(sa);
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
      MLX90614_SetEmissivity(sa, em);
      send_answer_chunk(channel_mask, ":EM=OK [mlx-EE]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":EM=FAIL; outbound", 1);
    }
    return;
  }
  var_name = "FIR=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t fir = atoi(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((fir >= 0) && (fir <= 7))
    {
      MLX90614_SetFIR(sa, fir);
      send_answer_chunk(channel_mask, ":FIR=OK [mlx-EE]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":FIR=FAIL; outbound", 1);
    }
    return;
  }
  var_name = "IIR=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t iir = atoi(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((iir >= 0) && (iir <= 7))
    {
      MLX90614_SetIIR(sa, iir);
      send_answer_chunk(channel_mask, ":IIR=OK [mlx-EE]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":IIR=FAIL; outbound", 1);
    }
    return;
  }

  var_name = "ND=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t nd = atoi(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((nd >= 0) && (nd <= 1000))
    {
      mlx->nd_timer_ = nd;
      send_answer_chunk(channel_mask, ":ND=OK [hub-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":ND=FAIL; outbound", 1);
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
      uint16_t value = 0;
      int16_t error = MLX90614_SMBusRead(sa, 0x2E, &value);
      new_sa |= (value & 0xFF00);
      if (error == 0)
      {
        error = MLX90614_SMBusWrite(sa, 0x2E, 0x0000);
        if(error == 0)
        {
          error = MLX90614_SMBusWrite(sa, 0x2E, new_sa);
        }
      }

      // as the SA is only updated after hardware reset => next lines are commented out...

      // keep same slave active after adress update.
      // if (g_active_slave == sa)
      // {
      //   g_active_slave = new_sa;
      // }
      // // disconnect old-SA.
      // cmd_90614_tear_down(sa);
      // g_device_list[sa] &= ~0x80; // reset 'found' bit on 'old'-SA.            
      // we assume that the new SA will use the same driver!
      g_device_list[new_sa] = g_device_list[sa] & ~0x80;
      // g_device_list[new_sa] |= 0x80; // set 'found' bit on 'new'-SA. (here not yet active...)

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
cmd_90614_ee(uint8_t sa, uint16_t *ee_data, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message)
{ // read all the EEPROM!
  MLX90614_t *mlx = cmd_90614_get_handle(sa);
  if (mlx == NULL)
  {
    *ee_count = 0;
    *error_message = MLX90614_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90614_init(sa);
  }

  *ee_start_address = 0x20;
  if (*ee_count < 32)
  {
    *ee_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90614_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *ee_count = 32;
  int ret = MLX90614_DumpEE(sa, ee_data);
  if (ret < 0)
  {
    *error_message = MLX90614_ERROR_COMMUNICATION;
    return;
  }
}


#ifdef __cplusplus
}
#endif
