// #include "mlx90393_api.h"
#include "mlx90393_cmd.h"
#include <string.h>
#include <stdint.h>
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAX_MLX90393_SLAVES
#define MAX_MLX90393_SLAVES 8
#endif // MAX_MLX90393_SLAVES

#define MLX90393_ERROR_BUFFER_TOO_SMALL "Buffer too small"
#define MLX90393_ERROR_COMMUNICATION "Communication error"
#define MLX90393_ERROR_NO_FREE_HANDLE "No free handle; pls recompile firmware with higher 'MAX_MLX90393_SLAVES'"

static MLX90393_t *g_mlx90393_list[MAX_MLX90393_SLAVES];


MLX90393_t *
cmd_90393_get_handle(uint8_t sa)
{
  if (sa >= 128)
  {
    return NULL;
  }

  for (uint8_t i=0; i<MAX_MLX90393_SLAVES; i++)
  {
    if (!g_mlx90393_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_mlx90393_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      return g_mlx90393_list[i];
    }
  }

  // not found => try to find a handle with slave address zero (not yet initialized)!
  for (uint8_t i=0; i<MAX_MLX90393_SLAVES; i++)
  {
    if (!g_mlx90393_list[i])
    {
      continue; // allow empty spots!
    }
    if (g_mlx90393_list[i]->slave_address_ == 0)
    { // found!
      return g_mlx90393_list[i];
    }
  }

  // not found => use first free spot!
  uint8_t i=0;
  for (; i<MAX_MLX90393_SLAVES; i++)
  {
    if (g_mlx90393_list[i] == NULL)
    {
      g_mlx90393_list[i] = (MLX90393_t *)malloc(sizeof(MLX90393_t));;
      memset(g_mlx90393_list[i], 0, sizeof(MLX90393_t));
      g_mlx90393_list[i]->slave_address_ = 0x80 | sa;
      return g_mlx90393_list[i];
    }
  }

  return NULL; // no free spot available
}


static void
delete_handle(uint8_t sa)
{
  for (uint8_t i=0; i<MAX_MLX90393_SLAVES; i++)
  {
    if (!g_mlx90393_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_mlx90393_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      memset(g_mlx90393_list[i], 0, sizeof(MLX90393_t));
      free(g_mlx90393_list[i]);
      g_mlx90393_list[i] = NULL;
    }
  }
}


void
cmd_90393_init(uint8_t sa)
{
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  // init functions goes here

  // turn off bit7, to indicate other routines this slave has been init
  mlx->slave_address_ &= 0x7F;
}


void
cmd_90393_tear_down(uint8_t sa)
{ // nothing special to do, just release all associated memory
  delete_handle(sa);
}


void
cmd_90393_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message)
{
  uint16_t frame_data[242];
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    *mv_count = 0;
    *error_message = MLX90393_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90393_init(sa);
  }

  if (*mv_count < 3+1)
  {
    *error_message = MLX90393_ERROR_BUFFER_TOO_SMALL;
    *mv_count = 0;
    return;
  }
  *mv_count = 3+1;

  // get the measurement values
  // default order is TA(sensor temperature), measurement value1, 2, 3, ..
  // in 90363 the order would be: TA/X/Y/Z for example.
}


void
cmd_90393_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message)
{
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    *raw_count = 0;
    *error_message = MLX90393_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90393_init(sa);
  }

  if (*raw_count < 4)
  {
    *raw_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90393_ERROR_BUFFER_TOO_SMALL;
    return; 
  }
  *raw_count = 4;

}


void
cmd_90393_nd(uint8_t sa, uint8_t *nd, char const **error_message)
{ // check if New Data is available.
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    *error_message = MLX90393_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90393_init(sa);
  }
  *nd = 0;

  // *nd = ((statusRegister & 0x0008) == 0x0008) ? 1 : 0;
}


void
cmd_90393_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message)
{ // read the Serial Number from the EEPROM
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    *sn_count = 0;
    *error_message = MLX90393_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90393_init(sa);
  }
  if (*sn_count < 4)
  {
    *sn_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90393_ERROR_BUFFER_TOO_SMALL;
    return; 
  }
  *sn_count = 4;
  // if (MLX90393_I2CRead(sa, 0x2407, 3, sn_list))
  // {
  //   *error_message = MLX90393_ERROR_COMMUNICATION;
  //   return;
  // }
}


void
cmd_90393_cs(uint8_t sa, uint8_t channel_mask, const char *input)
{// Configure Slave; refresh rate(RR), Mode, ..
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90393_init(sa);
  }

  char buf[16]; memset(buf, 0, sizeof(buf));
  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_HEADER=TA,X,Y,Z", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_UNIT=DegC,uT,uT,uT", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_RES=" xstr(MLX90393_LSB_C) "," xstr(MLX90393_LSB_uT) "," xstr(MLX90393_LSB_uT) "," xstr(MLX90393_LSB_uT), 1);



  // uint8_t rr = MLX90393_GetRefreshRate(sa);
  // uint8_t res = MLX90393_GetCurResolution(sa);

  // char buf[16]; memset(buf, 0, sizeof(buf));
  // send_answer_chunk(channel_mask, "cs:", 0);
  // uint8_to_hex(buf, sa);
  // send_answer_chunk(channel_mask, buf, 0);
  // send_answer_chunk(channel_mask, ":SA=", 0);
  // uint8_to_hex(buf, sa);
  // send_answer_chunk(channel_mask, buf, 1);

  // send_answer_chunk(channel_mask, "cs:", 0);
  // uint8_to_hex(buf, sa);
  // send_answer_chunk(channel_mask, buf, 0);
  // send_answer_chunk(channel_mask, ":EM=", 0);
  // const char *p = my_dtostrf(mlx->emissivity_, 10, 3, buf);
  // while (*p == ' ') p++; // remove leading space
  // send_answer_chunk(channel_mask, p, 1);

  // send_answer_chunk(channel_mask, "cs:", 0);
  // uint8_to_hex(buf, sa);
  // send_answer_chunk(channel_mask, buf, 0);
  // send_answer_chunk(channel_mask, ":TR=", 0);
  // p = my_dtostrf(mlx->t_room_, 10, 1, buf);
  // while (*p == ' ') p++; // remove leading space
  // send_answer_chunk(channel_mask, p, 1);

  // send_answer_chunk(channel_mask, "cs:", 0);
  // uint8_to_hex(buf, sa);
  // send_answer_chunk(channel_mask, buf, 0);
  // send_answer_chunk(channel_mask, ":RR=", 0);
  // uint8_to_hex(buf, rr);
  // send_answer_chunk(channel_mask, buf, 1);

  // send_answer_chunk(channel_mask, "cs:", 0);
  // uint8_to_hex(buf, sa);
  // send_answer_chunk(channel_mask, buf, 0);
  // send_answer_chunk(channel_mask, ":RES=", 0);
  // uint8_to_hex(buf, res);
  // send_answer_chunk(channel_mask, buf, 1);
}


void
cmd_90393_cs_write(uint8_t sa, uint8_t channel_mask, const char *input)
{//Write configuration to slave; refresh rate(RR), Mode, ..
  char buf[16]; memset(buf, 0, sizeof(buf));
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90393_init(sa);
  }
  const char *var_name = "RR=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int rr = atoi(input+strlen(var_name));
    send_answer_chunk(channel_mask, "+cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    if ((rr > 0) && (rr <= 7))
    {
      // write the refresh rate to the mlx sensor
      send_answer_chunk(channel_mask, ":EM=OK [mlx-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":EM=FAIL; outbound", 1);
    }
    return;
  }

  send_answer_chunk(channel_mask, "+cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":FAIL; unknown variable", 1);
}


void
cmd_90393_ee(uint8_t sa, uint16_t *ee_data, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message)
{ // read all the EEPROM!
  *ee_start_address = 0x2400;
  MLX90393_t *mlx = cmd_90393_get_handle(sa);
  if (mlx == NULL)
  {
    *ee_count = 0;
    *error_message = MLX90393_ERROR_NO_FREE_HANDLE;
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90393_init(sa);
  }
  if (*ee_count < 832)
  {
    *ee_count = 0; // input buffer not long enough, report nothing.
    *error_message = MLX90393_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *ee_count = 832;

  // if (MLX90393_DumpEE(sa, ee_data) < 0)
  // {
  //   *error_message = MLX90393_ERROR_COMMUNICATION;
  //   return;
  // }
}



#ifdef __cplusplus
}
#endif
