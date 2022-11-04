#include "mlx90640_api.h"
#include "mlx90640_i2c_driver.h"
#include "mlx90640_cmd.h"
#include "mlx90640_test.h"
#include <string.h>
#include <stdlib.h> 
#include <Arduino.h>
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"

#ifdef DEVICE_MLX90640_TEST_ENABLE

#ifdef __cplusplus
extern "C" {
#endif

static uint16_t g_frame_data[834];
  
void
cmd_90640_test(uint8_t sa, uint8_t channel_mask, const char *input)
{
  MLX90640_t *mlx = cmd_90640_get_handle(sa);
  if (mlx == NULL)
  {
    return;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90640_init(sa);
  }
  const char *var_name = "CACHE:EE=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    // 1. check input length is ok
    // 2. parse input hex-values into uint16_t array
    // 3. extract calibration and config parameters

    // 1. check input length is ok
    const char *p = input+strlen(var_name);

    Serial.print("len => "); Serial.println(strlen(p));
    Serial.println(p);

    if (strlen(input+strlen(var_name)) != (5*832-1))
    {
      send_answer_chunk(channel_mask, "CACHE:EE=FAIL; input data does not represent EE DATA", 1);
      return;
    }

    // 2. parse input hex-values into uint16_t array
    uint16_t ee_data[832];
    for (uint16_t i=0; i<832; i++, p+=5)
    {
      int32_t d = atohex16(p);
      if (d < 0)
      {
        send_answer_chunk(channel_mask, "CACHE:EE=FAIL; non-hexadecimal character detected", 1);
        return;
      }
      ee_data[i] = (uint16_t)(d);
    }

    // 3. extract calibration and config parameters
    MLX90640_ExtractParameters(ee_data, &mlx->mlx90640_);
    // todo: extract config.

    send_answer_chunk(channel_mask, "CACHE:EE=OK", 1);
    return;
  }


  var_name = "CACHE:FRAME_DATA=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    // 1. check input length is ok
    // 2. parse input hex-values into uint16_t array

    // 1. check input length is ok // todo: read rest of buffer from UART...
    const char *p = input+strlen(var_name);
    if (strlen(input+strlen(var_name)) != (5*834-1))
    {
      send_answer_chunk(channel_mask, "CACHE:FRAME_DATA=FAIL; input data does not represent FRAME DATA", 1);
      return;
    }

    // 2. parse input hex-values into uint16_t array
    for (uint16_t i=0; i<834; i++, p+=5)
    {
      int32_t d = atohex16(p);
      if (d < 0)
      {
        send_answer_chunk(channel_mask, "CACHE:FRAME_DATA=FAIL; non-hexadecimal character detected", 1);
        return;
      }
      g_frame_data[i] = (uint16_t)(d);
    }

    send_answer_chunk(channel_mask, "CACHE:FRAME_DATA=OK", 1);
    return;
  }

  var_name = "TIME:I2C_READ_EE";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    uint16_t ee_data[832];
    uint32_t counter = 0;
    uint32_t total_time_elapsed = 0; // in microseconds
    uint32_t total_time_target = 1000000; // in microseconds
    uint32_t start_time = micros();
    while (total_time_elapsed < total_time_target)
    {
      MLX90640_DumpEE(sa, ee_data);
      counter++;
      total_time_elapsed = micros() - start_time;
    }
    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);
    char buf[16]; memset(buf, 0, sizeof(buf));
    uint32_to_dec(buf, total_time_elapsed, 10);
    char *p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us, #", 0);
    uint32_to_dec(buf, counter, 10);
    p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);
    send_answer_chunk(channel_mask, ", ", 0);

    float time_per_hit = float(total_time_elapsed)/counter;
    p = my_dtostrf(time_per_hit, 10, 1, buf);
    while (*p == ' ') p++; // remove leading space
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us/hit", 1);
    return;
  }

  var_name = "TIME:I2C_READ_DATA_FRAME";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    uint32_t counter = 0;
    uint32_t total_time_elapsed = 0; // in microseconds
    uint32_t total_time_target = 1000000; // in microseconds
    uint32_t total_read_time = 0; // in microseconds
    uint32_t start_time = micros();

    // temporary go in fastest refresh rate mode.
    int rr_ori = MLX90640_GetRefreshRate(sa);  
    MLX90640_SetRefreshRate(sa, 7);

    while (total_time_elapsed < total_time_target)
    {
      uint8_t nd = 0;
      while (!nd) // make sure there is new data (avoid waiting for new data in getFrameData function)
      {
        cmd_90640_nd(sa, &nd);
      }
      uint32_t start_read_time = micros();
      MLX90640_GetFrameData(sa, g_frame_data);
      total_read_time += (micros() - start_read_time);
      counter++;
      total_time_elapsed = micros() - start_time;
    }

    // restore original refresh rate.
    MLX90640_SetRefreshRate(sa, rr_ori);


    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);
    char buf[16]; memset(buf, 0, sizeof(buf));
    uint32_to_dec(buf, total_time_elapsed, 10);
    char *p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us, ", 0);
    memset(buf, 0, sizeof(buf));
    uint32_to_dec(buf, total_read_time, 10);
    p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us, #", 0);

    uint32_to_dec(buf, counter, 10);
    p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);
    send_answer_chunk(channel_mask, ", ", 0);

    float time_per_hit = float(total_read_time)/counter;
    p = my_dtostrf(time_per_hit, 10, 1, buf);
    while (*p == ' ') p++; // remove leading space
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us/hit", 1);
    return;
  }


  // command is not yet handled, so try a couple other functions
  if (cmd_90640_test_float(sa, channel_mask, input) == 0)
  {
    return;
  }
  // cmd_90640_test_lut(sa, input, answer);
  // cmd_90640_test_int_lut(sa, input, answer);


  send_answer_chunk(channel_mask, "FAIL; unknown test", 1);
}


int8_t
cmd_90640_test_float(uint8_t sa, uint8_t channel_mask, const char *input)
{
  MLX90640_t *mlx = cmd_90640_get_handle(sa);
  if (mlx == NULL)
  {
    return -1;
  }
  if (mlx->slave_address_ & 0x80)
  {
    cmd_90640_init(sa);
  }

  const char *var_name = "FLOAT:TIME:TO";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    uint32_t counter = 0;
    uint32_t total_time_elapsed = 0; // in microseconds
    uint32_t total_time_target = 100000; // in microseconds
    uint32_t start_time = micros();
    float to_list[768];

    while (total_time_elapsed < total_time_target)
    {
      MLX90640_CalculateTo(g_frame_data, &mlx->mlx90640_, mlx->emissivity_, mlx->t_room_, to_list);
      counter++;
      total_time_elapsed = micros() - start_time;
    }

    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);
    char buf[16]; memset(buf, 0, sizeof(buf));
    uint32_to_dec(buf, total_time_elapsed, 10);
    char *p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us, #", 0);
    uint32_to_dec(buf, counter, 10);
    p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);
    send_answer_chunk(channel_mask, ", ", 0);

    float time_per_hit = float(total_time_elapsed)/counter;
    p = my_dtostrf(time_per_hit, 10, 1, buf);
    while (*p == ' ') p++; // remove leading space
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us/hit", 1);
    return 0;
  }
  
  var_name = "FLOAT:TIME:TA";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    uint32_t counter = 0;
    uint32_t total_time_elapsed = 0; // in microseconds
    uint32_t total_time_target = 100000; // in microseconds
    uint32_t start_time = micros();

    while (total_time_elapsed < total_time_target)
    {
      MLX90640_GetTa(g_frame_data, &mlx->mlx90640_);
      counter++;
      total_time_elapsed = micros() - start_time;
    }


    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);
    char buf[16]; memset(buf, 0, sizeof(buf));
    uint32_to_dec(buf, total_time_elapsed, 10);
    char *p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us, #", 0);
    uint32_to_dec(buf, counter, 10);
    p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);
    send_answer_chunk(channel_mask, ", ", 0);

    float time_per_hit = float(total_time_elapsed)/counter;
    p = my_dtostrf(time_per_hit, 10, 1, buf);
    while (*p == ' ') p++; // remove leading space
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us/hit", 1);
    return 0;
  }

  var_name = "FLOAT:TIME:VDD";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    uint32_t counter = 0;
    uint32_t total_time_elapsed = 0; // in microseconds
    uint32_t total_time_target = 100000; // in microseconds
    uint32_t start_time = micros();

    while (total_time_elapsed < total_time_target)
    {
      MLX90640_GetVdd(g_frame_data, &mlx->mlx90640_);
      counter++;
      total_time_elapsed = micros() - start_time;
    }
    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);
    char buf[16]; memset(buf, 0, sizeof(buf));
    uint32_to_dec(buf, total_time_elapsed, 10);
    char *p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us, #", 0);
    uint32_to_dec(buf, counter, 10);
    p = buf;
    while (*p == '0') p++;
    send_answer_chunk(channel_mask, p, 0);
    send_answer_chunk(channel_mask, ", ", 0);

    float time_per_hit = float(total_time_elapsed)/counter;
    p = my_dtostrf(time_per_hit, 10, 1, buf);
    while (*p == ' ') p++; // remove leading space
    send_answer_chunk(channel_mask, p, 0);

    send_answer_chunk(channel_mask, " us/hit", 1);
    return 0;
  }

  var_name = "FLOAT:CALC:VDD";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    float vdd = MLX90640_GetVdd(g_frame_data, &mlx->mlx90640_);

    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);

    char buf[16]; memset(buf, 0, sizeof(buf));
    const char *p = my_dtostrf(vdd, 10, 4, buf);
    while (*p == ' ') p++; // remove leading space
    send_answer_chunk(channel_mask, p, 0);
    send_answer_chunk(channel_mask, " V", 1);
    return 0;
  }

  var_name = "FLOAT:CALC:TA";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    float ta = MLX90640_GetTa(g_frame_data, &mlx->mlx90640_);
    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);

    char buf[16]; memset(buf, 0, sizeof(buf));
    const char *p = my_dtostrf(ta, 10, 4, buf);
    while (*p == ' ') p++; // remove leading space
    send_answer_chunk(channel_mask, p, 0);
    send_answer_chunk(channel_mask, " C", 1);
    return 0;
  }

  var_name = "FLOAT:CALC:TO";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    float to_list[768];
    memset(to_list, 0, sizeof(to_list));
    MLX90640_CalculateTo(g_frame_data, &mlx->mlx90640_, mlx->emissivity_, mlx->t_room_, to_list);
    // trick to compute both sub-pages.
    g_frame_data[833] ^= 0x0001;
    MLX90640_CalculateTo(g_frame_data, &mlx->mlx90640_, mlx->emissivity_, mlx->t_room_, to_list);
    g_frame_data[833] ^= 0x0001;



    send_answer_chunk(channel_mask, var_name, 0);
    send_answer_chunk(channel_mask, "=", 0);

    char buf[16]; memset(buf, 0, sizeof(buf));

    for (uint16_t pix=0; pix < 768; pix++)
    {
      const char *p = my_dtostrf(to_list[pix], 10, 2, buf);
      while (*p == ' ') p++; // remove leading space
      if (pix < (768-1))
      {
        send_answer_chunk(channel_mask, p, 0);
        send_answer_chunk(channel_mask, ",", 0);
      } else
      { // last one...
        send_answer_chunk(channel_mask, p, 1);
      }
    }

    return 0;
  }


  return 1;
}

#ifdef __cplusplus
}
#endif

#endif // DEVICE_MLX90640_TEST_ENABLE
