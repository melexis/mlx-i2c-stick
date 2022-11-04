#include "mlx90641_api.h"
#include "mlx90640_api.h"
#include <stdint.h>
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif



uint8_t mlx9064x_is_mlx90641(uint8_t sa)
{ // this is an engineering function only; incase of bad hamming code, the routine does not detect the slave as being a mlx90641!
  // do not use this routine in an end-product!
  uint16_t ee_data[832];
  paramsMLX90641 params;

  MLX90641_DumpEE(sa, ee_data);
  if (MLX90641_ExtractParameters(ee_data, &params) == 0)
  { // hamming code is correct, so we can be pretty sure it is a mlx90641.
    return 41;
  }
  return 0;
}


uint8_t mlx9064x_is_likely_mlx90640(uint8_t sa)
{ // this is an engineering function only; in case of corrupt eeprom values this routine will not detect the slave as a mlx90640!
  // do not use this routine in an end-product!

  uint16_t frame_data[834];
  uint16_t *ee_data = frame_data;
  paramsMLX90640 params;

  MLX90640_DumpEE(sa, ee_data);
  MLX90640_ExtractParameters(ee_data, &params);
  // Note: ee_data is not longer used, now the buffer memory is used for frame_data

  // no hamming code here, so let's check if VDD & TA are in a likely range.
  uint8_t sub_pages_seen = 0;
  for (uint8_t i=0; i<10; i++)// try to read 10 frames
  {
    if (MLX90640_GetFrameData(sa, frame_data) < 0)
    {
      continue; // failed to read the frame..., maybe no mlx90640, but let's give it a retry...
    }
    if (frame_data[833] == 0)
    {
      sub_pages_seen |= 0x01;
    }
    if (frame_data[833] == 1)
    {
      sub_pages_seen |= 0x02;
    }

    if (sub_pages_seen == 0x03) break; // quit when both subpages have been seen.
  }
  if (sub_pages_seen == 0x03)
  {
    float vdd = MLX90640_GetVdd(frame_data, &params);
    if ((2.0 < vdd) && (vdd < 4.0))
    {
      // vdd is within this range, so likely it is a mlx90640...
      return 40;
    }
  }
  return 0;
}


uint8_t mlx9064x_cmd_auto_detect(uint8_t sa)
{
  // routine for engineering purposes only.
  // do not use this routine in an end-product!
  if (mlx9064x_is_mlx90641(sa))
  {
    g_device_list[sa] &= 0xF0; 
    g_device_list[sa] |= DEVICE_90641;
    return 41;
  }
  if (mlx9064x_is_likely_mlx90640(sa))
  {
    g_device_list[sa] &= 0xF0; 
    g_device_list[sa] |= DEVICE_90640;
    return 40;
  }
  return 0;  
}


#ifdef __cplusplus
}
#endif
