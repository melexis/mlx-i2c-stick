#ifndef _MLX90614_CMD_
#define _MLX90614_CMD_

#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

#define MLX90614_LSB_C 50

struct MLX90614_t
{
  uint8_t slave_address_;
  unsigned long nd_timer_;
};

void cmd_90614_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message);
void cmd_90614_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message);
void cmd_90614_nd(uint8_t sa, uint8_t *nd, char const **error_message);
void cmd_90614_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message);
void cmd_90614_cs(uint8_t sa, uint8_t channel_mask, const char *input);
void cmd_90614_cs_write(uint8_t sa, uint8_t channel_mask, const char *input);
void cmd_90614_ee(uint8_t sa, uint16_t *ee_data, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message);

void cmd_90614_tear_down(uint8_t sa);


#ifdef  __cplusplus
}
#endif

#endif
