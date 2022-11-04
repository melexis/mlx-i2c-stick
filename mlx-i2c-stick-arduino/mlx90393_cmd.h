#ifndef _MLX90393_CMD_
#define _MLX90393_CMD_

#include <stdint.h>
// #include "mlx90393_api.h"

#ifdef  __cplusplus
extern "C" {
#endif

#define MLX90393_LSB_C 32
#define MLX90393_LSB_uT 8

struct MLX90393_t
{
  uint8_t slave_address_;
};

void cmd_90393_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message);
void cmd_90393_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message);
void cmd_90393_nd(uint8_t sa, uint8_t *nd, char const **error_message);
void cmd_90393_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message);
void cmd_90393_cs(uint8_t sa, uint8_t channel_mask, const char *input);
void cmd_90393_cs_write(uint8_t sa, uint8_t channel_mask, const char *input);
void cmd_90393_ee(uint8_t sa, uint16_t *ee_data, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message);

void cmd_90393_tear_down(uint8_t sa);

// this function is for internal use only... like in an app
MLX90393_t *cmd_90393_get_handle(uint8_t sa);

#ifdef  __cplusplus
}
#endif

#endif
