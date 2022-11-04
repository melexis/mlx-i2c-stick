#ifndef _MLX90632_CMD_
#define _MLX90632_CMD_

#include <stdint.h>
#include "mlx90632_advanced.h"

#ifdef  __cplusplus
extern "C" {
#endif

#define MLX90632_LSB_C 128

void cmd_90632_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message);
void cmd_90632_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message);
void cmd_90632_nd(uint8_t sa, uint8_t *nd, char const **error_message);
void cmd_90632_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message);
void cmd_90632_cs(uint8_t sa, uint8_t channel_mask, const char *input);
void cmd_90632_cs_write(uint8_t sa, uint8_t channel_mask, const char *input);
void cmd_90632_ee(uint8_t sa, uint16_t *ee_data, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message);

void cmd_90632_tear_down(uint8_t sa);

// this function is for internal use only... like in an app
Mlx90632Device *cmd_90632_get_handle(uint8_t sa);

#ifdef  __cplusplus
}
#endif

#endif
