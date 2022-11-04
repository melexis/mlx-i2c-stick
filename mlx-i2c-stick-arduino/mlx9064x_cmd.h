#ifndef _MLX9064x_CMD_
#define _MLX9064x_CMD_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t mlx9064x_is_mlx90641(uint8_t sa);
uint8_t mlx9064x_is_likely_mlx90640(uint8_t sa);
uint8_t mlx9064x_cmd_auto_detect(uint8_t sa);

#ifdef __cplusplus
}
#endif

#endif
