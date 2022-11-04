#ifndef _MLX90640_TEST_
#define _MLX90640_TEST_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


void cmd_90640_test(uint8_t sa, uint8_t channel_mask, const char *input);
int8_t cmd_90640_test_float(uint8_t sa, uint8_t channel_mask, const char *input);

#ifdef __cplusplus
}
#endif

#endif
