#ifndef __MLX_I2C_STICK_CMD_H__
#define __MLX_I2C_STICK_CMD_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// supporting functions 
char nibble_to_hex(uint8_t nibble);
void uint8_to_hex(char *hex, uint8_t dec);
void uint16_to_hex(char *hex, uint16_t dec);
void uint32_to_dec(char *str, uint32_t dec, int8_t digits);
int16_t atohex8(const char *in);
int32_t atohex16(const char *in);
const char *bytetohex(uint8_t dec);
const char *bytetostr(uint8_t dec);
char *my_dtostrf(float val,  int8_t char_num, uint8_t precision, char *chr_buffer);

// command functions.

uint8_t cmd_ch(uint8_t channel_mask, const char *input);
uint8_t cmd_ch_write(uint8_t channel_mask, const char *input);

uint8_t cmd_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message);
uint8_t cmd_raw(uint8_t sa, uint16_t *raw_list, uint16_t *raw_count, char const **error_message);
uint8_t cmd_nd(uint8_t sa, uint8_t *nd, char const **error_message);
uint8_t cmd_sn(uint8_t sa, uint16_t *sn_list, uint16_t *sn_count, char const **error_message);
uint8_t cmd_cs(uint8_t sa, uint8_t channel_mask, const char *input);
uint8_t cmd_cs_write(uint8_t sa, uint8_t channel_mask, const char *input);
uint8_t cmd_ee(uint8_t sa, uint16_t *ee_list, uint16_t *ee_count, uint16_t *ee_start_address, char const **error_message);
uint8_t cmd_test(uint8_t sa, uint8_t channel_mask, const char *input, char const **error_message);

uint8_t cmd_ca(uint8_t app_id, uint8_t channel_mask, const char *input);
uint8_t cmd_ca_write(uint8_t app_id, uint8_t channel_mask, const char *input);

uint8_t cmd_tear_down(uint8_t sa);

#ifdef __cplusplus
}
#endif

#endif // __MLX_I2C_STICK_CMD_H_
