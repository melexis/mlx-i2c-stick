#ifndef _PT100_ADS122C_CMD_
#define _PT100_ADS122C_CMD_

#include <SparkFun_ADS122C04_ADC_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_ADS122C0

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PT100_LSB_C 128

struct MyPt100Device_t
{
  uint8_t slave_address_;
  uint8_t wire_mode_;
  unsigned long nd_timer_;
  SFE_ADS122C04 ads122c_;
};

MyPt100Device_t *cmd_pt100_ads122c_get_handle(uint8_t sa);

void cmd_pt100_ads122c_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message);
void cmd_pt100_ads122c_nd(uint8_t sa, uint8_t *nd, char const **error_message);
//void cmd_pt100_ads122c_sn(uint8_t sa, char *answer, char const **error_message);
void cmd_pt100_ads122c_cs(uint8_t sa, uint8_t channel_mask, const char *input);
void cmd_pt100_ads122c_cs_write(uint8_t sa, uint8_t channel_mask, const char *input);

void cmd_pt100_ads122c_tear_down(uint8_t sa);

#ifdef  __cplusplus
}
#endif

#endif
