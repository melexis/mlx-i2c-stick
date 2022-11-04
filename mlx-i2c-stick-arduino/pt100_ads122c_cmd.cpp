#include "pt100_ads122c_cmd.h"
#include "mlx_i2c_stick.h"
#include "mlx_i2c_stick_cmd.h"

#ifdef DEVICE_PT100_ADS122C_ENABLE

#ifdef  __cplusplus
extern "C" {
#endif

#include "pt100_ads122c_cmd.h"


#ifndef MAX_PT100_ADS122C_SLAVES
#define MAX_PT100_ADS122C_SLAVES 8
#endif // MAX_PT100_ADS122C_SLAVES

#define PT100_ADS122C_ERROR_BUFFER_TOO_SMALL "Buffer too small"
#define PT100_ADS122C_ERROR_NO_FREE_HANDLE "No free handle; pls recompile firmware with higher 'MAX_PT100_ADS122C_SLAVES'"

#define PT100_ADS100C_DEFAULT_WIRE_MODE ADS122C04_2WIRE_MODE


static MyPt100Device_t *g_pt100_list[MAX_PT100_ADS122C_SLAVES];


MyPt100Device_t *
cmd_pt100_ads122c_get_handle(uint8_t sa)
{
  if (sa >= 128)
  {
    return NULL;
  }

  for (uint8_t i=0; i<MAX_PT100_ADS122C_SLAVES; i++)
  {
    if (!g_pt100_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_pt100_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      return g_pt100_list[i];
    }
  }

  // not found => try to find a handle with slave address zero (not yet initialized)!
  for (uint8_t i=0; i<MAX_PT100_ADS122C_SLAVES; i++)
  {
    if (!g_pt100_list[i])
    {
      continue; // allow empty spots!
    }
    if (g_pt100_list[i]->slave_address_ == 0)
    { // found!
      return g_pt100_list[i];
    }
  }

  // not found => use first free spot!
  uint8_t i=0;
  for (; i<MAX_PT100_ADS122C_SLAVES; i++)
  {
    if (g_pt100_list[i] == NULL)
    {
      g_pt100_list[i] = (MyPt100Device_t *)malloc(sizeof(MyPt100Device_t));;
      memset(g_pt100_list[i], 0, sizeof(MyPt100Device_t));
      g_pt100_list[i]->slave_address_ = 0x80 | sa;
      return g_pt100_list[i];
    }
  }

  return NULL; // no free spot available
}


static void
delete_handle(uint8_t sa)
{
  for (uint8_t i=0; i<MAX_PT100_ADS122C_SLAVES; i++)
  {
    if (!g_pt100_list[i])
    {
      continue; // allow empty spots!
    }
    if ((g_pt100_list[i]->slave_address_ & 0x7F) == sa)
    { // found!
      memset(g_pt100_list[i], 0, sizeof(MyPt100Device_t));
      free(g_pt100_list[i]);
      g_pt100_list[i] = NULL;
    }
  }
}



void
cmd_pt100_ads122c_mv(uint8_t sa, float *mv_list, uint16_t *mv_count, char const **error_message)
{
  MyPt100Device_t *pt100 = cmd_pt100_ads122c_get_handle(sa);
  if (pt100 == NULL)
  {
    *mv_count = 0;
    *error_message = PT100_ADS122C_ERROR_NO_FREE_HANDLE;
    return;
  }

  if (*mv_count <= 2)
  {
    *mv_count = 0;
    *error_message = PT100_ADS122C_ERROR_BUFFER_TOO_SMALL;
    return;
  }
  *mv_count = 2;

  if (pt100->slave_address_ == 0) // only the case of a new handle!
  {
    pt100->ads122c_.begin(sa, WIRE);
    pt100->slave_address_ = sa;
    pt100->ads122c_.configureADCmode(PT100_ADS100C_DEFAULT_WIRE_MODE); 
  }

  mv_list[0] = pt100->ads122c_.readInternalTemperature();
  mv_list[1] = pt100->ads122c_.readPT100Centigrade();
}


void
cmd_pt100_ads122c_nd(uint8_t sa, uint8_t *nd, char const **error_message)
{ // nd = new data
  MyPt100Device_t *pt100 = cmd_pt100_ads122c_get_handle(sa);

  if (pt100 == NULL)
  {
    *error_message = PT100_ADS122C_ERROR_NO_FREE_HANDLE;
    return;
  }

  if (pt100->slave_address_ == 0) // only the case of a new handle!
  {
    pt100->ads122c_.begin(sa, WIRE);
    pt100->slave_address_ = sa;
    pt100->ads122c_.configureADCmode(PT100_ADS100C_DEFAULT_WIRE_MODE); 
  }

  if (pt100->nd_timer_ == 0)
  { // first time
    pt100->nd_timer_ = millis();
    *nd = 1; // first time there is new data
    return;
  }
  *nd = 0;
  if ((millis() - pt100->nd_timer_) > 1000)
  {
    pt100->nd_timer_ = millis();
    *nd = 1;
  }
}


void
cmd_pt100_ads122c_cs(uint8_t sa, uint8_t channel_mask, const char *input)
{
  MyPt100Device_t *pt100 = cmd_pt100_ads122c_get_handle(sa);

  if (pt100 == NULL)
  { // failed to get handle!
    return;
  }

  if (pt100->slave_address_ == 0) // only the case of a new handle!
  {
    pt100->ads122c_.begin(sa, WIRE);
    pt100->slave_address_ = sa;
    pt100->ads122c_.configureADCmode(PT100_ADS100C_DEFAULT_WIRE_MODE); 
  }

  const char *p = NULL;
  switch(pt100->wire_mode_)
  {
    case ADS122C04_4WIRE_MODE:
      p = ":MODE=4WIRE_MODE";
      break;
    case ADS122C04_3WIRE_MODE:
      p = ":MODE=3WIRE_MODE";
      break;
    case ADS122C04_2WIRE_MODE:
      p = ":MODE=2WIRE_MODE";
      break;
    case ADS122C04_TEMPERATURE_MODE:
      p = ":MODE=TEMPERATURE_MODE";
      break;
    case ADS122C04_RAW_MODE:
      p = ":MODE=RAW_MODE";
      break;
    case ADS122C04_4WIRE_HI_TEMP:
      p = ":MODE=4WIRE_HI_TEMP";
      break;
    case ADS122C04_3WIRE_HI_TEMP:
      p = ":MODE=3WIRE_HI_TEMP";
      break;
    case ADS122C04_2WIRE_HI_TEMP:
      p = ":MODE=2WIRE_HI_TEMP";
      break;
    default:
      break;
  }

  if (p)
  {
    char buf[16]; memset(buf, 0, sizeof(buf));
    send_answer_chunk(channel_mask, "cs:", 0);
    uint8_to_hex(buf, sa);
    send_answer_chunk(channel_mask, buf, 0);
    send_answer_chunk(channel_mask, p, 1);
  }

  char buf[16]; memset(buf, 0, sizeof(buf));
  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_HEADER=T_sensor,T_pt100", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_UNIT=DegC,DegC", 1);

  send_answer_chunk(channel_mask, "cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);
  send_answer_chunk(channel_mask, ":RO:MV_RES=" xstr(PT100_LSB_C) "," xstr(PT100_LSB_C), 1);
}


void
cmd_pt100_ads122c_cs_write(uint8_t sa, uint8_t channel_mask, const char *input)
{
  char buf[16]; memset(buf, 0, sizeof(buf));
  MyPt100Device_t *pt100 = cmd_pt100_ads122c_get_handle(sa);

  if (pt100 == NULL)
  { // failed to get handle!
    return;
  }

  if (pt100->slave_address_ == 0) // only the case of a new handle!
  {
    pt100->ads122c_.begin(sa, WIRE);
    pt100->slave_address_ = sa;
    pt100->ads122c_.configureADCmode(PT100_ADS100C_DEFAULT_WIRE_MODE); 
  }

  send_answer_chunk(channel_mask, "+cs:", 0);
  uint8_to_hex(buf, sa);
  send_answer_chunk(channel_mask, buf, 0);

  const char *var_name = "MODE=";
  if (!strncmp(var_name, input, strlen(var_name)))
  {
    int16_t mode = 9999;
    if (!strcmp(input+strlen(var_name),"4WIRE_MODE"))       mode = ADS122C04_4WIRE_MODE;
    if (!strcmp(input+strlen(var_name),"3WIRE_MODE"))       mode = ADS122C04_3WIRE_MODE;
    if (!strcmp(input+strlen(var_name),"2WIRE_MODE"))       mode = ADS122C04_2WIRE_MODE;
    if (!strcmp(input+strlen(var_name),"TEMPERATURE_MODE")) mode = ADS122C04_TEMPERATURE_MODE;
    if (!strcmp(input+strlen(var_name),"RAW_MODE"))         mode = ADS122C04_RAW_MODE;
    if (!strcmp(input+strlen(var_name),"4WIRE_HI_TEMP"))    mode = ADS122C04_4WIRE_HI_TEMP;
    if (!strcmp(input+strlen(var_name),"3WIRE_HI_TEMP"))    mode = ADS122C04_3WIRE_HI_TEMP;
    if (!strcmp(input+strlen(var_name),"2WIRE_HI_TEMP"))    mode = ADS122C04_2WIRE_HI_TEMP;
    if ((mode >= 0) && (mode <= 7))
    {
      pt100->ads122c_.configureADCmode(mode);   
      pt100->wire_mode_ = mode;
      send_answer_chunk(channel_mask, ":MODE=OK: [ADS112C-register]", 1);
    } else
    {
      send_answer_chunk(channel_mask, ":FAIL: unknown mode value", 1);
    }
    return;
  }
}


void
cmd_pt100_ads122c_tear_down(uint8_t sa)
{
  delete_handle(sa);
}


#ifdef  __cplusplus
}
#endif

#endif //DEVICE_PT100_ADS122C_ENABLE
