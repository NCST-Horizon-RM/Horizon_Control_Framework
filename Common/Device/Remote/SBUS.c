#include "SBUS.h"
#include "Horizon_MATH.h"


SBUS_ERROR_CODE_TypeDef SBUS_decode(uint8_t* raw, SBUS_DATA_typedef* data, uint8_t len)
{
  data->offline.last_feed_tick = HAL_GetTick();
  if (!raw || !data) {
    return 1;
  }

  const uint8_t MIN_DATA_LENGTH = 25;
  if (len < MIN_DATA_LENGTH) {

    return 2;
  }

  data->startbyte = raw[0];

  if (data->startbyte != 0x0F)
  {
    return 3;
  }

  data->CH[0] = (raw[1] | (int16_t)raw[2] << 8) & 0x7FF;
  data->CH[1] = (raw[2] >> 3 | (int16_t)raw[3] << 5) & 0x7FF;
  data->CH[2] = (raw[3] >> 6 | (int16_t)raw[4] << 2 | (int16_t)raw[5] << 10) & 0x7FF;
  data->CH[3] = (raw[5] >> 1 | (int16_t)raw[6] << 7) & 0x7FF;
  data->CH[4] = (raw[6] >> 4 | (int16_t)raw[7] << 4) & 0x7FF;
  data->CH[5] = (raw[7] >> 7 | (int16_t)raw[8] << 1 | (int16_t)raw[9] << 9) & 0x7FF;
  data->CH[6] = (raw[9] >> 2 | (int16_t)raw[10] << 6) & 0x7FF;
  data->CH[7] = (raw[10] >> 5 | (int16_t)raw[11] << 3) & 0x7FF;

  data->CH[8] = (raw[12] | (int16_t)raw[13] << 8) & 0x7FF;
  data->CH[9] = (raw[13] >> 3 | (int16_t)raw[14] << 5) & 0x7FF;
  data->CH[10] = (raw[14] >> 6 | (int16_t)raw[15] << 2 | (int16_t)raw[16] << 10) & 0x7FF;
  data->CH[11] = (raw[16] >> 1 | (int16_t)raw[17] << 7) & 0x7FF;
  data->CH[12] = (raw[17] >> 4 | (int16_t)raw[18] << 4) & 0x7FF;
  data->CH[13] = (raw[18] >> 7 | (int16_t)raw[19] << 1 | (int16_t)raw[20] << 9) & 0x7FF;
  data->CH[14] = (raw[20] >> 2 | (int16_t)raw[21] << 6) & 0x7FF;
  data->CH[15] = (raw[21] >> 5 | (int16_t)raw[22] << 3) & 0x7FF;

  for (uint8_t i = 0; i < 16; i++)
  {
    data->CH[i] -= 1024;
  }

  for (uint8_t i = 0; i < 16; i++)
  {
    if (data->CH[i]<=10 && data->CH[i]>=-10)
    {
      data->CH[i]=0;
    }
  }

  data->flags = raw[23];
  data->endbyte = raw[24];

  if((data->flags)!=0){
    return 5;
  }

  if (data->endbyte != 0x00)
  {
    return 4;
  }

  return 0;
}

int16_t SBUS_GetChannelValue(SBUS_DATA_typedef* data, SBUS_Channel_t Channel)
{
  if (data == NULL || Channel > 16)
  {
    return 0;
  }
  return data->CH[Channel];
}

SBUS_ChannelState SBUS_GetChannelState(SBUS_DATA_typedef* data, SBUS_Channel_t Channel)
{
  if (data == NULL || Channel > 16)
  {
    return SBUS_SW_Error;
  }

  if (data->CH[Channel] > 500)
  {
    return SBUS_SW_Down;
  }

  else if (data->CH[Channel] < -500)
  {
    return SBUS_SW_UP;
  }

  else
  {
    return SBUS_SW_Cen;
  }

}