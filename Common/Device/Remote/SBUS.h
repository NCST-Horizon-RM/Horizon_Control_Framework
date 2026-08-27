#ifndef HORIZON_SBUS_H
#define HORIZON_SBUS_H

#include "main.h"
#include "Offline_Detector.h"


typedef enum
{
  SBUS_SUCCESS = 0,
  SBUS_INVALID_PARAM = 1,
  SBUS_DATA_LENGTH_ERROR = 2,
  SBUS_STARTBYTE_ERROR = 3,
  SBUS_ENDBYTE_ERROR = 4
}SBUS_ERROR_CODE_TypeDef;


typedef struct
{
  Offline_Check_t offline;
  uint8_t startbyte;
  int16_t CH[16];
  uint8_t flags;
  uint8_t endbyte;
}SBUS_DATA_typedef;


typedef enum
{
  SBUS_SW_UP=1,
  SBUS_SW_Down,
  SBUS_SW_Cen,
  SBUS_SW_Error,
}SBUS_ChannelState;

typedef enum
{
  SBUS_Channel_1=0,
  SBUS_Channel_2,
  SBUS_Channel_3,
  SBUS_Channel_4,
  SBUS_Channel_5,
  SBUS_Channel_6,
  SBUS_Channel_7,
  SBUS_Channel_8,
  SBUS_Channel_9,
  SBUS_Channel_10,
  SBUS_Channel_11,
  SBUS_Channel_12,
  SBUS_Channel_13,
  SBUS_Channel_14,
  SBUS_Channel_15,
  SBUS_Channel_16,
}SBUS_Channel_t;

SBUS_ERROR_CODE_TypeDef SBUS_decode(uint8_t *raw,  SBUS_DATA_typedef* data,uint8_t len);
int16_t SBUS_GetChannelValue( SBUS_DATA_typedef* data, SBUS_Channel_t Channel);
SBUS_ChannelState SBUS_GetChannelState( SBUS_DATA_typedef* data, SBUS_Channel_t Channel);

#endif