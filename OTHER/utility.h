#ifndef __UTILITY_H__
#define __UTILITY_H__

#include "stm32f0xx.h"


uint16_t crc16one(uint8_t input,uint16_t incrc);

uint8_t CrcCheck(uint8_t *buff,uint16_t buffsize,uint8_t* crc);

void BuffReset_API(uint8_t *pdata,uint16_t size);

uint32_t ASCII_TO_INT(uint8_t *pdata,uint16_t NUM);

#endif

