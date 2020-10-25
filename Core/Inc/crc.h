#include "stm32f4xx_hal.h"

#ifndef _MB_CRC_H
#define _MB_CRC_H

uint16_t crc16_modbus(uint8_t * pucFrame, uint16_t usLen);

#endif
