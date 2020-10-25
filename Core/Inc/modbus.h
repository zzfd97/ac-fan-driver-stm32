#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifndef MODBUS_H_
#define MODBUS_H_

#define DEVICE_ID 0x02
#define REGISTERS_NUMBER 16
#define REQUEST_TYPE_READ 0
#define REQUEST_TYPE_WRITE 1

/* type definition for modbus registers table, unused registers should have field active = false */
struct register_t
{
	bool active;
	int16_t value;
};

void modbus_init(struct register_t * modbus_registers_pointer);

/* Function to process modbus frame. Returns REQUEST_TYPE_READ or REQUEST_TYPE_WRITE or -1 if frame is not valid*/
int8_t modbus_process_frame(uint8_t * frame, uint16_t frame_size);

#endif /* MODBUS_H_ */
