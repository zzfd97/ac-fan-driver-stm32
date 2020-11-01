#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifndef MODBUS_H_
#define MODBUS_H_

// Adjustable parameters
#define DEVICE_ID 0x02
#define REGISTERS_NUMBER 16

#define REQUEST_TYPE_READ 0
#define REQUEST_TYPE_WRITE 1

typedef int16_t modbus_register;

/* Set value in Modbus register */
bool modbus_set_reg_value(uint16_t offset, int16_t value);

/* Get value from Modbus register */
int16_t modbus_get_reg_value(uint16_t offset);

/* Process Modbus frame and generate response */
bool modbus_process_frame(uint8_t * frame, uint16_t frame_size, uint8_t * response_buffer, uint16_t * response_length);

#endif /* MODBUS_H_ */
