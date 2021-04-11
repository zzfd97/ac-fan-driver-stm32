#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "main.h"

#ifndef RS485_H_
#define RS485_H_

void rs485_init(UART_HandleTypeDef * uart_handler_ptr);
bool rs485_collect_byte_to_buffer(uint8_t * byte);
void rs485_get_frame(uint8_t * dest_array, uint8_t array_size);
void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size);
bool rs485_rx_buffer_empty(void);
bool rs485_rx_buffer_full(void);

#endif /* RS485_H_ */
