#include <string.h>
#include <config.h>
#include <rs485.h>
#include <stdbool.h>
#include <logger.h>

/* STATIC FUNCTION PROTOTYPES */
void transmitter_enable(void);
void transmitter_disable(void);

/* GLOBAL VARIABLES */
uint8_t uart_tx_buffer[RS_TX_BUFFER_SIZE];
uint8_t uart_rx_buffer[RS_RX_BUFFER_SIZE];
uint8_t * tx_buffer_pointer = uart_tx_buffer;
uint8_t * rx_buffer_pointer = uart_rx_buffer;
uint16_t bytes_to_transmit = 0;
UART_HandleTypeDef * uart_handler;

/* FUNCTION DEFINITIONS */
void rs485_init(UART_HandleTypeDef * uart_handler_ptr)
{
	uart_handler = uart_handler_ptr;
}


bool rs485_collect_byte_to_buffer(uint8_t * byte)
{
	if (!rs485_rx_buffer_full())
	{
		*rx_buffer_pointer = *byte;
		rx_buffer_pointer++;
		return true;
	}
	else
		return false;
}


void rs485_get_frame(uint8_t * where_to_write, uint8_t array_size)
{
	memcpy(where_to_write, uart_rx_buffer, array_size);
	memset(uart_rx_buffer, 0, RS_RX_BUFFER_SIZE);
	rx_buffer_pointer = uart_rx_buffer;
}


void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size)
{
	transmitter_enable();

	// TODO what should be timeout here?
	HAL_StatusTypeDef status = HAL_UART_Transmit(uart_handler, byte_array, array_size, 100); // must be blocking, as array is passed by pointer!
	if (status != HAL_OK)
	{
		log_usb(LEVEL_ERROR, "ERR: Cannot send buffer");
	}

	transmitter_disable(); // disable DIR pin after transmission is finished
}


bool rs485_rx_buffer_empty(void)
{
	if (rx_buffer_pointer == uart_rx_buffer)
		return true;
	else
		return false;
}


bool rs485_rx_buffer_full(void)
{
	if (rx_buffer_pointer <= uart_rx_buffer + RS_RX_BUFFER_SIZE)
		return false;
	else
		return true;
}


void transmitter_enable(void)
{
	HAL_GPIO_WritePin(rs_dir_GPIO_Port, rs_dir_Pin, GPIO_PIN_SET);
}

void transmitter_disable(void)
{
	HAL_GPIO_WritePin(rs_dir_GPIO_Port, rs_dir_Pin, GPIO_PIN_RESET);
}

