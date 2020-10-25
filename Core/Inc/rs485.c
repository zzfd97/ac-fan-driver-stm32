#include <string.h>
#include <config.h>
#include <rs485.h>
#include <stdbool.h>

/* STATIC FUNCTION PROTOTYPES */
void transmit_byte(uint8_t data);
void transmitter_enable(void);
void transmitter_disable(void);
bool rx_buffer_full(void);

/* GLOBAL VARIABLES */
uint8_t uart_tx_buffer[RS_TX_BUFFER_SIZE];
uint8_t uart_rx_buffer[RS_RX_BUFFER_SIZE];
uint8_t * tx_buffer_pointer = uart_tx_buffer;
uint8_t * rx_buffer_pointer = uart_rx_buffer;
uint16_t bytes_to_transmit = 0;

/* FUNCTION DEFINITIONS */
void rs485_init(void)
{
// TODO
}

void transmit_byte(uint8_t data)
{
// TODO
}

void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size)
{
	transmitter_enable();
	memcpy(uart_tx_buffer, byte_array, array_size);
	bytes_to_transmit = array_size;
	rs485_transmit_from_buffer();
}

void transmitter_enable(void)
{
	HAL_GPIO_WritePin(rs_dir_GPIO_Port, rs_dir_Pin, GPIO_PIN_SET);
}

void transmitter_disable(void)
{
	HAL_GPIO_WritePin(rs_dir_GPIO_Port, rs_dir_Pin, GPIO_PIN_SET);
}

void rs485_transmit_from_buffer(void)
{
	if (bytes_to_transmit > (tx_buffer_pointer-uart_tx_buffer))
	{
		transmit_byte(*tx_buffer_pointer);
		tx_buffer_pointer++;
	}
	else
	{
		transmitter_disable();
		bytes_to_transmit = 0;
		tx_buffer_pointer = uart_tx_buffer;
	}
}

bool rs485_ready_to_send(void)
{
	if (tx_buffer_pointer == uart_tx_buffer)
		return true;
	else
		return false;
}

bool rx_buffer_full(void)
{
	if (rx_buffer_pointer <= uart_rx_buffer + RS_RX_BUFFER_SIZE)
		return false;
	else
		return true;
}

bool rs485_rx_buffer_empty(void)
{
	if (rx_buffer_pointer == uart_rx_buffer)
		return true;
	else
		return false;	
}

bool rs485_get_byte_to_buffer()
{
//	if (!rx_buffer_full())
//	{
//		uint8_t received_byte = usart_get(&USART0);
//		*rx_buffer_pointer = received_byte;
//		rx_buffer_pointer++;
//		return true;
//	}
//	else
//		return false;
}

void rs485_get_frame(uint8_t * dest_array, uint8_t array_size)
{
	memcpy(dest_array, uart_rx_buffer, array_size);
	memset(uart_rx_buffer, 0, RS_TX_BUFFER_SIZE);
	rx_buffer_pointer = uart_rx_buffer;
}
