#include "usbd_cdc_if.h"
#include <logger.h>
#include <string.h>

void log_usb(char * string_buffer)
{
	const uint16_t max_string_size = 100;
	const uint8_t line_ending[] = {'\n', '\r'};

	uint8_t string_size = strlen(string_buffer);

	if (string_size > max_string_size)
	{
		string_size = max_string_size;
	}

	uint8_t output_buffer[string_size+2];
	memcpy(output_buffer, string_buffer, string_size);
	memcpy(output_buffer+string_size, line_ending, 2);

	CDC_Transmit_FS((uint8_t*)output_buffer, sizeof(output_buffer));
}
