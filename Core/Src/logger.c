#include "usbd_cdc_if.h"
#include <logger.h>
#include <string.h>

/* PRIVATE METHOD */
void log_usb(char * string_buffer);

enum log_level_type level;

void logger_set_level(int8_t level_to_set)
{
	level = level_to_set;
}

uint8_t get_level()
{
	return (uint8_t)level;
}

void log_error(char * string_buffer)
{
	if (get_level() >= LEVEL_ERROR)
	{
		log_usb(string_buffer);
	}
}

void log_info(char * string_buffer)
{
	if (get_level() >= LEVEL_INFO)
	{
		log_usb(string_buffer);
	}
}

void log_debug(char * string_buffer)
{
	if (get_level() == LEVEL_DEBUG)
	{
		log_usb(string_buffer);
	}
}

void log_usb(char * string_buffer)
{
	const uint16_t max_string_size = 100;
	const int max_retries = 1000;
	const uint8_t line_ending[] = {'\n', '\r'};

	uint8_t string_size = strlen(string_buffer);

	if (string_size > max_string_size)
	{
		string_size = max_string_size;
	}

	uint8_t output_buffer[string_size+2];
	memcpy(output_buffer, string_buffer, string_size);
	memcpy(output_buffer+string_size, line_ending, 2);

	int result = 1;
	int retries = 0;
	while (result != 0 && retries < max_retries)
	{
		result = CDC_Transmit_FS((uint8_t*)output_buffer, sizeof(output_buffer));
		retries++;
	}
}
