#include "usbd_cdc_if.h"
#include <logger.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

enum log_level_type level = LEVEL_INFO; // this is a default logging level

void logger_set_level(int8_t level_to_set)
{
	level = level_to_set;
}

uint8_t get_level()
{
	return (uint8_t)level;
}

int log_usb(int8_t log_level, char * format, ...)
{
	if (get_level() < log_level)
	{
		return 0;
	}
	uint8_t max_string_len = 100;
	char string_buffer[max_string_len];

	// create string from format and arguments
	va_list argptr;
	va_start(argptr, format);
	vsnprintf(string_buffer, max_string_len, format, argptr);
	va_end(argptr);

	// send data
	const int max_retries = 100;
	int retries = 0;
	while (retries < max_retries)
	{
		if (CDC_Transmit_FS((uint8_t*)string_buffer, strlen(string_buffer)) == 0)
		{
			return 0;
		}
		else
		{
			retries++;
		}
		if (retries >= max_retries)
		{
			return 1;
		}
	}
	return 0;
}
