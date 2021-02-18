#include "usbd_cdc_if.h"
#include <logger.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

enum log_level_type level;

void logger_set_level(int8_t level_to_set)
{
	level = level_to_set;
}

uint8_t get_level()
{
	return (uint8_t)level;
}

void log_usb(int8_t log_level, char * format, ...)
{
	if (get_level() < log_level)
	{
		return;
	}
	uint8_t max_string_len = 100;
	char string_buffer[max_string_len];

	// create string from format and arguments
	va_list argptr;
	va_start(argptr, format);
	vsnprintf(string_buffer, max_string_len, format, argptr);
	va_end(argptr);

	// send data
	const int max_retries = 1000;
	int transmit_result = 1;
	int retries = 0;
	while (transmit_result != 0 && retries < max_retries)
	{
		transmit_result = CDC_Transmit_FS((uint8_t*)string_buffer, strlen(string_buffer));
		retries++;
	}
}
