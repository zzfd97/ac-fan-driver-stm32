// Simple logger for STM32 that uses STM's USB implementation to log data through USB CDC

// 1. USB must be initialized outside of this module, before logging anything
// 2. Default log level is LEVEL_INFO
// 3. Logger messages can be formatted the same way like printf arguments
// 4. This header must be included to any file using logger

// Sample use:
// logger_set_level(LEVEL_ERROR); // set log level first
// log_usb(LEVEL_ERROR, "Error code: %d\n\r", err_code);

#include "stm32f4xx_hal.h"

enum log_level_type {LEVEL_NONE = 0, LEVEL_ERROR = 1, LEVEL_INFO = 2, LEVEL_DEBUG = 3};

void logger_set_level(int8_t level_to_set);

void log_usb(int8_t log_level, char * format, ...);
