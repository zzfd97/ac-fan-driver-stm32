#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <config.h>

/* TYPE DEFINITIONS */
typedef struct
{
	uint16_t adc_values[ADC_SENSOR_NUMBER];
	int16_t temperatures[ADC_SENSOR_NUMBER];
	bool connected_status[ADC_SENSOR_NUMBER]; // true for connected, false for not connected
} sensors_t;

/* calculates temperature from adc value */
int16_t ntc_to_temperature(uint16_t adc_value);

/* calculates temperature from adc value */
int16_t pt100_to_temperature(uint16_t adc_value);

/**
 * Check if temperatures are in accepted range
 * @param offset Number representing offset of given register (for 40001 use 0)
 * @return value: 0 if no error, or binary representation of channels where error is detected
 * for example 0b000001 (dec 1) indicates error on sensor with index '0', 0b000100 (dec 4) indicates error on sensor with index '2'
 */
int16_t check_for_error(sensors_t * sensor_array);
