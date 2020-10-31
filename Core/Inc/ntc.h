#include "stm32f4xx_hal.h"

#define ADC_SENSOR_NUMBER 6
#define TEMPERATURE_STATUS_NO_ERROR 0
#define TEMPERATURE_STATUS_ERROR 1

/* TYPE DEFINITIONS */
typedef struct
{
	uint16_t adc_values[ADC_SENSOR_NUMBER];
	int16_t temperatures[ADC_SENSOR_NUMBER];
} sensors_t;

/* reads temperature from all sensors */
void ntc_calculate_temperatures(sensors_t * sensor_values);

/* check if temperatures are in accepted range */
int16_t check_temperatures(sensors_t * sensor_array);
