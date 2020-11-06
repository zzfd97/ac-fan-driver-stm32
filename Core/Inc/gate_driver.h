/*
 * gate_driver.h
 *  Created on: 06.11.2020
 */

#ifndef SRC_GATE_DRIVER_H_
#define SRC_GATE_DRIVER_H_

#include <config.h>
#include "stm32f4xx_hal.h"


typedef enum
{
	GATE_IDLE,
	GATE_ACTIVE,
} gate_state_t;

typedef struct
{
	uint32_t gate_pin;
	const uint8_t temp_sensor_index;
	uint8_t work_state;
	int16_t setpoint;
	int16_t output_voltage_decpercent;
	uint32_t activation_delay_us; // time from zero-crossing to gate activation
	gate_state_t state;
} channel_t;


void drive_fans(channel_t * channel_array, uint8_t channel_number, uint32_t gate_pulse_delay_counter_us);

uint32_t get_gate_delay_us(uint16_t output_power);

void set_gate_state(channel_t * fan, gate_state_t pulse_state);


#endif /* SRC_GATE_DRIVER_H_ */
