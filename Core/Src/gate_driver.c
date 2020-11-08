/*
 * gate_driver.c
 *  Created on: 06.11.2020
 */

#include <gate_driver.h>


void drive_fans(channel_t * channel_array, uint8_t array_size, uint32_t gate_pulse_delay_counter_us)
{
	for (uint8_t i = 0; i < array_size; i++)
	{
		if (channel_array[i].output_voltage_decpercent < MIN_OUTPUT_VOLTAGE_DECPERCENT)
		{
			set_gate_state(&(channel_array[i]), GATE_IDLE); // full off
		}

		else if (channel_array[i].output_voltage_decpercent >= MAX_OUTPUT_VOLTAGE_DECPERCENT)
		{
			set_gate_state(channel_array + i, GATE_ACTIVE); // full on
		}

		else if ( (gate_pulse_delay_counter_us >= channel_array[i].activation_delay_us) && (gate_pulse_delay_counter_us < (channel_array[i].activation_delay_us + GATE_PULSE_MIN_TIME_US)) )
		{
			set_gate_state(&(channel_array[i]), GATE_ACTIVE);
		}
		else
		{
			set_gate_state(&(channel_array[i]), GATE_IDLE);
		}
	}

//	//FOR DEBUG PURPOSES ONLY : TOGGLING PIN
//	if (channel_array[0].state == GATE_IDLE)
//		set_gate_state(&(channel_array[0]), GATE_ACTIVE);
//	else
//		set_gate_state(&(channel_array[0]), GATE_IDLE);
}


uint32_t get_gate_delay_us(uint16_t output_voltage_percent)
{
	uint16_t mean_voltage = (output_voltage_percent*23)/100;
	double activation_angle_rad = acos(mean_voltage/230.0); // acos function input is double, value from -1 to 1
	uint32_t gate_delay = HALF_SINE_PERIOD_US*activation_angle_rad/(PI/2.0);

	if (gate_delay > MAX_GATE_DELAY_US)
		gate_delay = MAX_GATE_DELAY_US;

	if (gate_delay < MIN_GATE_DELAY_US)
		gate_delay = MIN_GATE_DELAY_US;

	return gate_delay - ZERO_CROSSING_DETECTION_OFFSET_US;
}


void set_gate_state(channel_t * fan, gate_state_t pulse_state)
{
	if (fan->state == pulse_state)
	{
		return; // no state change
	}

	fan->state = pulse_state;

	if (pulse_state == GATE_ACTIVE)
	{
		HAL_GPIO_WritePin(GPIOC, fan->gate_pin, GPIO_PIN_RESET); // optotransistor is active low
	}

	if (pulse_state != GATE_ACTIVE)
	{
		HAL_GPIO_WritePin(GPIOC, fan->gate_pin, GPIO_PIN_SET);
	}
}
