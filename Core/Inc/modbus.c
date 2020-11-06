#include <string.h>
#include <modbus.h>
#include <crc.h>

/* LOCAL DEFINES */
#define MAX_REGISTERS_OFFSET (REGISTERS_NUMBER-1)
#define FUNCTION_READ_MULTIPLE 0x03
#define FUNCTION_WRITE_SINGLE 0x06
#define POSITION_DEVICE_ID 0
#define POSITION_FUNCTION 1

/* STATIC FUNCTION DECLARATIONS */
uint8_t get_high_byte(uint16_t two_byte);
uint8_t get_low_byte(uint16_t two_byte);
uint16_t get_short_little_endian(uint8_t * first_byte_pointer);
uint16_t get_short_big_endian(uint8_t * first_byte_pointer);
void print_buffer(uint8_t * buffer, uint16_t length);

/* GLOBAL VARIABLES */
static int16_t registers[REGISTERS_NUMBER]; // allocates memory for modbus register data


int16_t modbus_get_reg_value(uint16_t offset)
{
	if (offset > MAX_REGISTERS_OFFSET)
	{
		// error handling
	}

	return registers[offset];
}


bool modbus_set_reg_value(uint16_t offset, int16_t value)
{
	if (offset > MAX_REGISTERS_OFFSET)
	{
		return false;
	}

	registers[offset] = value;
	return true;
}


bool modbus_process_frame(uint8_t * request, uint16_t request_size, uint8_t * response, uint16_t * response_size)
{
	// check CRC
	uint16_t crc_calculated = crc16_modbus(request, request_size-2);
	uint16_t crc_received = get_short_little_endian(request+request_size-2);
	if (crc_calculated != crc_received)
	{
		printf("ERROR: modbus_process_frame, CRC does not match ");
		return false;
	}
	
	switch (request[POSITION_FUNCTION])
	{
		case FUNCTION_READ_MULTIPLE:
			printf("Request 0x%02x received, ", FUNCTION_READ_MULTIPLE);
			uint16_t first_address_offset = get_short_big_endian(request+2);
			uint16_t registers_number = get_short_big_endian(request+4);
			printf("first register offset: %d, number of registers: %d\n", first_address_offset, registers_number);

			if ( (first_address_offset >= REGISTERS_NUMBER) || (registers_number > REGISTERS_NUMBER) )
			{
				printf("ERROR: modbus_process_frame, requested registers not valid\n");
				return false;
			}

			*response_size = 3 + 2*registers_number + 2;

			/* Add constant elements */
			*(response + POSITION_DEVICE_ID) = DEVICE_ID;
			*(response + POSITION_FUNCTION) = FUNCTION_READ_MULTIPLE;
			*(response + 2) = 2*registers_number;

			/* Add data registers */
			for (int i = 0; i < registers_number; i++)
			{
				*(response + 3 + 2*i) =  get_high_byte(registers[first_address_offset+i]);
				*(response + 3 + 2*i + 1) =  get_low_byte(registers[first_address_offset+i]);
			}

			/* Add CRC */
			uint16_t crc_value = crc16_modbus(response, (*response_size)-2);
			*(response + 3 + 2*registers_number) = get_low_byte(crc_value);
			*(response + 4 + 2*registers_number) = get_high_byte(crc_value);

			break;
	
		case FUNCTION_WRITE_SINGLE:
			printf("Request 0x%02x received, ", FUNCTION_WRITE_SINGLE);
			uint16_t register_offset = get_short_big_endian(request+2);
			int16_t value_to_set = get_short_big_endian(request+4);
			printf("setting register with offset %d, value to set: %d\n", register_offset, value_to_set);

			if (register_offset > MAX_REGISTERS_OFFSET)
			{
				return false;
			}

			if (register_offset >= REGISTERS_NUMBER)
			{
				printf("ERROR: modbus_process_frame, requested registers not valid\n");
				return false;
			}

			registers[register_offset] = value_to_set;
			memcpy(response, request, request_size); // response for that command is echo
			*response_size = request_size;
			break;

		default:
			printf("ERROR: modbus_process_frame, unsupported function detected\n");
			return false;
	}
	return true;
}


uint8_t get_high_byte(uint16_t two_byte)
{
	return ((two_byte >> 8) & 0xFF); // MSB
}


uint8_t get_low_byte(uint16_t two_byte)
{
	return (two_byte & 0xFF); // LSB
}


uint16_t get_short_little_endian(uint8_t * first_byte_pointer) // first byte is low byte
{
	return (short) (*(first_byte_pointer+1) << 8 | *(first_byte_pointer));
}


uint16_t get_short_big_endian(uint8_t * first_byte_pointer) // first byte is high byte
{
	return (short) (*first_byte_pointer << 8 | *(first_byte_pointer+1));
}


void print_buffer(uint8_t * buffer, uint16_t length)
{
	for (int index = 0; index < length; index++)
	{
		printf("0x%02x | ", *(buffer + index));
	}
	printf("\n");
}
