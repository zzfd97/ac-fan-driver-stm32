#include <string.h>
#include <crc.h>
#include <modbus.h>
#include <rs485.h>

#define MAX_REGISTERS_OFFSET (REGISTERS_NUMBER-1)
#define MODBUS_FUNCTION_READ 0x03
#define MODBUS_FUNCTION_WRITE 0x06
#define FRAME_ERROR_LENGTH -1
#define FRAME_ERROR_CRC -2

/* STATIC FUNCTION DECLARATIONS */
uint8_t get_high_byte(uint16_t two_byte);
uint8_t get_low_byte(uint16_t two_byte);
uint16_t get_short_little_endian(uint8_t * first_byte_pointer);
uint16_t get_short_big_endian(uint8_t * first_byte_pointer);
void get_info_registers(modbus_register  * data, uint16_t data_length);
void print_buffer(uint8_t * buffer, uint16_t length);


/* GLOBAL VARIABLES */
static int16_t registers[REGISTERS_NUMBER]; // allocates memory for modbus register data
uint8_t write_request_head[] = {DEVICE_ID, MODBUS_FUNCTION_WRITE};
uint8_t read_request_head[] = {DEVICE_ID, MODBUS_FUNCTION_READ};

bool modbus_set_reg_value(uint16_t offset, int16_t value)
{
	if (offset > REGISTERS_NUMBER -1)
	{
		return false;
	}

	registers[offset] = value;
	return true;
}

int16_t modbus_get_reg_value(uint16_t offset)
{
	if (offset > REGISTERS_NUMBER -1)
	{
		// error handling
	}

	return registers[offset];
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

bool modbus_process_frame(uint8_t * request, uint16_t request_size, uint8_t * response, uint16_t * response_size)
{
//	printf("Received Modbus frame: ");
//	print_buffer(frame, frame_size);

	// check CRC
	uint16_t crc_calculated = crc16_modbus(request, request_size-2);
	uint16_t crc_received = get_short_little_endian(request+request_size-2);
	if (crc_calculated != crc_received)
	{
		printf("ERROR: modbus_process_frame, CRC does not match ");
		return false;
	}
	
	// read request received
	switch (request[1])
	{
		case MODBUS_FUNCTION_READ:
			printf("Read request received, ");
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
			*(response + 0) = DEVICE_ID;
			*(response + 1) = MODBUS_FUNCTION_READ;
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
	
	// write request received
	case MODBUS_FUNCTION_WRITE:
		printf("Write request received, ");
		uint16_t register_offset = get_short_big_endian(request+2);
		int16_t value_to_set = get_short_big_endian(request+4);
		printf("setting register with offset %d, value to set: %d\n", register_offset, value_to_set);
		
		if (register_offset > MAX_REGISTERS_OFFSET)
		return FRAME_ERROR_LENGTH;
		
		if (register_offset >= REGISTERS_NUMBER)
		{
			printf("ERROR: modbus_process_frame, requested registers not valid\n");
			return false;
		}
		registers[register_offset] = value_to_set;
		// response for that command is echo
		memcpy(response, request, request_size);
		response_size = request_size;
		break;
	}
	return true;
}


void get_info_registers(modbus_register  * data, uint16_t data_length)
{
	memcpy(registers, data, data_length);
}

void print_buffer(uint8_t * buffer, uint16_t length)
{
	for (int index = 0; index < length; index++)
	{
		printf("0x%02x | ", *(buffer + index));
	}
	printf("\n");
}

