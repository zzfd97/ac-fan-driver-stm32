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
bool are_registers_valid(modbus_register * first_register, uint8_t registers_number);
void send_info_response(modbus_register * first_register, uint8_t registers_number);
void get_info_registers(modbus_register  * data, uint16_t data_length);
void print_buffer(uint8_t * buffer, uint16_t length);


/* GLOBAL VARIABLES */
static modbus_register registers[REGISTERS_NUMBER]; // allocates memory for modbus register data
uint8_t write_request_head[] = {DEVICE_ID, MODBUS_FUNCTION_WRITE};
uint8_t read_request_head[] = {DEVICE_ID, MODBUS_FUNCTION_READ};


modbus_handler modbus_init()
{
	modbus_handler handler;
	handler.registers = &registers;
	return handler;
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

int8_t modbus_process_frame(uint8_t * frame, uint16_t frame_size)
{
//	printf("Received Modbus frame: ");
//	print_buffer(frame, frame_size);
	// check CRC
	uint16_t crc_calculated = crc16_modbus(frame, frame_size-2);
	uint16_t crc_received = get_short_little_endian(frame+frame_size-2);
	if (crc_calculated != crc_received)
	return FRAME_ERROR_CRC;
	
	// read request received
	if (memcmp(frame, read_request_head, sizeof(write_request_head)) == 0)
	{
		printf("Read request received, ");
		uint16_t first_address_offset = get_short_big_endian(frame+2);
		uint16_t registers_number = get_short_big_endian(frame+4);
		printf("first register offset: %d, number of registers: %d\n", first_address_offset, registers_number);
		
		if (first_address_offset + registers_number-1 > MAX_REGISTERS_OFFSET)
		return -1; // index out of range
		
		if ( are_registers_valid(registers + first_address_offset, registers_number) )
		{
			send_info_response(registers + first_address_offset, registers_number);
			return REQUEST_TYPE_READ;
		}
	}
	
	// write request received
	else if ( memcmp(frame, write_request_head, sizeof(write_request_head)) == 0 )
	{
		printf("Write request received, ");
		uint16_t register_offset = get_short_big_endian(frame+2);
		int16_t value_to_set = get_short_big_endian(frame+4);
		printf("setting register with offset %d, value to set: %d\n", register_offset, value_to_set);
		
		if (register_offset > MAX_REGISTERS_OFFSET)
		return FRAME_ERROR_LENGTH;
		
		if ( are_registers_valid(registers + register_offset, 1) )
		{
			(registers + register_offset)->value = value_to_set;
			rs485_transmit_byte_array(frame, frame_size); // send echo as response
			return REQUEST_TYPE_WRITE;
		}
	}

	return -1;
}

void send_info_response(modbus_register * first_register, uint8_t registers_number)
{ 
	uint8_t response_buffer[5 + 2 * REGISTERS_NUMBER]; // max array size is needed when all registers are read back
	uint8_t frame_len = 3 + 2*registers_number + 2;
	
	/* Add constant elements */
	*(response_buffer + 0) = DEVICE_ID;
	*(response_buffer + 1) = MODBUS_FUNCTION_READ;
	*(response_buffer + 2) = 2*registers_number;
	
	/* Add data registers */
	for (int i = 0; i < registers_number; i++)
	{
		*(response_buffer + 3 + 2*i) =  get_high_byte((first_register + i)->value);
		*(response_buffer + 3 + 2*i + 1) =  get_low_byte((first_register + i)->value);
	}

	/* Add CRC */
	uint16_t crc_value = crc16_modbus(response_buffer, frame_len-2);
	*(response_buffer + 3 + 2*registers_number) = get_low_byte(crc_value);
	*(response_buffer + 4 + 2*registers_number) = get_high_byte(crc_value);
	
	printf("Sending Modbus read command response: ");
	print_buffer(response_buffer, frame_len);

	rs485_transmit_byte_array(response_buffer, frame_len);
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


bool are_registers_valid(modbus_register * first_register, uint8_t registers_number)
{
	for (int i = 0; i < registers_number; i++)
	{
		if ( (first_register + i)->active == false ) // found register that is not initialized
			return false;
	}
	return true;
}
