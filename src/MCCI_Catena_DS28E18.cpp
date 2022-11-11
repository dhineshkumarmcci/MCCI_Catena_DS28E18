/*

Module:  MCCI_Catena_DS28E18.cpp

Function:
        namespace McciCatena, class TxBuffer_t;

Copyright notice:
        See accompanying license file.

Author:
        Dhinesh Kumar Pitchai, MCCI Corporation	October 2022

*/

/* **** Includes **** */
#include <stdio.h>
#include <string.h>

#include <MCCI_Catena_DS28E18.h>

using namespace Mcci_Ds28e18;

/* **** Functions **** */

void Ds28e18::OneWire_SetSpeed(Ds28e18::one_wire_speed spd)
{
	this->onewire.speed = spd;
}

Ds28e18::one_wire_speed Ds28e18::OneWire_GetSpeed()
{
	return this->onewire.speed;
}

void Ds28e18::OneWire_SetROM(Ds28e18::one_wire_rom_command rom)
{
	this->onewire.rom_command = rom;
	this->_wire->write((uint8_t)rom);
}

Ds28e18::one_wire_rom_command Ds28e18::OneWire_GetROM()
{
	return this->onewire.rom_command;
}

void Ds28e18::OneWire_SetRomId(unsigned char *rom_id)
{
	for (int i = 0; i < 8; i++)
	{
		this->onewire.rom_id[i] = rom_id[i];
	}
}

unsigned char *Ds28e18::OneWire_GetRomId()
{
	return this->onewire.rom_id;
}

void Ds28e18::OneWire_Init(void)
{
    this->onewire.speed = STANDARD;
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return true  : device found, ROM number in ROM_NO buffer
//        false : device not found, end of search
//
int Ds28e18::OneWire_Next(unsigned char *romid)
{
   // leave the search state alone
   return this->_wire->search(romid);
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return true  : device found, ROM number in ROM_NO buffer
//        false : no device present
//
int Ds28e18::OneWire_First(unsigned char *romid)
{
   // reset the search state
   this->_wire->reset_search();

   return this->_wire->search(romid);
}

/*---------------------------------------------------------------------------*/

Ds28e18::Ds28e18() {}
Ds28e18::Ds28e18(OneWire* _oneWire)
{
        this->_wire = _oneWire;
}

int Ds28e18::begin()
{
        OneWire_Init();

	unsigned char status[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	unsigned char temp_rom_id[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	this->SetSpeed(STANDARD);
	this->_wire->skip();

	printf("\n-- Populate unique ROM ID of all devices on 1-Wire line   --");
	printf("\n-- using Write GPIO Configuration command (ignore result) --");
	this->WriteGpioConfiguration(CONTROL, 0xA5, 0x0F);

	printf("\n\n-- Search and initialize every device found on the 1-Wire line -- ");
	if(OneWire_First(temp_rom_id))
	{
		OneWire_SetRomId(temp_rom_id);
		OneWire_SetROM(MATCH_ROM);

		printf("\n\n-- Write GPIO Configuration so that the voltage on the GPIO ports is known --");
		if(!this->WriteGpioConfiguration(CONTROL, 0xA5, 0x0F))
		{
			return false;
		}

		printf("\n\n-- Read Device Status for valid status information and to clear the POR status bit --");
		if(!this->DeviceStatus(status))
		{
			return false;
		}
		else
		{
	    	printf("\nStatus: \n");
	    	for(int i = 0; i < sizeof(status); i++)
	    	{
	    		printf("[%02X]", status[i]);
	    	}
	    	printf("\n");
		}

		while (this->_wire->search(temp_rom_id))
		{
			OneWire_SetRomId(temp_rom_id);
			memset(status, 0xFF, sizeof(status));

			printf("\n\n-- Write GPIO Configuration so that the voltage on the GPIO ports is known --");
			if(!this->WriteGpioConfiguration(CONTROL, 0xA5, 0x0F))
			{
				return false;
			}

			printf("\n\n-- Read Device Status for valid status information and to clear the POR status bit --");
			if(!this->DeviceStatus(status))
			{
				return false;
			}
			else
			{
		    	printf("\nStatus: \n");
		    	for(int i = 0; i < sizeof(status); i++)
		    	{
		    		printf("[%02X]", status[i]);
		    	}
			}
		}
	}
	else
	{
		return false;
	}

	this->ClearSequencerPacket();

	return true;
}

bool Ds28e18::validAddress(const uint8_t* deviceAddress){
    return (this->_wire->crc8(deviceAddress, 7) == deviceAddress[7]);
}

bool Ds28e18::getAddress(uint8_t* deviceAddress, uint8_t index){
    uint8_t depth = 0;

    this->_wire->reset_search();

    while (depth <= index && this->_wire->search(deviceAddress)) {
        if (depth == index && validAddress(deviceAddress)) return true;
        depth++;
    }

    return false;
}

//-----------------------------------------------------------------------------
//Set desired 1-Wire speed between Standard and Overdrive for both, 1-Wire master and slave.
//Return '0' if at least one device is detected after a 1-Wire reset is performed on new speed.
//Return '1' otherwise
int Ds28e18::SetSpeed(one_wire_speed spd)
{
	printf("\n*Set Device Speed: ");
	int error;

	switch (spd)
	{
		case STANDARD:
			printf("STANDARD*");
			//Set host speed to Standard
			OneWire_SetSpeed(STANDARD);

			// do a 1-Wire reset in Standard and catch presence result
			error = this->_wire->reset();

			break;
		case OVERDRIVE:
			printf("OVERDRIVE*");
			//From Standard speed, do a 1-wire reset + Overdrive Skip ROM to set every device on the line to Overdrive
			this->_wire->reset();
			this->_wire->write((uint8_t)OVERDRIVE_SKIP);
	    	delay(40);

			//Set host speed to Overdrive
	    	OneWire_SetSpeed(OVERDRIVE);

	    	// do a 1-Wire reset in Overdrive and catch presence result
	    	error = this->_wire->reset();

	    	break;
		default:
			error = 1;
	}
	return error;
}

unsigned char *Ds28e18::GetSequencerPacket()
{
	return this->ds28e18.sequencer_packet;
}

int Ds28e18::GetSequencerPacketSize()
{
	return this->ds28e18.sequence_packet_idx;
}

void Ds28e18::ClearSequencerPacket()
{
	memset(this->ds28e18.sequencer_packet, 0x00, sizeof(this->ds28e18.sequencer_packet));
	this->ds28e18.sequence_packet_idx = 0;
}

unsigned int Ds28e18::calculateCrc16Block(unsigned char *data, int dataSize, unsigned int crc)
{
  crc = this->_wire->crc16(data, dataSize, crc);
  return crc;
}

int Ds28e18::run_command(device_function_commands command, unsigned char *parameters, int parameters_size, int delay_period, unsigned char *result_data)
{
	uint8_t tx_packet[3 + parameters_size];
	uint8_t tx_packet_CRC16[2];
	unsigned int expectedCrc = 0;
	uint8_t headerResponse[2];
	int result_data_length;
	uint8_t rx_packet_CRC16[2];

	tx_packet[0] = COMMAND_START;
	tx_packet[1] = 1 + parameters_size;
	tx_packet[2] = command;
	if (parameters_size)
	{
		memcpy(&tx_packet[3], parameters, parameters_size);
	}

	//Reset pulse + presence
	this->_wire->reset();

	//Execute ROM Command currently set
	switch(OneWire_GetROM())
	{
		case READ_ROM:
			printf("\nError: Not appropriate use of Read ROM ");
			return false;
		case MATCH_ROM:
			this->_wire->write((uint8_t)MATCH_ROM);
			this->_wire->write_bytes((uint8_t*)OneWire_GetRomId(), 8);
			break;
		case SEARCH_ROM:
			printf("\nError: Not appropriate use of Search ROM ");
			return false;
		case SKIP_ROM:
			this->_wire->write((uint8_t)SKIP_ROM);
			break;
		case RESUME:
			this->_wire->write((uint8_t)RESUME);
			break;
		case OVERDRIVE_SKIP:
			this->_wire->write((uint8_t)OVERDRIVE_SKIP);
			break;
		case OVERDRIVE_MATCH:
			this->_wire->write((uint8_t)OVERDRIVE_MATCH);
			this->_wire->write_bytes((uint8_t*)OneWire_GetRomId(), 8);
			break;
		default:
			printf("\nError: 1-Wire Communication Error");
			return false;
	}

	//Write command-specific 1-Wire packet, tx_packet
	this->_wire->write_bytes(tx_packet, sizeof(tx_packet));

	//Read CRC16 of the tx_packet
	this->_wire->read_bytes(tx_packet_CRC16, sizeof(tx_packet_CRC16));

	//Verify CRC16
	expectedCrc = calculateCrc16Block(tx_packet, sizeof(tx_packet), expectedCrc);
	expectedCrc ^= 0xFFFFU;
	if (expectedCrc != (unsigned int)((tx_packet_CRC16[1] << 8) | tx_packet_CRC16[0]))
	{
		printf("\nError: Invalid CRC16");
		return false;
	}

	//Send Release Byte, 0xAA
	this->_wire->write((uint8_t)RELEASE_BYTE);

	//Command-specific delay
	delay(delay_period);
	printf("<Delay: %d ms> ", delay_period);

	//Read general command specific 1-Wire packet
	this->_wire->read_bytes(headerResponse, sizeof(headerResponse)); //Dummy Byte + Length Byte;
	result_data_length = headerResponse[1];

	if (result_data_length == 0xFF)
	{
		printf("\nError: 1-Wire Communication Error");
		return false;
	}

	//Read rest of response
	this->_wire->read_bytes(result_data, result_data_length); //Result Byte + Result Data

	//Read CRC16 of the rx_packet
	this->_wire->read_bytes(rx_packet_CRC16, sizeof(rx_packet_CRC16));

	//Verify CRC16
	expectedCrc = 0;
	expectedCrc = calculateCrc16Block(&headerResponse[1], sizeof(headerResponse) - 1, expectedCrc);
	expectedCrc = calculateCrc16Block(result_data, result_data_length, expectedCrc);
	expectedCrc ^= 0xFFFFU;
	if (expectedCrc != (unsigned int)((rx_packet_CRC16[1] << 8) | rx_packet_CRC16[0]))
	{
		printf("\nError: Invalid CRC16");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
//-------- Device Function Commands -----------------------------------------
//---------------------------------------------------------------------------
/// Device Function Command: Write Sequencer (11h)
///
/// @param nineBitStartingAddress Target write address
/// @param txData Array of data to be written into the sequencer memory starting from the target write address
/// @param txDataSize Number of elements found in txData array
/// @return
/// true - command succesful @n
/// false - command failed
///
/// @note Use Sequencer Commands functions to help build txData array.
int Ds28e18::WriteSequencer(unsigned short nineBitStartingAddress, unsigned char *txData, int txDataSize)
{
	printf("\n*Write Sequencer*");

	unsigned char parameters[2 + txDataSize];
	unsigned char response[1];
	unsigned char addressLow;
	unsigned char addressHigh;

	addressLow = nineBitStartingAddress & 0xFF;
	addressHigh = (nineBitStartingAddress >> 8) & 0x01;

	parameters[0] = addressLow;
	parameters[1] = addressHigh;
	memcpy(&parameters[2], &txData[0], txDataSize);

	if (!run_command(WRITE_SEQUENCER, parameters, sizeof(parameters), SPU_Delay_tOP, response))
	{
		return false;
	}

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
/// Device Function Command: Read Sequencer (22h)
///
/// @param nineBitStartingAddress Target read address
/// @param readLength Number of data bytes to be read from the sequencer memory starting from the target read address
/// @param[out] rxData Array of data returned from specified memory address
/// @return
/// true - command succesful @n
/// false - command failed
int Ds28e18::ReadSequencer(unsigned short nineBitStartingAddress, unsigned char *rxData, unsigned short readLength)
{
	printf("\n*Read Sequencer*");

	unsigned char parameters[2];
	int response_length = 1 + readLength;
	unsigned char response[response_length];
	unsigned char addressLow;
	unsigned char addressHigh;

	if (readLength == 128)
	{
		readLength = 0;
	}

	addressLow = nineBitStartingAddress & 0xFF;
	addressHigh = (nineBitStartingAddress >> 8) & 0x01;

	parameters[0] = addressLow;
	parameters[1] = (readLength << 1) | addressHigh;

	if (!run_command(READ_SEQUENCER, parameters, sizeof(parameters), SPU_Delay_tOP, response))
	{
		return false;
	}

	memcpy(rxData, &response[1], response_length - 1);

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
/// Device Function Command: Run Sequencer (33h)
///
/// @param nineBitStartingAddress Target run address
/// @param runLength Number of data bytes to run from the sequencer memory starting from the target run address
/// @return
/// true - command succesful @n
/// false - command failed
int Ds28e18::RunSequencer(unsigned short nineBitStartingAddress, unsigned short runLength)
{
	printf("\n*Run Sequencer*");

	unsigned char parameters[3];
	int response_length = 3;
	unsigned char response[response_length];
	unsigned char addressLow;
	unsigned char addressHigh;
	unsigned char sequencerLengthLow;
	unsigned char sequencerLengthHigh;
	int totalSequencerCommunicationTime = 0;
	int run_sequencer_delay;
	int snackLo;
	int snackHi;
	unsigned short nackOffset;

	if (runLength == 512)
	{
		runLength = 0;
	}

	addressLow = nineBitStartingAddress & 0xFF;
	addressHigh = (nineBitStartingAddress >> 8) & 0x01;
	sequencerLengthLow = ((runLength & 0x7F) << 1);
	sequencerLengthHigh = ((runLength >> 7) & 0x03);

	parameters[0] = addressLow;
	parameters[1] = sequencerLengthLow | addressHigh;
	parameters[2] = sequencerLengthHigh;

	for (float i = 0; i < (runLength / 10); i++)  //add 1ms to Run Sequencer delay for every 10 sequencer commands
	{
		totalSequencerCommunicationTime += 1;
	}

	this->ds28e18.totalSequencerDelayTime += this->ds28e18.totalSequencerDelayTime * 0.05; // Add ~5% to delay option time for assurance

	run_sequencer_delay = SPU_Delay_tOP + this->ds28e18.totalSequencerDelayTime + totalSequencerCommunicationTime;

	if (!run_command(RUN_SEQUENCER, parameters, sizeof(parameters), run_sequencer_delay, response))
	{
		return false;
	}

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case POR_OCCURRED:
		printf("\nError: POR occurred resulting in the command sequencer memory being set to zero");
		return false;

	case EXECUTION_ERROR:
		printf("\nError: Execution Error (Sequencer Command packet or packets incorrectly formed)");
		return false;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	case NACK_OCCURED:
		snackLo = response[1];
		snackHi = response[2];
		nackOffset = snackLo + (snackHi << 8);
		if (nackOffset == 0)
		{
			nackOffset = 512;
		}
		printf("\nError: NACK occurred on address: 0x%02X", nackOffset);

		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
/// Device Function Command: Write Configuration (55h)
///
/// @param SPD Desired protocol speed from macros
/// @param INACK Desired INACK configuration from macros
/// @param PROT Desired protocol from macros
/// @param SPI_MODE Desired SPI Mode from macros
/// @return
/// true - command succesful @n
/// false - command failed
int Ds28e18::WriteConfiguration(protocol_speed SPD, ignore_nack INACK, protocol PROT, spi_mode SPI_MODE)
{
	printf("\n*Write Configuration*");

	unsigned char parameters[1];
	unsigned char response[1];

	parameters[0] = (SPI_MODE << 4) | (PROT << 3) | (INACK << 2) | SPD;

	if (!run_command(WRITE_CONFIGURATION, parameters, sizeof(parameters), SPU_Delay_tOP, response))
	{
		return false;
	}

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
/// Device Function Command: Read Configuration (6Ah)
///
/// @return
/// true - command succesful @n
/// false - command failed
int Ds28e18::ReadConfiguration(unsigned char *rxData)
{
	printf("\n*Read Configuration*");

	unsigned char parameters[0]; //no parameters
	int response_length = 2;
	unsigned char response[response_length];

	if (!run_command(READ_CONFIGURATION, parameters, 0, SPU_Delay_tOP, response))
	{
		return false;
	}

	memcpy(rxData, &response[1], response_length - 1);

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
/// Device Function Command: Write GPIO Configuration (83h)
///
/// @param CFG_REG_TARGET Desired GPIO Configuration Register to write to
/// @param GPIO_HI Control/Buffer register high byte
/// @param GPIO_LO Control/Buffer register low byte
/// @return
/// true - command succesful @n
/// false - command failed
///
/// @note Use GPIO Configuration functions to help build GPIO_HI/GPIO_LO parameter.
int Ds28e18::WriteGpioConfiguration(target_configuration_register CFG_REG_TARGET, unsigned char GPIO_HI, unsigned char GPIO_LO)
{
	printf("\n*Write GPIO Configuration*");

	unsigned char parameters[4];
	unsigned char response[1];

	parameters[0] = CFG_REG_TARGET;
	parameters[1] = 0x03;
	parameters[2] = GPIO_HI;
	parameters[3] = GPIO_LO;

	if (!run_command(WRITE_GPIO_CONFIGURATION, parameters, sizeof(parameters), SPU_Delay_tOP, response))
	{
		return false;
	}

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
/// Device Function Command: Read GPIO Configuration (7Ch)
///
/// @param CFG_REG_TARGET Desired GPIO Configuration Register from macros to read from
/// @param rxData[out] Array of 2 bytes to be updated with device current GPIO configuration for GPIO_HI and GPIO_LO
/// @return
/// true - command succesful @n
/// false - command failed
int Ds28e18::ReadGpioConfiguration(target_configuration_register CFG_REG_TARGET, unsigned char *rxData)
{
	printf("\n*Read GPIO Configuration*");

	unsigned char parameters[2];
	int response_length = 3;
	unsigned char response[response_length];

	parameters[0] = CFG_REG_TARGET;
	parameters[1] = 0x03;

	if (!run_command(READ_GPIO_CONFIGURATION, parameters, sizeof(parameters), SPU_Delay_tOP, response))
	{
		return false;
	}

	memcpy(rxData, &response[1], response_length - 1);

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
/// Device Function Command: Device Status (7Ah)
///
/// @param rxData[out] Array of 4 bytes to be updated with devices' status information
/// @return
/// true - command succesful @n
/// false - command failed
int Ds28e18::DeviceStatus(unsigned char *rxData)
{
	printf("\n*Device Status*");

	unsigned char parameters[0]; //no parameters
	int response_length = 5;
	unsigned char response[response_length];

	if (!run_command(DEVICE_STATUS, parameters, 0, SPU_Delay_tOP, response))
	{
		return false;
	}

	memcpy(rxData, &response[1], response_length - 1);

	// Parse result byte.
	switch (response[0]) {
	case SUCCESS:
		// Success response.
		break;

	case INVALID_PARAMETER:
		printf("\nError: Invalid input or parameter");
		return false;

	default:
		printf("\nError: 1-Wire Communicaton Error");
		return false;
	}

	return true;
}

//---------------------------------------------------------------------------
//-------- Sequencer Commands -----------------------------------------------
//---------------------------------------------------------------------------
/// Sequencer Command: Start (02h).
///
/// Add an I2C Start command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_I2C_Start()
{
	unsigned char i2c_start[1];
	i2c_start[0] = I2C_START;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], i2c_start, sizeof(i2c_start));
	this->ds28e18.sequence_packet_idx += sizeof(i2c_start);
}

//---------------------------------------------------------------------------
/// Sequencer Command: Stop (03h).
///
/// Add an I2C Stop command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_I2C_Stop()
{
	unsigned char i2c_stop[1];
	i2c_stop[0] = I2C_STOP;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], i2c_stop, sizeof(i2c_stop));
	this->ds28e18.sequence_packet_idx += sizeof(i2c_stop);
}

//---------------------------------------------------------------------------
/// Sequencer Command: Write Data (E3h).
///
/// Add an I2C Write Data command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param i2cData Array with data to be transmitted over the I2C bus
/// @param i2cDataSize Number of elements found in i2cData array
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_I2C_WriteData(uint16_t *i2cData, unsigned char i2cDataSize)
{
	unsigned char i2c_write_data[2 + i2cDataSize];
	i2c_write_data[0] = I2C_WRITE_DATA;
	i2c_write_data[1] = i2cDataSize;
	memcpy(&i2c_write_data[2], i2cData, i2cDataSize);

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], i2c_write_data, sizeof(i2c_write_data));
	this->ds28e18.sequence_packet_idx += sizeof(i2c_write_data);
}

//---------------------------------------------------------------------------
/// Sequencer Command: Read Data (D4h).
///
/// Add an I2C Read Data command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param readBytes Number of bytes to read from the I2C bus
/// @return
/// readArrayFFhStartingAddress - Address where I2C slave response will reside
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short Ds28e18::BuildPacket_I2C_ReadData(int readBytes)
{
	unsigned short readArrayFFhStartingAddress = this->ds28e18.sequence_packet_idx + 2;
	unsigned char i2c_read_data[2 + readBytes];

	i2c_read_data[0] = I2C_READ_DATA;
	if (readBytes == 256)
	{
		i2c_read_data[1] = 0;
	}
	else
	{
		i2c_read_data[1] = readBytes;
	}
	memset(&i2c_read_data[2], 0xFF, readBytes);

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], i2c_read_data, sizeof(i2c_read_data));
	this->ds28e18.sequence_packet_idx += sizeof(i2c_read_data);

	return readArrayFFhStartingAddress;
}

//---------------------------------------------------------------------------
/// Sequencer Command: Read Data w/NACK end (D3h).
///
/// Add an I2C Read Data w/NACK end command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param readBytes Number of bytes to read from the I2C bus
/// @return
/// readArrayFFhStartingAddress - Address where I2C slave response will reside
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short Ds28e18::BuildPacket_I2C_ReadDataWithNackEnd(int readBytes)
{
	unsigned short readArrayFFhStartingAddress = this->ds28e18.sequence_packet_idx + 2;
	unsigned char i2c_read_data_with_nack_end[2 + readBytes];

	i2c_read_data_with_nack_end[0] = I2C_READ_DATA_W_NACK_END;
	if (readBytes == 256)
	{
		i2c_read_data_with_nack_end[1] = 0;
	}
	else
	{
		i2c_read_data_with_nack_end[1] = readBytes;
	}
	memset(&i2c_read_data_with_nack_end[2], 0xFF, readBytes);

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], i2c_read_data_with_nack_end, sizeof(i2c_read_data_with_nack_end));
	this->ds28e18.sequence_packet_idx += sizeof(i2c_read_data_with_nack_end);

	return readArrayFFhStartingAddress;
}

//---------------------------------------------------------------------------
/// Sequencer Command: SPI Write/Read Byte (C0h).
///
/// Add a SPI Write/Read Byte command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param spiWriteData Array with data to be transmitted over the SPI bus. Data not important if only reading.
/// @param spiWriteDataSize Number of elements found in spiWriteData array. Set to 0 if only reading.
/// @param readBytes Number of bytes to read from SPI bus. Set to 0 if only writting.
/// @param fullDuplex Set 'true' when interfacing with a full duplex SPI slave. Otherwise, set 'false'
/// @return
/// readArrayFFhStartingAddress - If reading, address where SPI slave response will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short Ds28e18::BuildPacket_SPI_WriteReadByte(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int readBytes, bool fullDuplex)
{
	unsigned short readArrayFFhStartingAddress = 0;
	unsigned char spi_write_read_data_byte[255];
	int idx = 0;

	//command
	spi_write_read_data_byte[idx++] = SPI_WRITE_READ_BYTE;

	if (spiWriteDataSize != 0 && readBytes != 0)
	{
		//Write Length
		spi_write_read_data_byte[idx++] = spiWriteDataSize;

		//Read Length
		spi_write_read_data_byte[idx] = readBytes;
		if (!fullDuplex)
		{
			spi_write_read_data_byte[idx] += spiWriteDataSize;
		}
		idx++;

		//Write Array
		for (int i = 0; i < spiWriteDataSize; i++)
		{
			spi_write_read_data_byte[idx++] = spiWriteData[i];
		}

		//Read Array
		if (!fullDuplex)
		{
			memset(&spi_write_read_data_byte[idx], 0xFF, spiWriteDataSize);
			idx += spiWriteDataSize;
		}
		readArrayFFhStartingAddress = idx;
		memset(&spi_write_read_data_byte[idx], 0xFF, readBytes);
		idx += readBytes;
	}

	else if(spiWriteDataSize != 0 && readBytes == 0)
	{
		//Write Length
		spi_write_read_data_byte[idx++] = spiWriteDataSize;

		//Read Length
		spi_write_read_data_byte[idx++] = 0;

		//Write Array
		for (int i = 0; i < spiWriteDataSize; i++)
		{
			spi_write_read_data_byte[idx++] = spiWriteData[i];
		}

		//Read Array
		//omitted
	}

	else if(spiWriteDataSize == 0 && readBytes != 0)
	{
		//Write Length
		spi_write_read_data_byte[idx++] = 0;

		//Read Length
		spi_write_read_data_byte[idx++] = readBytes;

		//Write Array
		//omitted

		//Read Array
		readArrayFFhStartingAddress = idx;
		memset(&spi_write_read_data_byte[idx], 0xFF, readBytes);
		idx += readBytes;
	}

	else
	{
		//Write Length
		spi_write_read_data_byte[idx++] = 0;

		//Read Length
		spi_write_read_data_byte[idx++] = 0;

		//Write Array
		//omitted

		//Read Array
		//omitted
	}

	readArrayFFhStartingAddress += this->ds28e18.sequence_packet_idx;
	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], spi_write_read_data_byte, idx);
	this->ds28e18.sequence_packet_idx += idx;

	return readArrayFFhStartingAddress;
}

//---------------------------------------------------------------------------
/// Sequencer Command: SPI Write/Read Bit (B0h).
///
/// Add a SPI Write/Read Bit command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param spiWriteData Array with data to be transmitted over the SPI bus. Data not important if only reading.
/// @param spiWriteDataSize Number of elements found in spiWriteData array. Set to 0 if only reading.
/// @param writeBits Number of bits to write to SPI bus. Set to 0 if only reading.
/// @param readBits Number of bits to read from SPI bus. Set to 0 if only writting.
/// @return
/// readArrayFFhStartingAddress - If reading, address where SPI slave response will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short Ds28e18::BuildPacket_SPI_WriteReadBit(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int writeBits, int readBits)
{
	unsigned char readBitsInBytes = 0;
	unsigned short readArrayFFhStartingAddress = 0;
	unsigned char spi_write_read_data_bit[255];
	int idx = 0;

	if (readBits > 0 && readBits < 9)
	{
		readBitsInBytes = 1;
	}
	else if (readBits >= 9 && readBits < 17)
	{
		readBitsInBytes = 2;
	}
	else if (readBits >= 17 && readBits < 25)
	{
		readBitsInBytes = 3;
	}
	else if (readBits >= 25 && readBits < 33)
	{
		readBitsInBytes = 4;
	}
	else if (readBits >= 33 && readBits < 41)
	{
		readBitsInBytes = 5;
	}
	else if (readBits >= 41 && readBits < 49)
	{
		readBitsInBytes = 6;
	}
	else if (readBits >= 49 && readBits < 57)
	{
		readBitsInBytes = 7;
	}
	else if (readBits >= 57 && readBits < 65)
	{
		readBitsInBytes = 8;
	}

	//command
	spi_write_read_data_bit[idx++] = SPI_WRITE_READ_BIT;

	if (writeBits != 0 && readBits != 0)
	{
		//Write Length
		spi_write_read_data_bit[idx++] = writeBits;

		//Read Length
		spi_write_read_data_bit[idx++] = readBits;

		//Write Array
		for (int i = 0; i < spiWriteDataSize; i++)
		{
			spi_write_read_data_bit[idx++] = spiWriteData[i];
		}

		//Read Array
		readArrayFFhStartingAddress = idx;
		memset(&spi_write_read_data_bit[idx], 0xFF, readBitsInBytes);
		idx += readBitsInBytes;
	}

	else if(writeBits != 0 && readBits == 0)
	{
		//Write Length
		spi_write_read_data_bit[idx++] = writeBits;

		//Read Length
		spi_write_read_data_bit[idx++] = 0;

		//Write Array
		for (int i = 0; i < spiWriteDataSize; i++)
		{
			spi_write_read_data_bit[idx++] = spiWriteData[i];
		}

		//Read Array
		//omitted
	}

	else if(writeBits == 0 && readBits != 0)
	{
		//Write Length
		spi_write_read_data_bit[idx++] = 0;

		//Read Length
		spi_write_read_data_bit[idx++] = readBits;

		//Write Array
		//omitted

		//Read Array
		readArrayFFhStartingAddress = idx;
		memset(&spi_write_read_data_bit[idx], 0xFF, readBitsInBytes);
		idx += readBitsInBytes;
	}

	else
	{
		//Write Length
		spi_write_read_data_bit[idx++] = 0;

		//Read Length
		spi_write_read_data_bit[idx++] = 0;

		//Write Array
		//omitted

		//Read Array
		//omitted
	}

	readArrayFFhStartingAddress += this->ds28e18.sequence_packet_idx;
	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], spi_write_read_data_bit, idx);
	this->ds28e18.sequence_packet_idx += idx;

	return readArrayFFhStartingAddress;
}

//---------------------------------------------------------------------------
/// Sequencer Command: SPI SS_High (01h).
///
/// Add a SPI SS_High command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_SPI_SlaveSelectHigh()
{
	unsigned char spi_slave_select_high[1];
	spi_slave_select_high[0] = SPI_SS_HIGH;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], spi_slave_select_high, sizeof(spi_slave_select_high));
	this->ds28e18.sequence_packet_idx += sizeof(spi_slave_select_high);
}

//---------------------------------------------------------------------------
/// Sequencer Command: SPI SS_Low (80h).
///
/// Add a SPI SS_Low command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_SPI_SlaveSelectLow()
{
	unsigned char spi_slave_select_low[1];
	spi_slave_select_low[0] = SPI_SS_LOW;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], spi_slave_select_low, sizeof(spi_slave_select_low));
	this->ds28e18.sequence_packet_idx += sizeof(spi_slave_select_low);
}

//---------------------------------------------------------------------------
/// Sequencer Command: Delay (DDh).
///
/// Add a Delay command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_Utility_Delay(utility_delay delayTimeInMs)
{
	switch (delayTimeInMs)
	{
	case DELAY_1:
		this->ds28e18.totalSequencerDelayTime += 1;
		break;
	case DELAY_2:
		this->ds28e18.totalSequencerDelayTime += 2;
		break;
	case DELAY_4:
		this->ds28e18.totalSequencerDelayTime += 4;
		break;
	case DELAY_8:
		this->ds28e18.totalSequencerDelayTime += 8;
		break;
	case DELAY_16:
		this->ds28e18.totalSequencerDelayTime += 16;
		break;
	case DELAY_32:
		this->ds28e18.totalSequencerDelayTime += 32;
		break;
	case DELAY_64:
		this->ds28e18.totalSequencerDelayTime += 64;
		break;
	case DELAY_128:
		this->ds28e18.totalSequencerDelayTime += 128;
		break;
	case DELAY_256:
		this->ds28e18.totalSequencerDelayTime += 256;
		break;
	case DELAY_512:
		this->ds28e18.totalSequencerDelayTime += 512;
		break;
	case DELAY_1024:
		this->ds28e18.totalSequencerDelayTime += 1024;
		break;
	case DELAY_2048:
		this->ds28e18.totalSequencerDelayTime += 2048;
		break;
	case DELAY_4096:
		this->ds28e18.totalSequencerDelayTime += 4096;
		break;
	case DELAY_8192:
		this->ds28e18.totalSequencerDelayTime += 8192;
		break;
	case DELAY_16384:
		this->ds28e18.totalSequencerDelayTime += 16394;
		break;
	case DELAY_32768:
		this->ds28e18.totalSequencerDelayTime += 32768;
		break;
	}

	unsigned char utility_delay[2];
	utility_delay[0] = UTILITY_DELAY;
	utility_delay[1] = delayTimeInMs;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], utility_delay, sizeof(utility_delay));
	this->ds28e18.sequence_packet_idx += sizeof(utility_delay);
}

//---------------------------------------------------------------------------
/// Sequencer Command: SENS_VDD On (CCh).
///
/// Add a SENS_VDD On command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_Utility_SensVddOn()
{
	unsigned char utility_sens_vdd_on[1];
	utility_sens_vdd_on[0] = UTILITY_SENS_VDD_ON;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], utility_sens_vdd_on, sizeof(utility_sens_vdd_on));
	this->ds28e18.sequence_packet_idx += sizeof(utility_sens_vdd_on);
}

//---------------------------------------------------------------------------
/// Sequencer Command: SENS_VDD Off (BBh).
///
/// Add a SENS_VDD Off command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_Utility_SensVddOff()
{
	unsigned char utility_sens_vdd_off[1];
	utility_sens_vdd_off[0] = UTILITY_SENS_VDD_OFF;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], utility_sens_vdd_off, sizeof(utility_sens_vdd_off));
	this->ds28e18.sequence_packet_idx += sizeof(utility_sens_vdd_off);
}

//---------------------------------------------------------------------------
/// Sequencer Command: GPIO_BUF Write (D1h).
///
/// Add a GPIO_BUF Write command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param GPIO_BUF Buffer register high byte.
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_Utility_GpioBufferWrite(unsigned char GPIO_BUF)
{
	unsigned char utility_gpio_buff_write[2];
	utility_gpio_buff_write[0] = UTILITY_GPIO_BUF_WRITE;
	utility_gpio_buff_write[1] = GPIO_BUF;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], utility_gpio_buff_write, sizeof(utility_gpio_buff_write));
	this->ds28e18.sequence_packet_idx += sizeof(utility_gpio_buff_write);
}

//---------------------------------------------------------------------------
/// Sequencer Command: GPIO_BUF Read (1Dh).
///
/// Add a GPIO_BUF Read command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.

/// @return readArrayFFhStartingAddress - Starting address where configuration data will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short Ds28e18::BuildPacket_Utility_GpioBufferRead()
{
	unsigned short readArrayFFhStartingAddress = this->ds28e18.sequence_packet_idx + 1;
	unsigned char utility_gpio_buff_read[2];
	utility_gpio_buff_read[0] = UTILITY_GPIO_BUF_READ;
	utility_gpio_buff_read[1] = 0xFF; //GPIO_BUF

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], utility_gpio_buff_read, sizeof(utility_gpio_buff_read));
	this->ds28e18.sequence_packet_idx += sizeof(utility_gpio_buff_read);

	return readArrayFFhStartingAddress;
}

//---------------------------------------------------------------------------
/// Sequencer Command: GPIO_CNTL Write (E2h).
///
/// Add a GPIO_CNTL Write command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param GPIO_CRTL_HI Control register high byte.
/// @param GPIO_CRTL_LO Control register low byte.
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void Ds28e18::BuildPacket_Utility_GpioControlWrite(unsigned char GPIO_CRTL_HI, unsigned char GPIO_CRTL_LO)
{
	unsigned char utility_gpio_cntl_write[3];
	utility_gpio_cntl_write[0] = UTILITY_GPIO_CNTL_WRITE;
	utility_gpio_cntl_write[1] = GPIO_CRTL_HI;
	utility_gpio_cntl_write[2] = GPIO_CRTL_LO;

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], utility_gpio_cntl_write, sizeof(utility_gpio_cntl_write));
	this->ds28e18.sequence_packet_idx += sizeof(utility_gpio_cntl_write);
}

//---------------------------------------------------------------------------
/// Sequencer Command: GPIO_CNTL Read (2Eh).
///
/// Add a GPIO_CNTL Read command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.

/// @return readArrayFFhStartingAddress - Starting address where configuration data will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short Ds28e18::BuildPacket_Utility_GpioControlRead()
{
	unsigned short readArrayFFhStartingAddress = this->ds28e18.sequence_packet_idx + 1;
	unsigned char utility_gpio_cntl_read[3];
	utility_gpio_cntl_read[0] = UTILITY_GPIO_CNTL_READ;
	utility_gpio_cntl_read[1] = 0xFF; //GPIO_CTRL_HI
	utility_gpio_cntl_read[2] = 0xFF; //GPIO_CTRL_LO

	memcpy(&this->ds28e18.sequencer_packet[this->ds28e18.sequence_packet_idx], utility_gpio_cntl_read, sizeof(utility_gpio_cntl_read));
	this->ds28e18.sequence_packet_idx += sizeof(utility_gpio_cntl_read);

	return readArrayFFhStartingAddress;
}