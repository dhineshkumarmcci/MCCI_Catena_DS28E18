/*

Module:  MCCI_Catena_DS28E18.h

Function:
        namespace McciCatena, class TxBuffer_t;

Copyright notice:
        See accompanying license file.

Author:
        Dhinesh Kumar Pitchai, MCCI Corporation	October 2022

*/

#ifndef _MCCI_CATENA_DS28E18_H_		/* prevent multiple includes */
#define _MCCI_CATENA_DS28E18_H_

/* **** Includes **** */
#include <Arduino.h>
#include <stdbool.h>
#include <OneWire.h>
#include <stdint.h>

/* **** Definitions **** */
#define SPU_Delay_tOP			1 		//ms
#define SUCCESS				1

/// \brief namespace for this library
namespace Mcci_Ds28e18 {

/// \brief instance object for DS28E18
class Ds28e18
    {
public:
    typedef enum {
        STANDARD,       /**< 1-Wire Standard Speed */
        OVERDRIVE,        /**< 1-Wire Overdrive Speed */
        } one_wire_speed;

    typedef enum {
        READ_ROM = 0x33,
        MATCH_ROM = 0x55,
        SEARCH_ROM = 0xF0,
        SKIP_ROM = 0xCC,
        RESUME = 0xA5,
        OVERDRIVE_SKIP = 0x3C,
        OVERDRIVE_MATCH = 0x69,
        RELEASE_BYTE = 0XAA,
        } one_wire_rom_command;

    typedef struct {
        one_wire_speed speed; 	/**< Speed type */
        one_wire_rom_command rom_command;
        unsigned char rom_id[8];
        } One_Wire;

    typedef enum {
        COMMAND_START = 0x66,
        WRITE_SEQUENCER = 0x11,
        READ_SEQUENCER = 0x22,
        RUN_SEQUENCER = 0x33,
        WRITE_CONFIGURATION = 0x55,
        READ_CONFIGURATION = 0x6A,
        WRITE_GPIO_CONFIGURATION = 0x83,
        READ_GPIO_CONFIGURATION = 0x7C,
        DEVICE_STATUS = 0x7A,
        } device_function_commands;

    typedef enum {
        //I2C
        I2C_START = 0x02,
        I2C_STOP = 0x03,
        I2C_WRITE_DATA = 0xE3,
        I2C_READ_DATA = 0xD4,
        I2C_READ_DATA_W_NACK_END = 0xD3,

        // SPI
        SPI_WRITE_READ_BYTE = 0xC0,
        SPI_WRITE_READ_BIT = 0xB0,
        SPI_SS_HIGH = 0x01,
        SPI_SS_LOW = 0x80,

        // Utility
        UTILITY_DELAY = 0xDD,
        UTILITY_SENS_VDD_ON = 0xCC,
        UTILITY_SENS_VDD_OFF = 0xBB,
        UTILITY_GPIO_BUF_WRITE = 0xD1,
        UTILITY_GPIO_BUF_READ = 0x1D,
        UTILITY_GPIO_CNTL_WRITE = 0xE2,
        UTILITY_GPIO_CNTL_READ = 0x2E,
        } sequencer_commands;

    typedef enum {
        POR_OCCURRED = 0x44,
        EXECUTION_ERROR = 0x55,
        INVALID_PARAMETER = 0x77,
        NACK_OCCURED = 0x88,
        PASS = 0xAA,
        } result_byte;

    typedef enum {
        KHZ_100,
        KHZ_400,
        KHZ_1000,
        KHZ_2300,
        } protocol_speed;

    typedef enum {
        DONT_IGNORE,
        IGNORE,
        } ignore_nack;

    typedef enum {
        I2C,
        SPI,
        } protocol;

    typedef enum {
        MODE_0 = 0x00,
        MODE_3 = 0x03,
        } spi_mode;

    typedef enum {
        CONTROL = 0x0B,
        BUFFER = 0x0C,
        } target_configuration_register;

    typedef enum {
        DELAY_1,
        DELAY_2,
        DELAY_4,
        DELAY_8,
        DELAY_16,
        DELAY_32,
        DELAY_64,
        DELAY_128,
        DELAY_256,
        DELAY_512,
        DELAY_1024,
        DELAY_2048,
        DELAY_4096,
        DELAY_8192,
        DELAY_16384,
        DELAY_32768,
        } utility_delay;

    typedef struct {
        unsigned char sequencer_packet[512];
        int sequence_packet_idx;
        unsigned int totalSequencerDelayTime;
        } DS28E18;

    // 1-Wire Error
    int ONE_WIRE_COMMUNICATION_ERROR  = 0xFF;

    // CFG_REG_TARGET Offset
    int GPIO_CTRL_REG = 0x0B;
    int GPIO_BUF_REG = 0x0C;

public:
    Ds28e18();
    Ds28e18(OneWire*);

    // One-Wire functions
    void OneWire_Init(void);
    void OneWire_SetSpeed(one_wire_speed spd);
    one_wire_speed OneWire_GetSpeed();
    void OneWire_SetROM(one_wire_rom_command rom);
    one_wire_rom_command OneWire_GetROM();
    void OneWire_SetRomId(unsigned char *rom_id);
    unsigned char *OneWire_GetRomId();
    int OneWire_Next(unsigned char *romid);
    int OneWire_First(unsigned char *romid);
    
    // device address
    bool getAddress(uint8_t* deviceAddress, uint8_t index);
    bool validAddress(const uint8_t* deviceAddress);

    // General functions
    int run_command(Ds28e18::device_function_commands command, unsigned char *parameters, int parameters_size, int delay_period, unsigned char *result_data);
    unsigned int calculateCrc16Block(unsigned char *data, int dataSize, unsigned int crc);

    // High Level Functions
    int begin();
    int SetSpeed(one_wire_speed spd);
    unsigned char *GetSequencerPacket();
    int GetSequencerPacketSize();
    void ClearSequencerPacket();

    // Device Function Commands
    int WriteSequencer(unsigned short nineBitStartingAddress, unsigned char *txData, int txDataSize);
    int ReadSequencer(unsigned short nineBitStartingAddress, unsigned char *rxData,  unsigned short readLength);
    int RunSequencer(unsigned short nineBitStartingAddress, unsigned short runLength);
    int WriteConfiguration(protocol_speed SPD, ignore_nack INACK, protocol PROT, spi_mode SPI_MODE);
    int ReadConfiguration(unsigned char *rxData);
    int WriteGpioConfiguration(target_configuration_register CFG_REG_TARGET, unsigned char GPIO_HI, unsigned char GPIO_LO);
    int ReadGpioConfiguration(target_configuration_register CFG_REG_TARGET, unsigned char *rxData);
    int DeviceStatus(unsigned char *rxData);

    // Sequencer Commands
    void BuildPacket_I2C_Start();
    void BuildPacket_I2C_Stop();
    void BuildPacket_I2C_WriteData(uint16_t *i2cData, unsigned char i2cDataSize);
    unsigned short BuildPacket_I2C_ReadData(int readBytes);
    unsigned short BuildPacket_I2C_ReadDataWithNackEnd(int readBytes);
    unsigned short BuildPacket_SPI_WriteReadByte(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int readBytes, bool fullDuplex);
    unsigned short BuildPacket_SPI_WriteReadBit(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int writeBits, int readBits);
    void BuildPacket_SPI_SlaveSelectHigh();
    void BuildPacket_SPI_SlaveSelectLow();
    void BuildPacket_Utility_Delay(utility_delay delayTimeInMs);
    void BuildPacket_Utility_SensVddOn();
    void BuildPacket_Utility_SensVddOff();
    void BuildPacket_Utility_GpioBufferWrite(unsigned char GPIO_BUF);
    unsigned short BuildPacket_Utility_GpioBufferRead();
    void BuildPacket_Utility_GpioControlWrite(unsigned char GPIO_CRTL_HI, unsigned char GPIO_CRTL_LO);
    unsigned short BuildPacket_Utility_GpioControlRead();

private:
    // Take a pointer to one wire instance
    OneWire* _wire;
    
    DS28E18 ds28e18;
    One_Wire onewire;
    };

}

/**** end of MCCI_Catena_DS28E18.h ****/
#endif /* _MCCI_CATENA_DS28E18_H_ */