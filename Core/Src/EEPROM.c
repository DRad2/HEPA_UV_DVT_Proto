#include "EEPROM.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c2;

/* Define i2C that will be used with EEPROM */
#define EEPROM_I2C &hi2c2

/* EEPROM Address */
/* Address = 7 bits
 * The 8th bit is the Read/Write bit (RW). This bit is set to 1 for read and 0 for write operations */
#define EEPROM_ADDR 0x50 << 1
#define EEPROM_ID_PAGE_ADDR 0x58 << 1;

/* Define the Page Size and number of pages */
#define PAGE_SIZE 64     // in Bytes
#define PAGE_NUM  250    // number of pages


uint8_t line[] = "\r\n";

// function to determine the remaining bytes
uint16_t bytestowrite (uint16_t size, uint16_t offset)
{
	if ((size+offset)<PAGE_SIZE) return size;	//if size of data to be sent can fit on the page, transmit all data at once
	else return PAGE_SIZE-offset;				//if size of data can't fit on a single page, transmit enough data to fill one page
}

/* Write EEPROM */
void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
// Find out the number of bit, where the page addressing starts
// Page size = 64
// log(64)/log(2) = 6 => 6 last bits of the two address bytes are reserved for byte address (0-63 range, 63 = 111111)
// The page addressing starts at bit A6
// We use 8 next bits (A6 to A13) for page bytes (0-249 range, 249 = 1111 1001)
int paddrposition = log(PAGE_SIZE)/log(2);

// calculate the start page and the end page
uint16_t startPage = page;
uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

// number of pages to be written
uint16_t numofpages = (endPage-startPage) + 1;
uint16_t pos=0;

// write the data
for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 * MemAddress contains the start page
		 * It needs to be shifted by 6, so that the start page info is correctly starting at bit A6 in the two address bytes that
		 * will be sent to EEPROM
		 * offset relates to the offset inside the start page
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written

		if(HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, I2C_MEMADD_SIZE_16BIT, &data[pos], bytesremaining, 1000) != HAL_OK)
				{
					Error_Handler();
				}// write the data to the EEPROM

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   	// since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
	}
}

/* Read EEPROM */
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	int paddrposition = log(PAGE_SIZE)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000);
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;

	}
	HAL_UART_Transmit(&hlpuart1, data, strlen((const char*)(data)), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, line, sizeof(line), HAL_MAX_DELAY);
	//HAL_Delay(1000);
}

/* Erase Page */
void EEPROM_PageErase (uint16_t page)
{
	// calculate the memory address based on the page number
	int paddrposition = log(PAGE_SIZE)/log(2);
	uint16_t MemAddress = page<<paddrposition;

	// create a buffer to store the reset values
	uint8_t data[PAGE_SIZE];
	memset(data,0xff,PAGE_SIZE);

	// write the data to the EEPROM
	HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, data, PAGE_SIZE, 1000);

	HAL_Delay (5);  // write cycle delay
}
