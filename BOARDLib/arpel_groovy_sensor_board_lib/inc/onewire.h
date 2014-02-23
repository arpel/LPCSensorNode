/*
 * onewire.h
 *
 * OneWire interface declarations.
 *
 *
 *  Created on: Jan 15, 2011
 *      Author: James Harwood
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

typedef union {
	uint64_t l;
	uint8_t b[8];
	struct {
		uint8_t crc;
		uint8_t addr[6];
		uint8_t family;
	} s;
} RomType;

/*****************************************************************************
 ** Function name:		ow_init
 **
 ** Description:			Initialize the OneWire interface. Must be called by
 ** 						the application before any of the other functions can
 ** 						be used.
 **
 ** parameters:			None
 ** Returned value:		None
 **
 **
 *****************************************************************************/
void ow_init();

/*****************************************************************************
 ** Function name:		ow_completed
 **
 ** Description:		Test the onewire completion status.
 **
 ** parameters:			None
 ** Returned value:		True if the onewire operation has completed
 **
 *****************************************************************************/
__inline uint8_t ow_completed();

/*****************************************************************************
 ** Function name:		ow_reset
 **
 ** Description:		Initiate a bus reset operation.
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
uint8_t ow_reset();

/*****************************************************************************
 ** Function name:		ow_write
 **
 ** Description:		Initiate a multi-byte write operation.
 **
 ** parameters:			data - pointer to a buffer holding the data to write
 ** 					len - the number of bytes to write
 **
 ** Returned value:		None
 **
 *****************************************************************************/
void ow_write(uint8_t *data, uint16_t len);

/*****************************************************************************
 ** Function name:		ow_read
 **
 ** Description:		Initiate a multi-byte read operation.
 **
 ** parameters:			data - pointer to a buffer to store the data read
 ** 					len - the number of bytes to read
 **
 ** Returned value:		None
 **
 *****************************************************************************/
void ow_read(uint8_t *data, uint16_t len);

/*****************************************************************************
 ** Function name:		ow_read_rom
 **
 ** Description:			Read the Rom address of the device on the bus. Can only
 ** 						be used if there is only a single device present.
 **
 ** parameters:			addr - Rom Address pointer to be filled with Rom value
 ** Returned value:		0 if success, 1 on crc failure
 **
 *****************************************************************************/
uint8_t ow_read_rom(RomType *addr);

/*****************************************************************************
 ** Function name:		ow_wait_conv
 **
 ** Description:		If a DS18S20 is externally powered, it (may) pull the
 ** 					bus low on read bit cycles until the temperature
 ** 					conversion is complete.
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void ow_wait_conv();

/*****************************************************************************
 ** Function name:		ow_read_power
 **
 ** Description:			Detect if any of the devices on the bus are
 ** 						parasitic powered.
 **
 ** parameters:			None
 ** Returned value:		0 if any parasitic powered devices are present
 ** 						1 if all devices are externally powered
 **
 *****************************************************************************/
uint8_t ow_read_power();

/* ---------------------------------------------------------------------------------------
 *  ROM Search functions below taken from Maxim Application Note 162
 *
 *  A final application probably won't need the ROM search functions, so a
 *  fair amount of code space can be saved by commenting out the #define below
 *
 */

#define ROM_SEARCH
#ifdef ROM_SEARCH

/*****************************************************************************
 ** Function name:		ow_srch_first
 **
 ** Description:		Resets the current state of a ROM search and calls
 ** 					ow_srch_next to find the first device on the 1-wire bus.
 **
 **
 ** parameters:			rom_ptr - address of a RomType variable to hold the result
 ** Returned value:		0 if no ROM code was found
 ** 					1 if a ROM code was found
 **
 *****************************************************************************/
uint8_t ow_srch_first(RomType *rom_ptr);

/*****************************************************************************
 ** Function name:		ow_srch_next
 **
 ** Description:		Searches for the next device on the 1-wire bus.
 **
 **
 ** parameters:			rom_ptr - address of a RomType variable to hold the result
 ** Returned value:		0 if no more ROM codes were found
 ** 					1 if a ROM code was found
 **
 *****************************************************************************/
uint8_t ow_srch_next(RomType *rom_ptr);

/*****************************************************************************
 ** Function name:		ow_find_devices
 **
 ** Description:		Search for devices on the 1-wire bus and print out
 ** 					their ROM codes
 **
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
short ow_find_devices(RomType *sensorList, short sensorListSize);

void ow_convertTemp();

uint8_t ow_read_temp(uint16_t *result, RomType *addr);

void ow_read_scratchpad(uint8_t *spad, uint8_t len);

uint64_t passfn();

#endif  // ROM_SEARCH
#endif /* ONEWIRE_H_ */
