/* onewire.c
 *
 * A OneWire interface implementation utilizing Timer16B0
 *
 *  2011-02-07    Mat Kattanek
 *                adjusted all MR2 timings for LPC1114
 *                reset() timing now longer 550ms (instead 480)
 *  Created on: Jan 15, 2011
 *      Author: James Harwood
 */

#include "chip.h"                                   // LPC1xxx definitions
#include "onewire.h"
#include "crc8.h"

#define TRUE 1
#define FALSE 0

#define ROM_SEARCH

#ifdef ROM_SEARCHNOT
#include <stdio.h>
#endif

// Define which timer to use (change all three defines below)
#define TIMER_IRQn 	TIMER1_IRQn
#define LPC_TMR 	LPC_TIM1

/*----------------------------------------------------------------------------*
 * end of System configuration section
 *----------------------------------------------------------------------------*/
#define  PINSEL     ((volatile uint32_t*)&LPC_PINCON->PINSEL1)
#define  PINMODE     ((volatile uint32_t*)&LPC_PINCON->PINMODE1)
#define  PINMODE_OD ((volatile uint32_t*)&LPC_PINCON->PINMODE_OD1)

#define GPIO_PINSEL(p,b,v)      PINSEL[(p) * 2 + (b) / 16] = (PINSEL[(p) * 2 + (b) / 16] & ~(3 << ((b) * 2 % 32))) | (v << ((b) * 2 % 32))
#define GPIO_PINMODE_IN(p,b,v)  PINMODE[(p) * 2 + (b) / 16] = (PINMODE[(p) * 2 + (b) / 16] & ~(3 << ((b) * 2 % 32))) | (v << ((b) * 2 % 32))
#define GPIO_PINMODE_OUT(p,b,v) PINMODE_OD[(p)] = ((PINMODE_OD[(p)] & ~(1 << b))) | (v << (b))

#define GPIO_IN_PULLUP     0x00
#define GPIO_IN_REPEATER   0x01
#define GPIO_IN_FLOAT      0x02
#define GPIO_IN_PULLDOWN   0x03

#define GPIO_OUT_NORMAL    0x00
#define GPIO_OUT_OPENDRAIN 0x01

#if 0
#define LPC_GPIO(a)       (LPC_GPIO0 + a*(LPC_GPIO1-LPC_GPIO0))
#define GPIO_DIR_OUT(a,b) (LPC_GPIO(a)->FIODIR |=  (1<<b))
#define GPIO_DIR_IN(a,b)  (LPC_GPIO(a)->FIODIR &= ~(1<<b))
#define GPIO_SET(a,b)     (LPC_GPIO(a)->FIOSET  =  (1<<b))
#define GPIO_CLR(a,b)     (LPC_GPIO(a)->FIOCLR  =  (1<<b))
#define GPIO_IS_SET(a,b)  ((LPC_GPIO(a)->FIOPIN & (1<<b)) !=0)
#else
#define GPIO_DIR_OUT(a,b) Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, a ,b);
#define GPIO_DIR_IN(a,b)  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, a ,b);
#define GPIO_SET(a,b)     Chip_GPIO_SetPinState(LPC_GPIO_PORT, a, b, true);
#define GPIO_CLR(a,b)     Chip_GPIO_SetPinState(LPC_GPIO_PORT, a,b, false);
#define GPIO_IS_SET(a,b)  (Chip_GPIO_GetPinState(LPC_GPIO_PORT, a,b) != false)
#endif

#define OW_OPENDRAIN    GPIO_PINMODE_OUT(OW_PORT, OW_PIN, GPIO_OUT_OPENDRAIN)
#define OW_OUTPUTPOWER  GPIO_PINMODE_OUT(OW_PORT, OW_PIN, GPIO_OUT_NORMAL)

#define OW_PORT   0
#define OW_PIN   11

#define OW_LOW     GPIO_CLR(OW_PORT, OW_PIN)
#define OW_HIGH    GPIO_SET(OW_PORT, OW_PIN)
#define OW_SENSE   GPIO_IS_SET(OW_PORT, OW_PIN)

#if 0
#define OW_OUTPUT 	LPC_GPIO0->FIODIR |= (1 << OW_PIN)
#define OW_INPUT 	LPC_GPIO0->FIODIR &= ~(1 << OW_PIN)
#else
#define OW_OUTPUT 	GPIO_DIR_OUT(OW_PORT, OW_PIN);
#define OW_INPUT 	GPIO_DIR_IN(OW_PORT, OW_PIN);
#endif

// Match Control register bit positions
#define MR0I 0
#define MR1I 3
#define MR2I 6

// prescale timer to 1uS interval
#define PRESCALE	((SystemCoreClock/LPC_SYSCON->SYSAHBCLKDIV)/(1000000ul * 2))

typedef enum {
	OW_RESET, RESET_COMPLETE, WRITE, WR_COMPLETE, READ, RD_COMPLETE,
#ifdef ROM_SEARCH
	WRITE_BIT, WR_BIT_COMPLETE, READ_BIT, RD_BIT_COMPLETE,
#endif
	WAIT_CONV, CONV_COMPLETE, WR_ERROR,
} TOWState;

// Globals used by both the ISR and the main thread
static volatile TOWState ow_state;
static volatile uint8_t presence;
static volatile uint8_t *data_ptr;
static volatile uint16_t data_len;
static volatile uint16_t data_index;
static volatile uint8_t cur_byte;
static volatile uint8_t bit_pos;
static volatile uint8_t single_bit;
static volatile uint8_t wait_conv_state;
static volatile uint8_t tempFlag = 1;


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
void ow_init() {
	/* Custom Initialization */
	Chip_MRT_Init();

	Chip_MRT_SetMode(LPC_MRT_CH0, MRT_MODE_ONESHOT);
	Chip_MRT_SetMode(LPC_MRT_CH1, MRT_MODE_ONESHOT);
	Chip_MRT_SetMode(LPC_MRT_CH2, MRT_MODE_ONESHOT);

	/* Enable the IRQ for the MRT */
	NVIC_EnableIRQ(MRT_IRQn);
}

/*****************************************************************************
 ** Function name:		ow_reset
 **
 ** Description:			Initiate a bus reset operation.
 ** 						Blocks the calling thread until complete.
 **
 ** parameters:			None
 ** Returned value:		0 if no devices present, 1 if devices are present
 **
 *****************************************************************************/
uint8_t ow_reset() {
	presence = 0;
	ow_state = OW_RESET;

	/*
	LPC_TMR->MR0 = (SystemCoreClock / 4000000 * 550) - 1; // was 480    reset drive duration
	LPC_TMR->MR1 = (SystemCoreClock / 4000000 * (550 + 80)) - 1; // was 480+80 presence pulse detection
	LPC_TMR->MR2 = (SystemCoreClock / 4000000 * 2180) - 1; // was 960    reset finished
	LPC_TMR->MCR = (1 << MR0I) | (1 << MR1I) | (1 << MR2I) | (1 << 7);
	*/
	Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 550);
	Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 630);
	Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 2180);

	Chip_MRT_SetEnabled(LPC_MRT_CH0);
	Chip_MRT_SetEnabled(LPC_MRT_CH1);
	Chip_MRT_SetEnabled(LPC_MRT_CH2);

	OW_OUTPUT; // set to output
	OW_LOW; // drive low

	while (ow_state != RESET_COMPLETE)
		; // wait for reset completed
	return presence;
}

/*****************************************************************************
 ** Function name:		ow_write
 **
 ** Description:			Initiate a multi-byte write operation.
 ** 						Blocks the calling thread until complete.
 **
 ** parameters:			data - pointer to a buffer holding the data to write
 ** 						len - the number of bytes to write
 **
 ** Returned value:		None
 **
 *****************************************************************************/
void ow_write(uint8_t *data, uint16_t len) {
	data_ptr = data;
	data_len = len;
	data_index = 0;
	bit_pos = 0;
	cur_byte = data[data_index];
	ow_state = WRITE;
	/*
	LPC_TMR->MR0 = (SystemCoreClock / 4000000 * 3) - 1; // was 2 end of write pulse
	LPC_TMR->MR1 = (SystemCoreClock / 4000000 * 60) - 1; // end of write cycle
	LPC_TMR->MR2 = (SystemCoreClock / 4000000 * 164) - 1; // was 62 ready to start next write cycle
	LPC_TMR->MCR = (1 << MR0I) | (1 << MR1I) | (1 << MR2I)|(1 << 7);
	*/

	OW_OUTPUT; // set to output
	OW_LOW; // drive low
	/*
	LPC_TMR->TCR = 0x01; // start timer
	*/
	//lpc_sct_start();

	Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 3);
	Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 60);
	Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 164);

	while (ow_state != WR_COMPLETE) {
		if (ow_state == WR_ERROR)
			break; // wait for write completed
	}
}

/*****************************************************************************
 ** Function name:		ow_read
 **
 ** Description:			Initiate a multi-byte read operation.
 ** 						Blocks the calling thread until complete.
 **
 ** parameters:			data - pointer to a buffer to store the data read
 ** 						len - the number of bytes to read
 **
 ** Returned value:		None
 **
 *****************************************************************************/
void ow_read(uint8_t *data, uint16_t len) {
	data_ptr = data;
	data_len = len;
	data_index = 0;
	bit_pos = 0;
	cur_byte = 0;
	ow_state = READ;
	/*
	LPC_TMR->MR0 = (SystemCoreClock / 4000000 * 2) - 1; //was 2  end of read pulse
	LPC_TMR->MR1 = (SystemCoreClock / 4000000 * 16) - 1; // time to sample line
	LPC_TMR->MR2 = (SystemCoreClock / 4000000 * 166) - 1; // ready to start next read cycle
	LPC_TMR->MCR = (1 << MR0I) | (1 << MR1I) | (1 << MR2I)|(1 << 7);
	*/

	OW_OUTPUT; // set to output
	OW_LOW; // drive low
	/*
	LPC_TMR->TCR = 0x01; // start timer
	*/
	//lpc_sct_start();
	Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 2);
	Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 16);
	Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 166);

	while (ow_state != RD_COMPLETE)
		; // wait for read completed

}

#ifdef ROM_SEARCH
/*****************************************************************************
 ** Function name:		ow_write_bit
 **
 ** Description:		Writes a single bit to the bus.
 ** 					Blocks the calling thread until complete.
 **
 ** parameters:			bit - value to be written
 **
 **
 ** Returned value:		None
 **
 *****************************************************************************/
void ow_write_bit(uint8_t bit) {
	single_bit = bit;
	ow_state = WRITE_BIT;
	/*
	LPC_TMR->MR0 = (SystemCoreClock / 4000000 * 3) - 1; // end of write pulse
	LPC_TMR->MR1 = (SystemCoreClock / 4000000 * 60) - 1;
	; // end of write cycle
	LPC_TMR->MR2 = (SystemCoreClock / 4000000 * 164) - 1;
	; // ready to start next write cycle
	LPC_TMR->MCR = (1 << MR0I) | (1 << MR1I) | (1 << MR2I)|(1 << 7);
	*/
	OW_OUTPUT; // set to output
	OW_LOW; // drive low
	/*
	LPC_TMR->TCR = 0x01; // start timer
	*/
	//lpc_sct_start();
	Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 3);
	Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 60);
	Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 164);

	while (ow_state != WR_BIT_COMPLETE)
		; // wait for write completed
}

/*****************************************************************************
 ** Function name:		ow_read_bit
 **
 ** Description:		Reads a single bit from the bus.
 ** 					Blocks the calling thread until complete.
 **
 ** parameters:			None
 **
 ** Returned value:		Received bit value
 **
 *****************************************************************************/
uint8_t ow_read_bit() {
	ow_state = READ_BIT;
	single_bit = 0;
	/*
	LPC_TMR->MR0 = (SystemCoreClock / 4000000 * 2) - 1;
	; // end of read pulse
	LPC_TMR->MR1 = (SystemCoreClock / 4000000 * 16) - 1;
	; // time to sample line
	LPC_TMR->MR2 = (SystemCoreClock / 4000000 * 166) - 1;
	; // ready to start next read cycle
	LPC_TMR->MCR = (1 << MR0I) | (1 << MR1I) | (1 << MR2I)|(1 << 7);
	*/

	OW_OUTPUT; // set to output
	OW_LOW; // drive low
	/*
	LPC_TMR->TCR = 0x01; // start timer
	*/
	//lpc_sct_start();
	Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 2);
	Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 16);
	Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 166);

	while (ow_state != RD_BIT_COMPLETE)
		; // wait for read bit completed
	return single_bit;
}

#endif // ROM_SEARCH
/*****************************************************************************
 ** Function name:

 **
 ** Description:			If a DS18S20 is externally powered, it (may) pull the
 ** 						bus low on read bit cycles until the temperature
 ** 						conversion is complete.
 ** 						Blocks the calling thread until complete.
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void ow_wait_conv() {
	OW_HIGH;
	ow_state = WAIT_CONV;
	wait_conv_state = 0;
	/*
	LPC_TMR->MR0 = (SystemCoreClock / 4000000 * 2) - 1;
	; // end of read pulse
	LPC_TMR->MR1 = (SystemCoreClock / 4000000 * 12) - 1;
	; // time to sample line
	LPC_TMR->MR2 = (SystemCoreClock / 4000000 * 76) - 1;
	; // ready to start next read cycle
//	LPC_TMR->MR3 = (SystemCoreClock / 4000000 * 750000) - 1;
//	; // ready to start next read cycle
	LPC_TMR->MCR = (1 << MR0I) | (1 << MR1I) | (1 << MR2I)|(1 << 7);//| (1 << 9)| (1 << 10);
	*/

	OW_OUTPUT; // set to output
	OW_LOW; // drive low
	/*
	LPC_TMR->TCR = 0x01; // start timer
	*/
	//lpc_sct_start();
	Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 2);
	Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 12);
	Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 76);

	while (ow_state != CONV_COMPLETE)
		; // wait for read completed


}

/*****************************************************************************
 ** Function name:		TIMER1_IRQHandler
 **
 ** Description:			Handles the Match Register interrupts during each
 ** 						of the reset, write bit or read bit cycles
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
uint32_t bitbanger = 0;

extern volatile uint32_t sctTick;

void MRT_IRQHandler(void) {
	switch (ow_state) {
		case OW_RESET:
			switch (LPC_MRT->IRQ_FLAG) {
				case MRT0_INTFLAG:
					Chip_MRT_IntClear(LPC_MRT_CH0);
					OW_HIGH; // set pin high
					OW_INPUT; // set to input
					break;
				case MRT1_INTFLAG:
					Chip_MRT_IntClear(LPC_MRT_CH1);
					if (OW_SENSE == 0)
						presence = 1;
					break;
				case MRT2_INTFLAG:
					Chip_MRT_IntClear(LPC_MRT_CH2);
					//lpc_sct_halt();
					ow_state = RESET_COMPLETE;
					break;
				}
			break; // end of reset case

	case WRITE:
		switch (LPC_MRT->IRQ_FLAG) {
			case MRT0_INTFLAG: // end of write pulse
				Chip_MRT_IntClear(LPC_MRT_CH0);
				if ((cur_byte >> bit_pos) & 0x01) {
					OW_HIGH; // set pin high
				} // else keep the line low
				bitbanger |= (1 << 1);
				break;
			case MRT1_INTFLAG: // end of write cycle
				Chip_MRT_IntClear(LPC_MRT_CH1);
				OW_HIGH; // set pin high
				bitbanger |= (1 << 2);
				break;
			case MRT2_INTFLAG: // start next write cycle
				Chip_MRT_IntClear(LPC_MRT_CH2);
				bitbanger |= (1 << 4);
				bit_pos++;
				if (bit_pos == 8) {
					// end of byte
					bit_pos = 0;
					data_index++;
					if (data_index == data_len) {
						// end of data
						ow_state = WR_COMPLETE;
						OW_INPUT; // allow line to be pulled up
						//lpc_sct_halt();
						return;
					} else {
						// next byte
						cur_byte = data_ptr[data_index];
					}
				}
				//lpc_sct_halt();
				bitbanger |= (1 << (bit_pos + 15));
				OW_OUTPUT; // set to output
				OW_LOW; // drive low
				//lpc_sct_start();
				/* Restart */
				Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 3);
				Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 60);
				Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 164);
				break;
			case 0x07: // start next write cycle
				Chip_MRT_ClearIntPending(0x07);
				//lpc_sct_halt();
				bitbanger |= (1 << 7);
				ow_state = WR_ERROR;
				break;
			}
		break; // end of write case

	case READ:
		switch (LPC_MRT->IRQ_FLAG) {
			case MRT0_INTFLAG: // end of read pulse
				Chip_MRT_IntClear(LPC_MRT_CH0);
				OW_HIGH; // set pin high
				OW_INPUT; // set to input
				break;
			case MRT1_INTFLAG: // time to sample line
				Chip_MRT_IntClear(LPC_MRT_CH1);
				if (OW_SENSE) {
					cur_byte |= (1 << bit_pos);
				}
				break;
			case MRT2_INTFLAG: // start next read cycle
				Chip_MRT_IntClear(LPC_MRT_CH2);
				bit_pos++;
				if (bit_pos == 8) {
					// end of byte
					data_ptr[data_index] = cur_byte;
					cur_byte = 0;
					bit_pos = 0;
					data_index++;
					if (data_index == data_len) {
						// end of data
						ow_state = RD_COMPLETE;
						//lpc_sct_halt();
						return;
					}
				}

				OW_OUTPUT; // set to output
				OW_LOW; // drive low
				/* Restart */
				Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 2);
				Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 16);
				Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 166);
				break;
			}
		break; // end of read case

#ifdef ROM_SEARCH
	case WRITE_BIT:
		switch (LPC_MRT->IRQ_FLAG) {
			case MRT0_INTFLAG: // end of write pulse
				Chip_MRT_IntClear(LPC_MRT_CH0);
				if (single_bit) {
					OW_HIGH; // set pin high
				} // else keep the line low
				break;
			case MRT1_INTFLAG: // end of write cycle
				Chip_MRT_IntClear(LPC_MRT_CH1);
				OW_HIGH; // set pin high
				break;
			case MRT2_INTFLAG: // end of write bit cycle
				Chip_MRT_IntClear(LPC_MRT_CH2);
				ow_state = WR_BIT_COMPLETE;
				OW_INPUT; // allow line to be pulled up
				//lpc_sct_halt();
				break;
			}
		break; // end of write bit case

	case READ_BIT:
		switch (LPC_MRT->IRQ_FLAG) {
			case MRT0_INTFLAG: // end of read pulse
				Chip_MRT_IntClear(LPC_MRT_CH0);
				OW_HIGH; // set pin high
				OW_INPUT; // set to input
				break;
			case MRT1_INTFLAG: // time to sample line
				Chip_MRT_IntClear(LPC_MRT_CH1);
				if (OW_SENSE) {
					single_bit = 1;
				}
				break;
			case MRT2_INTFLAG: // end of read bit cycle
				Chip_MRT_IntClear(LPC_MRT_CH2);
				ow_state = RD_BIT_COMPLETE;
				//lpc_sct_halt();
				break;
			}
		break; // end of read bit case
#endif // ROM_SEARCH
	case WAIT_CONV:
		switch (LPC_MRT->IRQ_FLAG) {
			case MRT0_INTFLAG: // end of read pulse
				Chip_MRT_IntClear(LPC_MRT_CH0);
				OW_HIGH; // set pin high
				OW_INPUT; // set to input
				bitbanger |= (1 << 1);
				break;
			case MRT1_INTFLAG: // time to sample line
				Chip_MRT_IntClear(LPC_MRT_CH1);
				if (OW_SENSE) {
					// received a one bit
					//if (wait_conv_state == 1)
					wait_conv_state++;
				}
				/*			else {
				 // received a zero bit
				 if (wait_conv_state == 0)
				 wait_conv_state++;
				 } */
				bitbanger |= (1 << 2);
				break;
			case MRT2_INTFLAG: // start next read cycle
				Chip_MRT_IntClear(LPC_MRT_CH2);
				if (wait_conv_state >= 1) {
					ow_state = CONV_COMPLETE;
					//lpc_sct_halt();
					bitbanger |= (1 << 8);
					return;
				}
				//lpc_sct_halt();
				OW_OUTPUT; // set to output
				OW_LOW; // drive low
				bitbanger |= (1 << 4);

				Chip_MRT_SetInterval(LPC_MRT_CH0, 24 * 2);
				Chip_MRT_SetInterval(LPC_MRT_CH1, 24 * 12);
				Chip_MRT_SetInterval(LPC_MRT_CH2, 24 * 76);
				break;
			case 0x03: // end of write cycle
				bitbanger |= (1 << 3);
				break;
			case 0x07: // start next write cycle
				Chip_MRT_ClearIntPending(0x07);
				//lpc_sct_halt();
				bitbanger |= (1 << 7);
				ow_state = CONV_COMPLETE;
				break;
			case 0x08:
				tempFlag = 1;
				break;
			}

		break; // end of wait convert case
	default:
		break;
	}

	return;
}

//#define PARASITIC 0			// affects the implementation of temperature conversion delay
//DS1820 Registers
#define DS1820_REG_TEMPLSB    0
#define DS1820_REG_TEMPMSB    1
#define DS1820_REG_CNTREMAIN  6
#define DS1820_REG_CNTPERSEC  7
#define DS1820_SCRPADMEM_LEN  9     /* length of scratchpad memory */
#define DS1820_ADDR_LEN       8

// OneWire command definitions
#define CMD_ROM   0x033		// read rom
#define CMD_CONV  0x044		// convert temperature
#define CMD_MATCH 0x055		// match rom
#define CMD_PWR   0x0b4		// read power
#define CMD_READ  0x0be		// read scratch pad
#define CMD_SKIP  0x0cc		// skip rom
// SysTick timer count
extern volatile uint32_t msTicks;

static uint8_t read_buf[10];

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
uint8_t ow_read_rom(RomType *addr) {
	uint8_t crc;
	uint8_t cmd = CMD_ROM;

	crc = ow_reset();

	bitbanger = 0;
	ow_write(&cmd, 1);
	crc = bitbanger;

	ow_read(addr->b, 8);

	crc = crc8(addr->b, 8);
	return crc;
}

void ow_convertTemp() {
	uint8_t cmds = CMD_SKIP;
	uint8_t cmdc = CMD_CONV;

	ow_reset();

	ow_write(&cmds, 1);
	ow_write(&cmdc, 1);
}

void ow_read_scratchpad(uint8_t *spad, uint8_t len) {
	uint8_t crc;
	uint8_t cmds = CMD_SKIP;
	uint8_t cmdr = CMD_READ;

	crc = ow_reset();

	bitbanger = 0;
	ow_write(&cmds, 1);

	bitbanger = 0;
	ow_write(&cmdr, 1);
	crc = bitbanger;

	ow_read(spad, len);

	//        return crc;
}

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
uint8_t ow_read_power() {
	uint8_t cmd = CMD_PWR;
	ow_reset();
	ow_write(&cmd, 1);
	ow_read(read_buf, 1); // only the lsb is significant
	if (read_buf[0] & 0x01) {
		return 1; // ext power
	} else {
		return 0; // parasitic power
	}
}

/*****************************************************************************
 ** Function name:		ow_read_temp
 **
 ** Description:		Read current temperature from DS18x20 sensor
 ** 					Blocks the calling thread until complete.
 **
 ** parameters:			result - address of signed 16 bit word to be filled with
 ** 					temperature value. The format depends on the device
 ** 					part number (DS18S20 vs DS18B20), and user configuration.
 **
 **
 ** 					addr - Rom Address pointer to select a specific device
 **						May be NULL if there is only one device present the on OW bus.
 **
 **						For example:
 **						RomType my_sensor;
 **						my_sensor.l = 0x07100080042ecdf10ull;
 **						result = ow_read_temp(&temp, &my_sensor);
 **
 ** Returned value:		0 on success,
 ** 					1 if crc error,
 ** 					2 if no devices present
 *****************************************************************************/
uint8_t ow_read_temp(uint16_t *result, RomType *addr) {
	static uint8_t wr_buf[10];
	uint8_t crc, i;

	if (!ow_reset())
		return 2;
//	while(!tempFlag){
//
//	}
//
//	tempFlag = 0;
	// convert temperature
	if (addr == NULL) {
		wr_buf[0] = CMD_SKIP;
		wr_buf[1] = CMD_CONV;
		ow_write(wr_buf, 2);
	} else {
		wr_buf[0] = CMD_MATCH;
		for (i = 0; i < 8; i++) {
			wr_buf[i + 1] = addr->b[i];
		}
		wr_buf[9] = CMD_CONV;
		ow_write(wr_buf, 10);
	}
	bitbanger = 0;
	ow_wait_conv();

	if (!ow_reset())
		return 2;

	// read scratch pad
	if (addr == NULL) {
		wr_buf[1] = CMD_READ;
		ow_write(wr_buf, 2);
	} else {
		wr_buf[9] = CMD_READ;
		ow_write(wr_buf, 10);
	}
	ow_read(read_buf, 9);
	crc = crc8(read_buf, 9);
	if (crc == 0) {
		*result = read_buf[0];
		*result |= (read_buf[1] << 8);
		return 0;
	} else {
		return 1;
	}
}

#ifdef ROM_SEARCH
static uint8_t nLastDiscrepancy_u8;
static uint8_t doneFlag = FALSE;

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
uint8_t ow_srch_next(RomType *rom_ptr) {
	uint8_t bitpos_u8 = 1;// ROM Bit index
	uint8_t byteidx_u8 = 0;// ROM Byte index
	uint8_t mask_u8 = 1;// bit mask
	uint8_t state_u8 = 0;
	uint8_t nDiscrepancyMarker_u8 = 0;// discrepancy marker
	uint8_t bit_b;// Output bit
	uint8_t nxt_b;// return value
	uint8_t cmd = 0xF0; // SearchROM command
	uint8_t crc;
	nxt_b = FALSE;// set the next flag to false

	//	for (byteidx_u8 = 0; byteidx_u8 < 8; byteidx_u8++) {
	//		(*rom_ptr).b[byteidx_u8] = 0x00;
	//	}
	if ((!ow_reset()) || doneFlag)// no parts -> return false
	{
		nLastDiscrepancy_u8 = 0;// reset the search
		return FALSE;
	}
	ow_write(&cmd, 1);// send SearchROM command
	do
	/* description for values of state_u8: */
	/* 00    There are devices connected to the bus which have conflicting */
	/*       bits in the current ROM code bit position. */
	/* 01    All devices connected to the bus have a 0 in this bit position. */
	/* 10    All devices connected to the bus have a 1 in this bit position. */
	/* 11    There are no devices connected to the 1-wire bus. */

	/* if there are no devices on the bus */

	{
		state_u8 = 0;
		if (ow_read_bit() == 1) {
			state_u8 = 2;
		}
		if (ow_read_bit() == 1) {
			state_u8 |= 1;// and its complement
		}
		if (state_u8 == 3)// there are no devices on the 1-wire
		{
			break;
		} else {
			if (state_u8 > 0)// all devices coupled have 0 or 1
			{
				bit_b = state_u8 >> 1;// bit write value for search
			} else {
				// if this discrepancy is before the last
				//  discrepancy on a previous Next then pick
				//  the same as last time
				if (bitpos_u8 < nLastDiscrepancy_u8) {
					bit_b = (((*rom_ptr).b[byteidx_u8] & mask_u8) > 0);
				} else {
					// if equal to last pick 1
					bit_b = (bitpos_u8 == nLastDiscrepancy_u8);// if not then pick 0
				}
				// if 0 was picked then record
				//position with mask k
				if (bit_b == 0) {
					nDiscrepancyMarker_u8 = bitpos_u8;
				}
			}
			if (bit_b == 1)// isolate bit in ROM[n] with mask k
			{
				(*rom_ptr).b[byteidx_u8] |= mask_u8;
			} else {
				(*rom_ptr).b[byteidx_u8] &= ~mask_u8;
			}
			ow_write_bit(bit_b);// ROM search write

			/* increment bit position */
			bitpos_u8++;

			/* calculate next mask value */
			mask_u8 = mask_u8 << 1;

			/* check if this byte has finished */
			if (mask_u8 == 0)// if the mask is 0 then go to new ROM
			{ // byte n and reset mask
				byteidx_u8++;
				mask_u8++;
			}
		}
	} while (byteidx_u8 < 8);//loop until through all ROM bytes 0-7

	crc = crc8((*rom_ptr).b, 8);

	/* if search was unsuccessful then */
	if (bitpos_u8 < 65 || (crc != 0)) {
		// reset the last discrepancy to 0
		nLastDiscrepancy_u8 = 0;
	} else {
		// search was successful, so set lastDiscrep,
		//lastOne, nxt
		nLastDiscrepancy_u8 = nDiscrepancyMarker_u8;
		doneFlag = (nLastDiscrepancy_u8 == 0);

		/* indicates search is not complete yet, more parts remain */
		nxt_b = TRUE;
	}
	return nxt_b;
}

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
uint8_t ow_srch_first(RomType *rom_ptr) {
	nLastDiscrepancy_u8 = 0;// reset the rom search last discrepancy global
	doneFlag = FALSE;
	return ow_srch_next(rom_ptr);// call next and return its return value
}

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
short ow_find_devices(RomType *sensorList, short sensorListSize) {
	//RomType ROM;
	unsigned int sensorIndex = 0;
	if (ow_srch_first(&sensorList[sensorIndex])) {
		do {
			//printf("found: %#0.16llx\n", ROM.l);
			//found_devices = ROM.l;
		} while (++sensorIndex < sensorListSize && ow_srch_next(&sensorList[sensorIndex]));
	}else{
		//printf("Nothing");
		//found_devices = 0xCAFE;
	}
	return sensorIndex;
}

#endif  // ROM_SEARCH
