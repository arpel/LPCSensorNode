/*
 * @brief NXP LPCXpresso LPC812 board file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "string.h"
#include "retarget.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

static const uint8_t ledBits[] = {7, 17};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* System oscillator rate and clock rate on the CLKIN pin */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize the LEDs on the NXP LPC812 LPCXpresso Board */
static void Board_LED_Init(void)
{
	int i;

	for (i = 0; i < sizeof(ledBits); i++) {
		Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, ledBits[i]);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, ledBits[i], true);
	}
}

/* Board Debug UART Initialisation function */
STATIC void Board_UART_Init(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Divided by 1 */
	Chip_Clock_SetUARTClockDiv(1);

	/* Connect the U0_TXD_O and U0_RXD_I signals to port pins(P0.4, P0.0) */
	Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 4);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 0);

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set the LED to the state of "On" */
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, ledBits[LEDNumber], (bool) !On);
}

/* Return the state of LEDNumber */
bool Board_LED_Test(uint8_t LEDNumber)
{
	return (bool) !Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, ledBits[LEDNumber]);
}

/* Toggles the current state of a board LED */
void Board_LED_Toggle(uint8_t LEDNumber)
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, 0, ledBits[LEDNumber]);
}

/* Sends a character on the UART */
void Board_UARTPutChar(char ch)
{
#if defined(DEBUG_UART)
	Chip_UART_SendBlocking(DEBUG_UART, &ch, 1);
#endif
}

/* Gets a character from the UART, returns EOF if no character is ready */
int Board_UARTGetChar(void)
{
#if defined(DEBUG_UART)
	uint8_t data;

	if (Chip_UART_Read(DEBUG_UART, &data, 1) == 1) {
		return (int) data;
	}
#endif
	return EOF;
}

/* Outputs a string on the debug UART */
void Board_UARTPutSTR(char *str)
{
#if defined(DEBUG_UART)
	while (*str != '\0') {
		Board_UARTPutChar(*str++);
	}
#endif
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void)
{
#if defined(DEBUG_UART)
	Board_UART_Init();
	Chip_UART_Init(DEBUG_UART);
	Chip_UART_ConfigData(DEBUG_UART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(DEBUG_UART, 115200);
	Chip_UART_Enable(DEBUG_UART);
	Chip_UART_TXEnable(DEBUG_UART);
#endif
}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void)
{
	/* Sets up DEBUG UART */
	DEBUGINIT();

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/* Initialize the LEDs */
	Board_LED_Init();
}

#if 0

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#include "onewire.h"
#include "stdio.h"
#include "timer.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP;

#define SENSORS 12

RomType t_sensor[SENSORS];
uint16_t t_val; //temperature value
short temperature[SENSORS]; //temperature *10 (to avoid float)
uint8_t sensor_counter; //counter for sensor loop


int main(void) {
	printf("LPC1769_DS18S20 Init..."__DATE__" "__TIME__"\n");
	ow_init(); //init ow
	if (ow_reset()) {
		ow_find_devices();
	}

	t_sensor[0].l = 0x65000802a4464010;
	t_sensor[1].l = 0x1e000802a509b010;
	t_sensor[2].l = 0x1b000802a4c86810;
	t_sensor[3].l = 0xee000802a43e6210;
	t_sensor[4].l = 0xd1000802a4e8ca10;
	t_sensor[5].l = 0x92000802a54b5a10;
	t_sensor[6].l = 0x21000802a490d110;
	t_sensor[7].l = 0x2b000802a4ccbd10;
	t_sensor[8].l = 0x10000802a4b59310;
	t_sensor[9].l = 0xe6000802a4b7d310;
	t_sensor[10].l = 0x4f000802a5338710;
	t_sensor[11].l = 0x6e000802a481ef10;

/*
	t_sensor[0].l = 0xc2000802a51c4010;
	t_sensor[1].l = 0xd2000802a526a010;
	t_sensor[2].l = 0x42000802a4e86010;
	t_sensor[3].l = 0x1e000802a51c4410;
	t_sensor[4].l = 0x6f000802a4aee610;
	t_sensor[5].l = 0x13000802a4af7610;
	t_sensor[6].l = 0xba000802a4a39e10;
	t_sensor[7].l = 0x47000802a477e110;
	t_sensor[8].l = 0x7c000802a53a7110;
	t_sensor[9].l = 0x11000802a4dff110;
	t_sensor[10].l = 0x5b000802a547c910;
	t_sensor[11].l = 0x8a000802a4386910;
	t_sensor[12].l = 0x43000802a4d6f910;
	t_sensor[13].l = 0xf7000802a5559510;
	t_sensor[14].l = 0x0b000802a5243510;
	t_sensor[15].l = 0xd6000802a4e96310;
	t_sensor[16].l = 0x0c000802a5524710;
	t_sensor[17].l = 0x63000802a4a56f10;
*/
	//	t_sensor[1].l = 0x4800080284984810; //64-Bit Serial Code sensor #2
	//	t_sensor[2].l = 0x580008028499c910; //64-Bit Serial Code sensor #3
	//	t_sensor[3].l = 0x5a0008028483a910; //64-Bit Serial Code sensor #4
	//	t_sensor[4].l = 0x130008028495ed10; //64-Bit Serial Code sensor #5
	//	t_sensor[5].l = 0x0300080281733310; //64-Bit Serial Code sensor #1

	while (1) {
		//read sensors
		for (sensor_counter = 0; sensor_counter < SENSORS; sensor_counter++) {
			if (ow_read_temp(&t_val, &t_sensor[sensor_counter])) {
				printf("Error Sensor %d\n", (sensor_counter + 1));
			} else {
				temperature[sensor_counter] = (t_val * 5); //read uint to int
				printf("Temperature #%d: %d \n", sensor_counter + 1,
						temperature[sensor_counter]);
			}
		} //end sensor loop
	}
	return 0;
}
#endif
