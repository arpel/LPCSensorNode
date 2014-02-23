/*
 * @brief Blinky example using SysTick and interrupt
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
#include <stdio.h>

#include "onewire.h"

#include <cr_mtb_buffer.h>
__CR_MTB_BUFFER(32);

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (5)	/* 10 ticks per second */
#define TIMEOUT_TICKS (100 * TICKRATE_HZ)	/* 10 seconds */

/* SystemTick Counter */
static volatile uint32_t sysTick;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#define OW_MAX_SENSORS 4
RomType OW_sensorList[OW_MAX_SENSORS];
short OW_sensorCount = 0;

uint16_t t_val; //temperature value
short temperature[OW_MAX_SENSORS]; //temperature *10 (to avoid float)
uint8_t sensor_counter; //counter for sensor loop

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	Board_LED_Toggle(0);
	sysTick++;

	for (sensor_counter = 0; sensor_counter < OW_sensorCount; sensor_counter++) {
		if (ow_read_temp(&t_val, &OW_sensorList[sensor_counter])) {
			//printf("Error Sensor %d\n", (sensor_counter + 1));
		} else {
			temperature[sensor_counter] = (t_val * 5); //read uint to int
		}
	}
}


/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{

	SystemCoreClockUpdate();
	Board_Init();

	Board_LED_Set(0, false);

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/* Power UP the sensor */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0 , 12);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 12, true);

	ow_init();

	if (ow_reset()) {
		OW_sensorCount = ow_find_devices(OW_sensorList, OW_MAX_SENSORS);
	}


	while (1) {
		__WFI();
	}

	return 0;
}
