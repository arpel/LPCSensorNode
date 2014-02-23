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

#include <cr_mtb_buffer.h>
__CR_MTB_BUFFER(32);

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (10)	/* 10 ticks per second */
#define TIMEOUT_TICKS (100 * TICKRATE_HZ)	/* 10 seconds */

/* SystemTick Counter */
static volatile uint32_t sysTick;
static volatile uint32_t sctTick;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

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
	//Board_LED_Toggle(0);
	sysTick++;
}

/**
 * @brief	Handle interrupt from State Configurable Timer
 * @return	Nothing
 */
void SCT_IRQHandler(void)
{
	Board_LED_Toggle(0);
	sctTick++;

	/* Clear the Interrupt */
	Chip_SCT_ClearEventFlag(LPC_SCT, SCT_EVT_0);
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

#if 0
	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/* Bail out after timeout */
	while (sysTick <= TIMEOUT_TICKS) {
		__WFI();
	}
#else
	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/* Custom Initialization */
	Chip_SCT_Init(LPC_SCT);

	/* Configure the SCT as a 32bit counter using the bus clock */
	Chip_SCT_Config(LPC_SCT, (0xf << 9) | SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_CLKMODE_BUSCLK);

	/* The match/capture REGMODE defaults to match mode */

	/* Set the match count for match register 0 */
	Chip_SCT_SetMatchCount(LPC_SCT, SCT_MATCH_0, SystemCoreClock / TICKRATE_HZ);

	/* Set the match reload value */
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, SystemCoreClock / TICKRATE_HZ);

	LPC_SCT->EVENT[0].CTRL = 0x5000;
	LPC_SCT->EVENT[0].STATE = 0x1;

	LPC_SCT->LIMIT = 0x1;

	/* Enable an Interrupt on the Match Event */
	Chip_SCT_EnableEventInt(LPC_SCT, SCT_EVT_0);

	/* Enable the IRQ for the SCT */
	NVIC_EnableIRQ(SCT_IRQn);

	LPC_SCT->CTRL_U &= ~(1 << 2);                   /* unhalt the SCT by clearing bit 2 of the unified CTRL register  */

	while (1) {
		__WFI();
	}
#endif

	return 0;
}
