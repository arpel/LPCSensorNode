/*
 * @brief NXP LPCXpresso LPC812 Sysinit file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
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

/* The System initialization code is called prior to the application and
   initializes the board for run-time operation. Board initialization
   for the NXP LPC812 board includes default pin muxing and clock setup
   configuration. */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setup system clocking */
STATIC void SystemSetupClocking(void)
{
#if defined (USE_ROM_API)
	uint32_t cmd[4], resp[2];
#endif

#if (LPC8XX_USE_XTAL_OSC == 1)
	/* EXT oscillator < 15MHz */
	Chip_Clock_SetPLLBypass(false, false);

	/* Turn on the SYSOSC by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_SYSOSC_PD);

	/* Select the PLL input to the external oscillator */
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_SYSOSC);

#else
	/* Turn on the IRC by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_IRC_PD);

	/* Select the PLL input in the IRC */
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_IRC);
#endif /* (LPC8XX_USE_XTAL_OSC == 1) */

	/* Setup FLASH access to 2 clocks (up to 30MHz) */
	Chip_FMC_SetFLASHAccess(FLASHTIM_30MHZ_CPU);

#if defined (USE_ROM_API)
#if (LPC8XX_USE_XTAL_OSC == 1)
	/* Use ROM API for setting up PLL */
	cmd[0] = Chip_Clock_GetMainOscRate() / 1000; /* in KHz */
#else
	cmd[0] = Chip_Clock_GetIntOscRate() / 1000; /* in KHz */
#endif
	cmd[1] = 24000000 / 1000; /* 24MHz system clock rate */
	cmd[2] = CPU_FREQ_EQU;
	cmd[2] = 24000000 / 10000;
	LPC_PWRD_API->set_pll(cmd, resp);

	/* Dead loop on fail */
	while (resp[0] != PLL_CMD_SUCCESS) {}

#else
	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCTL_PowerDown(SYSCTL_SLPWAKE_SYSPLL_PD);

	/* Configure the PLL M and P dividers */
	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 2 = 324Hz
	   MSEL = 1 (this is pre-decremented), PSEL = 2 (for P = 4)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 2 = 24MHz
	   FCCO = FCLKOUT * 2 * P = 24MHz * 2 * 4 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(1, 2);

	/* Turn on the PLL by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_SYSPLL_PD);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsSystemPLLLocked()) {}

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
#endif

	/* Select the CLKOUT clocking source */
	Chip_Clock_SetCLKOUTSource(SYSCTL_CLKOUTSRC_MAINSYSCLK, 1);
}

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
	/* Enable IOCON clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);

#if (LPC8XX_USE_XTAL_OSC == 1)
	/* Use Switch Matrix Tool swm.c file for the Pin Enable 0 variable */
	LPC_SWM->PINENABLE0 = 0xffffff82UL;	/* XTALIN + XTALOUT + CLKOUT on PIO0_1 + ACMP_I1 */

	/* Configure the pins for XTALIN/XTALOUT. */
	LPC_IOCON->PIO0[IOCON_PIO8] &= ~(3 << 3);
	LPC_IOCON->PIO0[IOCON_PIO9] &= ~(3 << 3);

#else
	/* Use Switch Matrix Tool swm.c file for the Pin Enable 0 variable */
	LPC_SWM->PINENABLE0 = 0xffffffb2UL;		/* IRC + CLKOUT on PIO0_1 + ACMP_I1 */
#endif /* (LPC8XX_USE_XTAL_OSC == 1) */

	/* Assign the CLKOUT function to a pin */
	LPC_SWM->PINASSIGN[8] &= ~(0xFE << 16);	/* bits 23:16 = PIN0_x := 1 in this case */

	/* Configure the pin for CLKOUT on PIO0_1 */
	LPC_IOCON->PIO0[IOCON_PIO1] &= ~(3 << 3);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set up and initialize hardware prior to call to main */
void Board_SystemInit(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Setup system clocking and muxing */
	SystemSetupMuxing();
	SystemSetupClocking();

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* IOCON clock left on, but may be turned off if no other IOCON
	   changes are needed */
}
