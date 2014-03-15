/****************************************************************************
 *   Project: ANxxxxx Implementing Sigma Delta ADC using LPC800 Analog
 *            Comparator
 *
 *   Description:
 *     This file contains sigma delta implementation using LPC800 comparator
 *     and SCT
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.

 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors'
 * relevant copyright in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers. This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 ****************************************************************************/

//#include <LPC8xx.h>
#include <chip.h>
#include "lpc8xx_sd_adc.h"

/* --------------------------------------------------------------------------
   SD-ADC Module internal configuration
   --------------------------------------------------------------------------*/

/* The result of the ADC reading */
static uint32_t gu32_adc_result;

/* --------------------------------------------------------------------------
   Analog comparator driver
   --------------------------------------------------------------------------*/

#define ACMP_IN_VLADDER_OUTPUT    0x00
#define ACMP_IN_ACMP_I1           0x01
#define ACMP_IN_ACMP_I2           0x02
#define ACMP_IN_INTERNAL_BANDGAP  0x06

/** \brief Assign PIO0_0 => ACMP_I1 */
static void pinassign_acmp_i1(void) {
	/* Disable pull up / pull down resistor for ACMP_I1 */
	LPC_IOCON->PIO0[IOCON_PIO0] = 0x80;
	LPC_SWM->PINENABLE0 &= ~(1 << 0); /* Enable ACMP_I1 */
}
/** \brief Assign PIO0_1 => ACMP_I2 */
static void pinassign_acmp_i2(void) {
	/* Disable pull up / pull down resistor for ACMP_I2 */
	LPC_IOCON->PIO0[IOCON_PIO1] = 0x80;
	LPC_SWM->PINENABLE0 &= ~(1 << 1); /* Enable ACMP_I2 */
}

/** \brief Assign pin to ACMP_O */
static void pinassign_acmp_o(uint32_t pin) {
	pin = pin & 0xFF;

	LPC_SWM->PINASSIGN[8] &=~(0xFF << 8);
	LPC_SWM->PINASSIGN[8] |= (pin << 8);  /* Enable ACMP_O */
}

/** \brief Init analog comparator */
static void acmp_init(void) {
	/* Power up comparator */
	LPC_SYSCTL ->PDRUNCFG &= ~(1 << 15);

	/* Clear comparator reset */
	LPC_SYSCTL ->PRESETCTRL |=  (1 << 12);

	/* Enable register access to the comparator */
	LPC_SYSCTL ->SYSAHBCLKCTRL |= (1 << 19);
}

/** \brief Disable analog comparator */
static void acmp_deinit(void) {
	/* Disable register access to the comparator */
	LPC_SYSCTL ->SYSAHBCLKCTRL &=~(1 << 19);

	/* Reset the comparator */
	LPC_SYSCTL ->PRESETCTRL &= ~(1 << 12); /*< Reset the comparator */

	/* Power down comparator */
	LPC_SYSCTL ->PDRUNCFG |=  (1 << 15);
}

/** \brief Configure the analog comparator
 *
 * \param vp_channel Positive input
 * \param vn_channel Negative input
 * \param is_output_synced 0 if output should not be sync with the core clock.
 *                           Otherwise it should be synced
 *
 */
static void acmp_configure(uint32_t vp_channel, uint32_t vn_channel,
		uint32_t is_output_synced) {
	LPC_CMP ->CTRL = 0;
	LPC_CMP ->CTRL |= ((vp_channel & 7) << 8) | ((vn_channel & 7) << 11);

	if (is_output_synced) {
		LPC_CMP ->CTRL |= (1 << 6);
	} else {
		/* Disable register access to the comparator to save power*/
		LPC_SYSCTL ->SYSAHBCLKCTRL &=~(1 << 19);
	}
}


/* --------------------------------------------------------------------------
   Voltage ladder driver
   --------------------------------------------------------------------------*/

/** \brief Enable Voltage ladder output
 *
 *  \param vladder_div Voltage ladder division
 */
static void vladder_enable(uint32_t vladder_div) {
	uint32_t delay;

	LPC_CMP ->LAD = ((vladder_div & 0x1F) << 1) | 1;

	/* Delay is needed to stabilize the VLadder output */
	for (delay = 0; delay < 0xFF; delay++);
}

/** \brief Disable Voltage ladder output
 */
static void vladder_disable(void) {
	LPC_CMP ->LAD = 0;
}

/* --------------------------------------------------------------------------
   SCT driver
   --------------------------------------------------------------------------*/

/** \brief Assign pin to CTIN_0 */
static void pinassign_ctin_0(uint32_t pin) {
	pin = pin & 0xFF;

	LPC_SWM->PINASSIGN[5] &=~(0xFF << 24);
	LPC_SWM->PINASSIGN[5] |= (pin << 24);  /* Enable CTIN_0 */
}

#if defined (CONFIG_ENABLE_DEBUG_PIN)
/** \brief Assign pin to CTOUT_0 */
static void pinassign_ctout_0(uint32_t pin) {
	pin = pin & 0xFF;

	LPC_SWM->PINASSIGN[6] &=~(0xFF << 24);
	LPC_SWM->PINASSIGN[6] |= (pin << 24);  /* Enable CTIN_0 */

}
#endif

/** \brief Initialize SCT to be used in sigma delta ADC
 *
 * The SCT is needed to sample, decimate and average the analog comparator
 * output.
 *
 * ACMP_O ----> CTIN_0 -----> LOW_COUNTER
 *                             |       |
 *           Event #1: Triggered       Event #2: Triggered when ACMP_O falling
 *           when ACMP_O rising                      |
 *                    |                              |
 *              SET CTOUT_0                   CLEAR CTOUT_0
 *
 * HIGH_COUNTER------> Event #0: Triggered when window size limit is reached
 *                                      |
 *                    CLEAR HIGH_COUNTER and CLEAR LOW_COUNTER
 *
 * The SCT is operating without any state variable
 */
static void sct_init(void) {
	/* Enable register access to the SCT */
	LPC_SYSCTL ->SYSAHBCLKCTRL |= (1 << 8);

	/* Clear SCT reset */
	LPC_SYSCTL ->PRESETCTRL |=  (1 << 8);

	/* Set SCT:
	 * - As two 16-bit counters
	 * - Use bus clock as the SCT and prescaler clock
	 * - Allows reload the match registers
	 * - Sync CTIN_0 with bus clock before triggering any event
	 */
	LPC_SCT->CONFIG = 0x200;

	/* Set averaging window size. Reset the HIGH and LOW counter when maximum
	 * window size reached, and generate interrupt to save the LOW counter value
	 *
	 * This maximum counting is used as max window size
	 */
	LPC_SCT->REGMODE_H = 0x0000;          /*< Use HIGH counter register 0 as
	                                          Match */
	LPC_SCT->MATCH[0].H     = CONFIG_WINDOW_SIZE - 1;
	LPC_SCT->MATCHREL_H[0]  = CONFIG_WINDOW_SIZE - 1;
	LPC_SCT->EVENT[0].CTRL  = 0x00001010; /*< Set event when MR0 = COUNT_H =
		                                          max window size */
	LPC_SCT->EVENT[0].STATE = 0x00000003; /*< Trigger event at any state */
	LPC_SCT->LIMIT_H = 0x0001;            /*< Reset HIGH counter when Event #0,
	                                          triggered */

	/* Set capture event to capture the analog comparator output. The capture
	 * event will be triggered if the analog output is toggled
	 */
	LPC_SCT->EVENT[1].CTRL  = 0x00002400; /*< Set event when CTIN_0 is rising
	                                       */
	LPC_SCT->EVENT[1].STATE = 0x00000003; /*< Trigger event at any state */
	LPC_SCT->START_L = 0x0002;            /*< Start counting on LOW counter when
	                                          Event #1 triggered */
#if defined (CONFIG_ENABLE_DEBUG_PIN)
	LPC_SCT->OUT[0].SET = 2;              /*< Set CTOUT_0 if Event #1 triggered
	                                       */
#endif
	LPC_SCT->EVENT[2].CTRL  = 0x00002800; /*< Set event when CTIN_0 is falling
	                                       */
	LPC_SCT->EVENT[2].STATE = 0x00000003; /*< Trigger event at any state */
	LPC_SCT->STOP_L = 0x0004;            /*< Stop counting on LOW counter when
	                                          Event #2 triggered */
#if defined (CONFIG_ENABLE_DEBUG_PIN)
	LPC_SCT->OUT[0].CLR = 4;              /*< Clear CTOUT_0 if Event #2
	                                          triggered */
#endif
	/* Trigger interrupt when Event #0 triggered */
	LPC_SCT->EVEN =    0x00000001;

	/* Start the HIGH counter:
	 * - Counting up
	 * - Single direction
	 * - Prescaler = bus_clk / CONFIG_SD_ADC_PRESCALER
	 * - Reset the HIGH counter
	 */
	LPC_SCT->CTRL_H = ((CONFIG_SD_ADC_PRESCALER - 1) << 5) | 0x08;

	/* Stop the LOW counter:
	 * - Counting up
	 * - Single direction
	 * - Prescaler = bus_clk / CONFIG_SD_ADC_PRESCALER
	 * - Reset the LOW counter
	 */
	LPC_SCT->CTRL_L = ((CONFIG_SD_ADC_PRESCALER - 1) << 5) | 0x0A;
}

static void sct_deinit(void) {
	/* Stop SCT */
	LPC_SCT->CTRL_H = 0x04;

	/* Reset SCT */
	LPC_SYSCTL ->PRESETCTRL &= ~(1 << 8);

	/* Disable register access to the SCT */
	LPC_SYSCTL ->SYSAHBCLKCTRL &=~(1 << 8);
}

/* --------------------------------------------------------------------------
   Sigma delta ADC driver
   --------------------------------------------------------------------------*/
/** \brief Initialize the sigma delta ADC module
 *
 * \param feedback_pin       The GPIO for sigma delta feedback pin
 * \param sdadc_debug_pin    The debug pin to view the signal that's captured by
 *                           the SD ADC. It should be different from
 *                           feedback_pin
 * *
 * \return -1 if fault detected, 0 otherwise.
 */
int32_t sdadc_init(uint32_t feedback_pin, uint32_t sdadc_debug_pin) {
	pinassign_acmp_i2();
	pinassign_acmp_o(feedback_pin);
	pinassign_ctin_0(feedback_pin);
#if defined (CONFIG_ENABLE_DEBUG_PIN)
	pinassign_ctout_0(sdadc_debug_pin);
#else
	sdadc_debug_pin = sdadc_debug_pin; /* Avoid compiler warning */
#endif

	sdadc_resume();
	return 0;
}

/** \brief Resume sigma delta ADC operation
 *
 */
void sdadc_resume(void) {
	acmp_init();
	sct_init();

	vladder_enable(CONFIG_VLADDER_PRESCALER);

	/* Analog comparator setup:
	 *     V+ => VDD / 2
	 *     V- => ACMP_I1 (PIO0_0)
	 *     Output => comparator_out_pin
	 */
	acmp_configure(ACMP_IN_VLADDER_OUTPUT, ACMP_IN_ACMP_I2, 0);
}

/** \brief Suspend the sigma delta ADC operation
 *
 */
void sdadc_suspend(void) {
	vladder_disable();
	sct_deinit();
	acmp_deinit();
}

/** \brief Read the sigma delta ADC result
 *
 * \param result The sigma delta ADC result
 * \return -1 if fault detected, 0 otherwise
 */
int32_t sdadc_getAdcReading(uint32_t *result) {
	*result = gu32_adc_result;

	return 0;
}

/** \brief ADC SCT interrupt handler
 *
 * This function needs to be called from user's SCT interrupt handler
 */
void sdadc_sct_irqhandler(void) {
	uint32_t event_flag = LPC_SCT->EVFLAG;
	static uint32_t old_capture_val;
	uint32_t capture_val;

	/* Reject interrupt from events that are not belonging to this module */
	if ((event_flag & 1) == 0) {
		return;
	}

	capture_val = LPC_SCT->COUNT_L;
	gu32_adc_result = (0x10000 + capture_val - old_capture_val) & 0xFFFF;
	old_capture_val = capture_val;

	SDADC_USER_IRQ(gu32_adc_result);

	LPC_SCT->EVFLAG = 1;    /*< Clear the IRQ flag */
}
