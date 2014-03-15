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

#ifndef LPC8XX_SD_ADC_H_
#define LPC8XX_SD_ADC_H_

/* Define the sampling rate of the sigma delta ADC. The sampling rate is equal
 * to (system_clock / CONFIG_SD_ADC_PRESCALER). Maximum SD ADC sampling rate is
 * 2Mhz.
 *
 * Minimum CONFIG_SD_ADC_PRESCALER = 1, maximum CONFIG_SD_ADC_PRESCALER = 256
 */
#define CONFIG_SD_ADC_PRESCALER       12

/* Define the averaging window size.
 *
 * This value will determine the final ADC rate. For example, value 1024 will
 * generate final ADC reading per ((1024 * CONFIG_ADC_PRESCALER) / system_clock)
 * seconds
 *
 * Minimum CONFIG_WINDOW_SIZE = 1, maximum CONFIG_WINDOW_SIZE = 65535
 */
#define CONFIG_WINDOW_SIZE            1024

/* Define the VLADDER_OUT voltage
 *
 * The VLADDER_OUT is the threshold for the integrator. The VLADDER_OUT value
 * is defined as (CONFIG_VLADDER_PRESCALER / 31 * VDD)
 *
 * Minimum CONFIG_VLADDER_PRESCALER = 0, maximum CONFIG_VLADDER_PRESCALER = 31
 */
#define CONFIG_VLADDER_PRESCALER      15

/* Enable the debug pin to view the signal that is captured by the SD ADC
 *
 * Undefine this to disable the debug pin
 */
//#define CONFIG_ENABLE_DEBUG_PIN

/* Define user interrupt function that will be called when ADC conversion is
 * done. We use this to avoid function pointer (which taking too much time in
 * ARM Cortex-M0.
 *
 * The function should follow this footprint: void function(uint32_t adc_val)
 *
 * \param x The ADC value.
 */
extern void sdadc_user_irqhandler(uint32_t adc_val);
#define SDADC_USER_IRQ(x)             sdadc_user_irqhandler(x)

/** \brief ADC SCT interrupt handler
 *
 * This function needs to be called from user's SCT interrupt handler
 */
void sdadc_sct_irqhandler(void);

/** \brief Initialize the sigma delta ADC module
 *
 * \param feedback_pin       The GPIO for sigma delta feedback pin
 * \param sdadc_debug_pin    The debug pin to view the signal that's captured by
 *                           the SD ADC. It should be different from
 *                           feedback_pin
 * *
 * \return -1 if fault detected, 0 otherwise.
 */
int32_t sdadc_init(uint32_t feedback_pin, uint32_t sdadc_debug_pin);

/** \brief Read the sigma delta ADC result
 *
 * \param result The sigma delta ADC result
 * \return -1 if fault detected, 0 otherwise
 */
int32_t sdadc_getAdcReading(uint32_t *result);

/** \brief Suspend the sigma delta ADC operation
 *
 * This function is used as part of power saving feature that needs to be called
 * by user before entering power saving mode
 */
void sdadc_suspend(void);

/** \brief Resume sigma delta ADC operation
 *
 * This function is used as part of power saving feature that needs to be called
 * by user after receiving resume signal
 */
void sdadc_resume(void);

#endif /* LPC8XX_SD_ADC_H_ */
