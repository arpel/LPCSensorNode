/****************************************************************************
 *   Project: ANxxxxx Implementing Sigma Delta ADC using LPC800 Analog
 *            Comparator
 *
 *   Description:
 *     This file contains simple IIR filter implementation
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

#include <stdint.h>
#include "simple_iir.h"

#if defined (ENABLE_IIR_LPF)
/* Single pole Low Pass Filter - IIR
 * y(n) = (a.x(n) + b.y(n-1)) >> 10;
 *
 * a = (1 - x) * 1024
 * b = x * 1024
 * x = e^-2.pi.fc
 * fc = Fcutoff / fsampling
 *
 * fsampling = (system_clock / CONFIG_SD_ADC_PRESCALER) / CONFIG_WINDOW_SIZE
 */
static uint32_t yn_1;

uint32_t iir_lpf(uint32_t x) {
	uint32_t y = ((980 * x) +(44 * yn_1)) >> 10;

	yn_1 = y;

	return y;
}
#endif
