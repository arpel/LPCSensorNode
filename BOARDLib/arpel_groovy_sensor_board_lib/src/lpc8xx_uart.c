/****************************************************************************
 *   Project: ANxxxxx Implementing Sigma Delta ADC using LPC800 Analog
 *            Comparator
 *
 *   Description:
 *     This file contains UART driver
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

#include <LPC8xx.h>

#define UART_BAUDRATE  115200
#define UART_TX_PIN    4

/* \brief Init the UART to 115200bps
 *
 * Set PIO0.4 as UART TXD. Assuming that the MCU run at 36Mhz, the UART clock is
 * set to 115200bps
 */
void uart_init(void) {
	LPC_SWM->PINASSIGN0 &= ~0xFF;
	LPC_SWM->PINASSIGN0 |=  UART_TX_PIN;

	/* Enable UART clock */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 14);

	/* Peripheral reset control to UART, a "1" bring it out of reset. */
	LPC_SYSCON->PRESETCTRL &= ~0x08;
	LPC_SYSCON->PRESETCTRL |=  0x08;

	/* UART Clock = SystemCoreClock / LPC_SYSCON->UARTCLKDIV */
	LPC_SYSCON->UARTCLKDIV = 1;
	LPC_USART0->BRG = (SystemCoreClock /
			(LPC_SYSCON->UARTCLKDIV * 16 * UART_BAUDRATE)) - 1;

	LPC_USART0->CFG = 4; /* Mode 8-N-1 */
	LPC_USART0->CFG|= 1; /* Enable UART */
}

/* Send the UART data
 *
 * \param data The pointer to data buffer
 * \param len  The length of the data
 */
void uart_send(uint8_t *data, uint32_t len) {
	while (len) {
		while ((LPC_USART0->STAT & 0x04) == 0); /* Wait until UART is ready */
		LPC_USART0->TXDATA = *data;
		data++; len --;
	}
}

/* Send single character
 *
 * \param c Character to be sent
 */
int uart_putchar(int c) {
	while ((LPC_USART0->STAT & 0x04) == 0); /* Wait until UART is ready */
	LPC_USART0->TXDATA = c;

	return 1;
}
