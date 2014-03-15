/****************************************************************************
 *   Project: ANxxxxx Implementing Sigma Delta ADC using LPC800 Analog
 *            Comparator
 *
 *   Description:
 *     This file contains circular buffer module
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
#include "circular_buffer.h"

struct __cbuff {
	uint8_t    i_last_write;   /*< Index to the last write */
	uint8_t    i_last_read;    /*< Index to the last read */
	uint8_t    available_size; /*< Available buffer size */
	uint32_t   buffer[CONFIG_CIRCULAR_BUFFER_LEN];
};

static struct __cbuff gst_cbuff;

/* \brief Clear / reset the circular buffer
 *
 * \param cb Circular buffer
 */
void cbuf_clear(void) {
	struct __cbuff *cb = &gst_cbuff;

	cb->i_last_write   = 0;
	cb->i_last_read    = 0;
	cb->available_size = CONFIG_CIRCULAR_BUFFER_LEN;
}

/* \brief Check if circular buffer is empty
 *
 * \param cb Circular buffer
 *
 * \return 0 if not empty, otherwise it's empty
 */
static inline int32_t cbuf_is_empty(struct __cbuff *cb) {
	return (cb->available_size == CONFIG_CIRCULAR_BUFFER_LEN);
}

/* \brief Read the data in the circular buffer
 *
 * \param cb Circular buffer
 * \param c  The pointer to user's buffer
 *
 * \return -1 if there's nothing in the buffer, 0 If there's data in the buffer
 */
int32_t cbuf_read(uint32_t *c) {
	struct __cbuff *cb = &gst_cbuff;

	if (cbuf_is_empty(cb)) {
		return -1;
	}

	__disable_irq();
	*c = cb->buffer[cb->i_last_read];
	cb->i_last_read++;
	if (cb->i_last_read == CONFIG_CIRCULAR_BUFFER_LEN) {
		cb->i_last_read = 0;
	}

	cb->available_size++;
	__enable_irq();

	return 0;
}

/* \brief Write the data into the circular buffer
 *
 * This function will override the last data if no space available. This
 * function should only be used in IRQ handler context.
 *
 * \param cb Circular buffer
 * \param c  The pointer to the data
 */
void cbuf_write(uint32_t c) {
	struct __cbuff *cb = &gst_cbuff;

	cb->buffer[cb->i_last_write] = c;
	cb->i_last_write++;
	if (cb->i_last_write == CONFIG_CIRCULAR_BUFFER_LEN) {
		cb->i_last_write = 0;
	}

	if (cb->available_size > 0) {
		cb->available_size--;
	} else {
		/* Force the read index to advance */
		cb->i_last_read++;
		if (cb->i_last_read == CONFIG_CIRCULAR_BUFFER_LEN) {
			cb->i_last_read = 0;
		}
	}
}
