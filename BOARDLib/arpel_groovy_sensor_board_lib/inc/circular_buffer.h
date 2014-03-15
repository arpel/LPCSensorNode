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

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#define CONFIG_CIRCULAR_BUFFER_LEN    20

/* \brief Clear / reset the circular buffer
 *
 * \param cb Circular buffer
 */
void cbuf_clear(void);

/* \brief Read the data in the circular buffer
 *
 * \param cb Circular buffer
 * \param c  The pointer to user's buffer
 *
 * \return -1 if there's nothing in the buffer, 0 If there's data in the buffer
 */
int32_t cbuf_read(uint32_t *c);

/* \brief Write the data into the circular buffer
 *
 * This function will override the last data if no space available. This
 * function should only be used in IRQ handler context.
 *
 * \param cb Circular buffer
 * \param c  The pointer to the data
 */
void cbuf_write(uint32_t c);

#endif /* CIRCULAR_BUFFER_H_ */
