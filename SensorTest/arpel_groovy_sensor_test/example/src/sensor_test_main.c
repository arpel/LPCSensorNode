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
#include "RF12.h"

#include <cr_mtb_buffer.h>
__CR_MTB_BUFFER(32);

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define destNodeID 1      // Valid for both rootNode and confNode
#define productionNetwork 100
#define configurationNetwork 200
//#define network 100      // RF12 Network group
#define freq RF12_868MHZ // Frequency of RFM12B module

#define NEED_ACK 0
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 100       // Number of milliseconds to wait for an ack

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

typedef struct {
  uint8_t nodeid;          // Node ID
  uint8_t id;              // Packet ID
  uint8_t status;          //
  uint8_t nbCommands;
  uint8_t lastCommand;
  unsigned short supplyV; // Supply voltage
  uint8_t numsensors;
} Payload_t;

Payload_t staticpayload;
/*****************************************************************************
 * Private functions
 ****************************************************************************/
//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//--------------------------------------------------------------------------------------------------
void RF_AirSend(Payload_t *pl){
  //bitClear(PRR, PRUSI); // enable USI h/w
  //digitalWrite(LEDpin, LOW);
  uint8_t i;
  uint8_t header;

  for (i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
       rf12_sleep(RF12_WAKEUP);

       header = RF12_HDR_ACK | RF12_HDR_DST | destNodeID;

       rf12_sendNow(header, pl, sizeof *pl);
       rf12_sendWait(2); // Wait for RF to finish sending while in standby mode
#if NEED_ACK
         byte acked = waitForAck();  // Wait for ACK
#endif
       rf12_sleep(RF12_SLEEP);
#if NEED_ACK
         if (acked) { break; }      // Return if ACK received
         loseSomeTime(RETRY_PERIOD * 500);     // If no ack received wait and try again
#else
          break;
#endif
  }

  //digitalWrite(LEDpin, HIGH);
  //bitSet(PRR, PRUSI); // disable USI h/w
}

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

	staticpayload.id += 1;
	RF_AirSend(&staticpayload);
}


/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	int count = 0;
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

/**/
	staticpayload.nodeid = 22;
	rf12_initialize(22, freq, productionNetwork, 1600); // Initialize RFM12 with settings defined above
	// Adjust low battery voltage to 2.2V
	rf12_control(0xC040);
	rf12_sleep(RF12_SLEEP);  // Put the RFM12 to sleep

	/**/

	while (1) {
		__WFI();
	}

	return 0;
}
