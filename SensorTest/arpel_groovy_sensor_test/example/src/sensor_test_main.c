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

/**/
static bool init_done = false;

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
static void WakeupTest(WKT_CLKSRC_T clkSrc, uint32_t timeoutInSecs, CHIP_PMU_MCUPOWER_T powerTest)
{
	/* 10KHz clock source */
	Chip_WKT_SetClockSource(LPC_WKT, clkSrc);

	/* Setup for wakeup in 5s */
	Chip_WKT_LoadCount(LPC_WKT, Chip_WKT_GetClockRate(LPC_WKT) * timeoutInSecs);

	/* We can optionally call Chip_SYSCTL_SetDeepSleepPD() to power down the BOD and WDT if we aren't using them in deep sleep modes. */
	Chip_SYSCTL_SetDeepSleepPD(SYSCTL_DEEPSLP_BOD_PD | SYSCTL_DEEPSLP_WDTOSC_PD);

	/* We should call Chip_SYSCTL_SetWakeup() to setup any peripherals we want to power back up on wakeup. For this example, we'll power back up the IRC, FLASH, the system oscillator, and the PLL */
	Chip_SYSCTL_SetWakeup(~(SYSCTL_SLPWAKE_IRC_PD | SYSCTL_SLPWAKE_IRCOUT_PD | SYSCTL_SLPWAKE_FLASH_PD | SYSCTL_SLPWAKE_SYSOSC_PD | SYSCTL_SLPWAKE_SYSPLL_PD));

	/* Tell PMU to go to sleep */
	Chip_PMU_Sleep(LPC_PMU, powerTest);

	/* Power anything back up here that isn't powered up on wakeup. The example code below powers back up the BOD and WDT oscillator, which weren't setup to power up in the Chip_SYSCTL_SetWakeup() function. */
	Chip_SYSCTL_SetDeepSleepPD(0);

	/* Will return here after wakeup and WKT IRQ, LED should be on */
	Chip_WKT_Stop(LPC_WKT);
}

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
void PeriodicWakeUP(void)
{
	/* LED will toggle state on wakeup event */
	Board_LED_Toggle(0);

	sysTick++;

	PeriodicTask();
}

void PeriodicTask(void)
{
	/* Power UP the sensor */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0 , 12);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 12, false);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 12, true);

	//ow_init();

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
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	if(!init_done)
		return;

	PeriodicWakeUP();

	if(staticpayload.id >= 10)
	{
		WakeupTest(WKT_CLKSRC_10KHZ, 5, PMU_MCU_POWER_DOWN);
	}
}

/**
 * @brief	Handle interrupt from Wake-up timer
 * @return	Nothing
 */
void WKT_IRQHandler(void)
{
	/* Clear WKT interrupt request */
	Chip_WKT_ClearIntStatus(LPC_WKT);

	/* Back to lightest sleep state for next WFI */
	LPC_PMU->PCON = PMU_PCON_PM_SLEEP;
	SCB->SCR = 0x0;
#if 0
	PeriodicWakeUP();
#endif
}

#include "lpc8xx_sd_adc.h"
#include "circular_buffer.h"
#include "simple_iir.h"

#define FEEDBACK_PIN       8  /*< Use PIO0_1 as feedback pin */
#define SD_ADC_DEBUG_PIN   13 /*< Use PIO0_15 as Analog Comparator output and CTIN_0 */

#if 0
/* SCT Interrupt Handler
 *
 * Required by the SD ADC module
 */
void SCT_IRQHandler(void) {
	sdadc_sct_irqhandler();
}

void sdadc_user_irqhandler(uint32_t adc_val) {
	cbuf_write(adc_val);
}
#endif


/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	int count = 0;
	SystemCoreClockUpdate();
	Board_Init();

	Board_LED_Set(0, true);

	/* Enable SysTick Timer */
	//SysTick_Config(SystemCoreClock / TICKRATE_HZ);

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

	init_done = true;

	/**/
#if 0
	cbuf_clear();

	/* Init the SD ADC pins */
	LPC_IOCON->PIO0[IOCON_PIO8]  = 0x80; /* Disable pull up / pull down resistor for feedback pin */
	//LPC_IOCON->PIO0[IOCON_PIO15] = 0x80; /* Disable pull up / pull down resistor for acmp_out_pin */
	sdadc_init(FEEDBACK_PIN, SD_ADC_DEBUG_PIN);

	NVIC_EnableIRQ(SCT_IRQn);

	//printf("%s\r\n", "Welcome to SD ADC Demo");

	while(1) {
		uint32_t adc_reading, dummy;
		if(cbuf_read(&adc_reading) >= 0) {
#if defined (ENABLE_IIR_LPF)
			uint32_t result = iir_lpf(adc_reading);
			printf("%d\r\n", result);
#else
			//printf("%d\r\n", adc_reading);
			dummy = adc_reading;
#endif
		}
	}

#endif

#if 1
		/* Alarm/wake timer as chip wakeup source */
		Chip_SYSCTL_EnablePeriphWakeup(SYSCTL_WAKEUP_WKTINT);

		/* Enable and reset WKT clock */
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_WKT);
		Chip_SYSCTL_PeriphReset(RESET_WKT);
#if 1
		/* Disable wakeup pad */
		Chip_PMU_ClearPowerDownControl(LPC_PMU, PMU_DPDCTRL_WAKEPAD | PMU_DPDCTRL_LPOSCDPDEN);

		/* Disable wakeup hysteresis by setting the bit (set to disable), enable 10KHz oscillator for all power down modes including deep power-down */
		Chip_PMU_SetPowerDownControl(LPC_PMU, PMU_DPDCTRL_WAKEUPPHYS | PMU_DPDCTRL_LPOSCEN | PMU_DPDCTRL_LPOSCDPDEN);
#endif
		/* Enable WKT interrupt */
		NVIC_EnableIRQ(WKT_IRQn);
		NVIC_SetPriority(WKT_IRQn, 1);

		/* Loop various tests */
#if 0
		/* Wakeup test with 10KHz clock, 5s wakeup, and PMU sleep state */
		WakeupTest(WKT_CLKSRC_10KHZ, 5, PMU_MCU_SLEEP);

		/* Wakeup test with 10KHz clock, 4s wakeup, and PMU deep sleep state */
		WakeupTest(WKT_CLKSRC_10KHZ, 4, PMU_MCU_DEEP_SLEEP);

		/* Wakeup test with 10KHz clock, 3s wakeup, and PMU MCU power down state */
		WakeupTest(WKT_CLKSRC_10KHZ, 3, PMU_MCU_POWER_DOWN);

		/* Wakeup test with 10KHz clock, 2s wakeup, and PMU MCU deep power down state */
		WakeupTest(WKT_CLKSRC_10KHZ, 5, PMU_MCU_DEEP_PWRDOWN);
#endif
#endif

	PeriodicTask();

	Board_LED_Set(0, false);

	while (1) {
		//Chip_WKT_LoadCount(LPC_WKT, Chip_WKT_GetClockRate(LPC_WKT) * 2);
		//Chip_PMU_Sleep(LPC_PMU, PMU_MCU_SLEEP);
		//Chip_PMU_Sleep(LPC_PMU, PMU_MCU_DEEP_SLEEP);
		WakeupTest(WKT_CLKSRC_10KHZ, 1, PMU_MCU_POWER_DOWN);
		//WakeupTest(WKT_CLKSRC_10KHZ, 1, PMU_MCU_DEEP_PWRDOWN);
		//__WFI();
	}

	return 0;
}
