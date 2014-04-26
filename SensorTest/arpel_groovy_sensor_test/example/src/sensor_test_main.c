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

//#define USE_DEEP_POWER_DOWN 1

#include "lpc8xx_sd_adc.h"
#include "circular_buffer.h"
#include "simple_iir.h"

#define FEEDBACK_PIN       8  /*< Use PIO0_1 as feedback pin */
#define SD_ADC_DEBUG_PIN   13 /*< Use PIO0_15 as Analog Comparator output and CTIN_0 */

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
  uint16_t temp;
  uint8_t status;          //
  uint8_t nbCommands;
  uint8_t lastCommand;
  uint32_t supplyV; // Supply voltage
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
void ADCInit(void){
	cbuf_clear();

	/* Init the SD ADC pins */
	LPC_IOCON->PIO0[IOCON_PIO8]  = 0x80; /* Disable pull up / pull down resistor for feedback pin */
	//LPC_IOCON->PIO0[IOCON_PIO15] = 0x80; /* Disable pull up / pull down resistor for acmp_out_pin */
	sdadc_init(FEEDBACK_PIN, SD_ADC_DEBUG_PIN);

	NVIC_EnableIRQ(SCT_IRQn);

	sdadc_suspend();
}

void ADCReadVCC(void){
	uint32_t adc_reading;

	sdadc_resume();

	while(cbuf_read(&adc_reading) < 0){}

	if(cbuf_read(&adc_reading) >= 0) {
	#if defined (ENABLE_IIR_LPF)
		staticpayload.supplyV = iir_lpf(adc_reading);
	#else
		staticpayload.supplyV = adc_reading;
	#endif
	}

	sdadc_suspend();
}

void PeriodicTask(void)
{
	for (sensor_counter = 0; sensor_counter < 4; sensor_counter++) {
		if(OW_sensorList[sensor_counter].l == 0)
			continue;

		if (ow_read_temp(&t_val, &OW_sensorList[sensor_counter])) {
			//printf("Error Sensor %d\n", (sensor_counter + 1));
		} else {
			temperature[sensor_counter] = (t_val * 5); //read uint to int
		}
	}

	/* ID */
	staticpayload.id = Chip_PMU_ReadGPREG(LPC_PMU, 0) + 1;
	Chip_PMU_WriteGPREG(LPC_PMU, 0, staticpayload.id);

	/* Temp */
	staticpayload.temp = temperature[0];

	ADCReadVCC();

	RF_AirSend(&staticpayload);
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void PeriodicWakeUP(void)
{
	/* LED will toggle state on wakeup event */
	Board_LED_Set(0, true);

	sysTick++;

	PeriodicTask();

	Board_LED_Set(0, false);
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
	msTicks++;
}

void delay_ms(uint32_t ms) {
	uint32_t now = msTicks;
	while ((msTicks-now) < ms);
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
#ifndef USE_DEEP_POWER_DOWN
	PeriodicWakeUP();
#endif
}


#if 1
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


static void WakeupTest(WKT_CLKSRC_T clkSrc, uint32_t timeoutInSecs, CHIP_PMU_MCUPOWER_T powerTest)
{
	/* 10KHz clock source */
	Chip_WKT_SetClockSource(LPC_WKT, clkSrc);
#ifndef USE_DEEP_POWER_DOWN
	/* Setup for wakeup in 5s */
	Chip_WKT_LoadCount(LPC_WKT, Chip_WKT_GetClockRate(LPC_WKT) * timeoutInSecs);

	/* We can optionally call Chip_SYSCTL_SetDeepSleepPD() to power down the BOD and WDT if we aren't using them in deep sleep modes. */
	Chip_SYSCTL_SetDeepSleepPD(SYSCTL_DEEPSLP_BOD_PD | SYSCTL_DEEPSLP_WDTOSC_PD);

	/* We should call Chip_SYSCTL_SetWakeup() to setup any peripherals we want to power back up on wakeup. For this example, we'll power back up the IRC, FLASH, the system oscillator, and the PLL */
	Chip_SYSCTL_SetWakeup(~(SYSCTL_SLPWAKE_IRC_PD | SYSCTL_SLPWAKE_IRCOUT_PD | SYSCTL_SLPWAKE_FLASH_PD | SYSCTL_SLPWAKE_SYSOSC_PD | SYSCTL_SLPWAKE_SYSPLL_PD));
#endif
#ifdef USE_DEEP_POWER_DOWN
	LPC_PMU->PCON &= ~PMU_PCON_DPDFLAG;
	LPC_PMU->PCON |= PMU_PCON_SLEEPFLAG;

	SCB->SCR |= (1UL << SCB_SCR_SLEEPDEEP_Pos);
	LPC_PMU->PCON = PMU_PCON_PM_DEEPPOWERDOWN;

	Chip_WKT_LoadCount(LPC_WKT, Chip_WKT_GetClockRate(LPC_WKT) * timeoutInSecs);

	/* Enter sleep mode */
	__WFI();

#else
	/* Tell PMU to go to sleep */
	Chip_PMU_Sleep(LPC_PMU, powerTest);

	/* Power anything back up here that isn't powered up on wakeup. The example code below powers back up the BOD and WDT oscillator, which weren't setup to power up in the Chip_SYSCTL_SetWakeup() function. */
	Chip_SYSCTL_SetDeepSleepPD(0);

	/* Will return here after wakeup and WKT IRQ, LED should be on */
	Chip_WKT_Stop(LPC_WKT);
#endif
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

	/* Enable SysTick Timer */
	//SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	if(comingFromDeepPowerDown){
		LPC_PMU->PCON &= ~PMU_PCON_DPDFLAG;
		LPC_PMU->PCON |= PMU_PCON_SLEEPFLAG;

		/* Clear WKT interrupt request */
		Chip_WKT_ClearIntStatus(LPC_WKT);
		Chip_WKT_Stop(LPC_WKT);

		/* Back to lightest sleep state for next WFI */
		LPC_PMU->PCON = PMU_PCON_PM_SLEEP;
		SCB->SCR = 0x0;

		/* Power anything back up here that isn't powered up on wakeup. The example code below powers back up the BOD and WDT oscillator, which weren't setup to power up in the Chip_SYSCTL_SetWakeup() function. */
		Chip_SYSCTL_SetDeepSleepPD(0);
	}

	Board_LED_Set(0, true);

	/* Power UP the sensor */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0 , 13);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 13, true);

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

	ADCInit();

	PeriodicTask();


	/* Alarm/wake timer as chip wakeup source */
	Chip_SYSCTL_EnablePeriphWakeup(SYSCTL_WAKEUP_WKTINT);

	/* Enable and reset WKT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_WKT);
	Chip_SYSCTL_PeriphReset(RESET_WKT);

	/* Disable wakeup hysteresis */
	Chip_PMU_ClearPowerDownControl(LPC_PMU,  PMU_DPDCTRL_WAKEUPPHYS);

	/* Disable wakeup pad by setting the bit (set to disable), enable 10KHz oscillator for all power down modes including deep power-down */
	Chip_PMU_SetPowerDownControl(LPC_PMU, PMU_DPDCTRL_WAKEPAD | PMU_DPDCTRL_LPOSCEN | PMU_DPDCTRL_LPOSCDPDEN);

	Chip_WKT_ClearIntStatus(LPC_WKT);
	NVIC_ClearPendingIRQ(WKT_IRQn);

	/* Enable WKT interrupt */
	NVIC_EnableIRQ(WKT_IRQn);
	NVIC_SetPriority(WKT_IRQn, 1);

#ifdef USE_DEEP_POWER_DOWN

	Board_LED_Set(0, false);

	WakeupTest(WKT_CLKSRC_10KHZ, 5, PMU_MCU_DEEP_PWRDOWN);
#else
	while (1) {
		//Chip_WKT_LoadCount(LPC_WKT, Chip_WKT_GetClockRate(LPC_WKT) * 2);
		//Chip_PMU_Sleep(LPC_PMU, PMU_MCU_SLEEP);
		//Chip_PMU_Sleep(LPC_PMU, PMU_MCU_DEEP_SLEEP);
		WakeupTest(WKT_CLKSRC_10KHZ, 1, PMU_MCU_SLEEP);
		//WakeupTest(WKT_CLKSRC_10KHZ, 1, PMU_MCU_POWER_DOWN);
		//WakeupTest(WKT_CLKSRC_10KHZ, 1, PMU_MCU_DEEP_PWRDOWN);
		//__WFI();
	}
#endif
	return 0;
}
