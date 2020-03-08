/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements the generic Timer driver

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#include <libraries/timer.h>

/**########################Variables############################**/
#define TIMER_A_CYCLES_PER_mS       1000
#define TIMER_FPGA_ON               20*TIMER_A_CYCLES_PER_mS

#define TIMER_B_CYCLES_PER_mS       1000
#define TIMER_FPGA_OFF              20*TIMER_B_CYCLES_PER_mS

Timer_A_UpModeConfig timer_A1 = {0};

/**########################External Functions############################**/
void TimerInit(void)
{
	//Timer A1_0
	timer_A1.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
	timer_A1.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
	timer_A1.timerPeriod = 0;
	timer_A1.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
	timer_A1.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
	timer_A1.timerClear = TIMER_A_DO_CLEAR;

	MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &timer_A1);

	/* Enabling interrupts */
	MAP_Interrupt_enableInterrupt(INT_TA1_0);


//    Timer_A_initCompareModeParam initCompParam_A = {0};
//    initCompParam_A.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
//    initCompParam_A.compareInterruptEnable =
//        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
//    initCompParam_A.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
//    initCompParam_A.compareValue = TIMER_FPGA_ON;
//    Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam_A);


//    //Second timer, turn FPGA off
//    //Start timer in continuous mode sourced by SMCLK
//    Timer_B_initContinuousModeParam initContParam = {0};
//    initContParam.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
//    initContParam.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_16;
//    initContParam.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
//    initContParam.timerClear = TIMER_B_DO_CLEAR;
//    initContParam.startTimer = false;
//    Timer_B_initContinuousMode(TIMER_B0_BASE, &initContParam);
//
//    //Initiaze compare mode
//    Timer_B_clearCaptureCompareInterrupt(TIMER_B0_BASE,
//                                         TIMER_B_CAPTURECOMPARE_REGISTER_0);
//
//    Timer_B_initCompareModeParam initCompParam = {0};
//    initCompParam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_0;
//    initCompParam.compareInterruptEnable =
//        TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
//    initCompParam.compareOutputMode = TIMER_B_OUTPUTMODE_OUTBITVALUE;
//    initCompParam.compareValue = TIMER_FPGA_OFF;
//    Timer_B_initCompareMode(TIMER_B0_BASE, &initCompParam);

}


void TimerA1SetStart(uint16_t time_ms)
{
	uint32_t compVal;
	compVal = ((float) CS_getACLK()/timer_A1.clockSourceDivider)*((float) time_ms/1000.0);
	MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, (uint16_t) compVal);
	MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

void TimerA1Start(uint16_t time_ms)
{
	MAP_Timer_A_clearTimer(TIMER_A1_BASE);
	MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

void TimerA1Stop(void)
{
	MAP_Timer_A_stopTimer(TIMER_A1_BASE);
}
/**########################ISR############################**/
void TA1_0_IRQHandler(void)
{
	MAP_Interrupt_disableMaster();
//	MAP_Interrupt_enableMaster
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);

	MAP_Timer_A_stopTimer(TIMER_A1_BASE);
	MAP_Timer_A_clearTimer(TIMER_A1_BASE);

//	MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN3);
	GpioToggle(&TinySDR.LED_D2);

//	printf("t INT, S: %x\n", MCU_State);

	if (MCU_State == MCU_STATE_BR_RX_WAIT)
	{
		br_miss++;
		if (br_miss > OTA_BR_MISS)
		{
			MCU_State = MCU_STATE_BR_INIT;
			br_miss = 0;
		}
	}

	MCU_State = MCU_STATE_BR_RX_INIT;

	MAP_Interrupt_enableMaster();
}
