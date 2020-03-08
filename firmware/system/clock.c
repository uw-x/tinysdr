/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements the generic clock driver

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#include <libraries/clock.h>

/**########################Internal functions############################**/
//uint32_t ClockGetDivider(uint32_t clockSource, uint32_t freq)

/**#######################External functions######################**/
void ClockInit( void )
{
	TinySDR.Clock.LFX = false;
	TinySDR.Clock.HFX = false;
	TinySDR.Clock.LFX_Freq = 0;
	TinySDR.Clock.HFX_Freq = 0;
	TinySDR.Clock.Source = 0;
	TinySDR.Clock.MCLK = 0;
	TinySDR.Clock.ACLK = 0;
	TinySDR.Clock.HSMCLK = 0;
	TinySDR.Clock.SMCLK = 0;
	TinySDR.Clock.BCLK = 0;

    /* Initializing the clock source as follows:
     *      MCLK = MODOSC = 24MHz
     *      ACLK = REFO/2 = 16kHz
     *      HSMCLK = DCO/2 = 1.5Mhz
     *      SMCLK = DCO/4 = 12MHz
     *      BCLK  = REFO = 32kHz
     */
    ClockSet(CLK_MCLK, 		CS_MODOSC_SELECT, 	CS_CLOCK_DIVIDER_1,	true);
    ClockSet(CLK_ACLK, 		CS_REFOCLK_SELECT, 	CS_CLOCK_DIVIDER_1, true);
    ClockSet(CLK_HSMCLK,	CS_DCOCLK_SELECT, 	CS_CLOCK_DIVIDER_2, true);
    ClockSet(CLK_SMCLK, 	CS_MODOSC_SELECT, 	CS_CLOCK_DIVIDER_2, true);
    ClockSet(CLK_BCLK, 		CS_REFOCLK_SELECT,	CS_CLOCK_DIVIDER_1, true);
    printf("ACLK: %d\n", CS_getACLK());
    printf("SMCLK: %d\n", CS_getSMCLK());
}

void ClockEnableHFXT(void)
{
	/* Configuring pins for peripheral/crystal usage and LED for output */
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PIN_CLK_HFXIN.portNumber,
			PIN_CLK_HFXIN.pin | PIN_CLK_HFXOUT.pin, GPIO_PRIMARY_MODULE_FUNCTION);

	MAP_CS_setExternalClockSourceFrequency(32000,48000000);

	/* Starting HFXT in non-bypass mode without a timeout. Before we start
	* we have to change VCORE to 1 to support the 48MHz frequency */
	MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
	MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
	MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
	CS_startHFXT(false);
	TinySDR.Clock.HFX = true;
	ClockSetSource(CS_HFXTCLK_SELECT);
	TinySDR.Clock.HFX_Freq = 48000000;
}

void ClockEnableLFXT(void)
{
    /* Configuring pins for peripheral/crystal usage and LED for output */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PIN_CLK_LFXIN.portNumber,
            PIN_CLK_LFXIN.pin | PIN_CLK_LFXOUT.pin, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_CS_setExternalClockSourceFrequency(32000,48000000);

    /* Starting LFXT in non-bypass mode without a timeout. */
    CS_startLFXT(false);
    TinySDR.Clock.LFX = true;
    ClockSetSource(CS_LFXTCLK_SELECT);
    TinySDR.Clock.LFX_Freq = 32000;
}

void ClockSetSource(uint32_t source)
{
	TinySDR.Clock.Source = source;
}

void ClockSet(Clock_Types_t type, uint32_t clockSource, uint32_t divider, bool enable)
{
	switch (type)
	{
		case CLK_MCLK:
			if (enable == false)
			{
				TinySDR.Clock.MCLK = 0;
				MAP_CS_disableClockRequest(CS_MCLK);
			}
			else
			{
				MAP_CS_initClockSignal(CS_MCLK, clockSource, divider);
				TinySDR.Clock.MCLK = CS_getMCLK();
			}
			break;
		case CLK_ACLK:
			if (enable == false)
			{
				TinySDR.Clock.ACLK = 0;
				MAP_CS_disableClockRequest(CS_ACLK);
			}
			else
			{
				MAP_CS_initClockSignal(CS_ACLK, clockSource, divider);
				TinySDR.Clock.ACLK = CS_getACLK();
			}
			break;
		case CLK_HSMCLK:
			if (enable == false)
			{
				TinySDR.Clock.HSMCLK = 0;
				MAP_CS_disableClockRequest(CS_HSMCLK);
			}
			else
			{
				MAP_CS_initClockSignal(CS_HSMCLK, clockSource, divider);
				TinySDR.Clock.HSMCLK = CS_getHSMCLK();
			}
			break;
		case CLK_SMCLK:
			if (enable == false)
			{
				TinySDR.Clock.SMCLK = 0;
				MAP_CS_disableClockRequest(CS_SMCLK);
			}
			else
			{
				MAP_CS_initClockSignal(CS_SMCLK, clockSource, divider);
				TinySDR.Clock.SMCLK = CS_getSMCLK();
			}
			break;
		case CLK_BCLK:
			if (enable == false)
			{
				TinySDR.Clock.BCLK = 0;
				MAP_CS_disableClockRequest(CS_BCLK);
			}
			else
			{
				MAP_CS_initClockSignal(CS_BCLK, clockSource, divider);
				TinySDR.Clock.BCLK = CS_getBCLK();
			}
			break;
		default:
			break;
	}
}

//uint32_t ClockGetDivider(uint32_t clockSource, uint32_t freq)
//{
//	uint32_t sourceFreq;
//	uint32_t divider;
//
//	if (TinySDR.Clock.Source == CS_HFXTCLK_SELECT)
//	{
//		sourceFreq = TinySDR.Clock.HFX_Freq;
//	}
//	else if (TinySDR.Clock.Source == CS_LFXTCLK_SELECT)
//	{
//		sourceFreq = TinySDR.Clock.LFX_Freq;
//	}
//	else if (TinySDR.Clock.Source == CS_DCOCLK_SELECT)
//	{
//		//TODO
//		sourceFreq = 0;
//	}
//
//	divider = source/freq;
//	return divider;
//}










































































