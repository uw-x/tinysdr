/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements the generic driver for power management

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#include <libraries/power.h>

/**########################Functions############################**/
void PWR_D2_Set(uint8_t volt)
{
	uint8_t ctl1;
	uint8_t ctl2;
	uint8_t ctl3;

	switch (volt)
	{
		case PWR_D2_0_00:
			ctl1 = 0; ctl2 = 0; ctl3 = 0;
			break;
		case PWR_D2_1_00:
			ctl1 = 1; ctl2 = 0; ctl3 = 0;
			break;
		case PWR_D2_1_40:
			ctl1 = 0; ctl2 = 1; ctl3 = 0;
			break;
		case PWR_D2_1_60:
			ctl1 = 1; ctl2 = 1; ctl3 = 0;
			break;
		case PWR_D2_1_85:
			ctl1 = 0; ctl2 = 0; ctl3 = 1;
			break;
		case PWR_D2_2_00:
			ctl1 = 1; ctl2 = 0; ctl3 = 1;
			break;
		case PWR_D2_2_50:
			ctl1 = 0; ctl2 = 1; ctl3 = 1;
			break;
		case PWR_D2_3_00:
			ctl1 = 1; ctl2 = 1; ctl3 = 1;
			break;
		default:
			ctl1 = 0; ctl2 = 0; ctl3 = 0;
			break;
	}

	GpioWrite(&TinySDR.D2_CTL1, ctl1);
	GpioWrite(&TinySDR.D2_CTL2, ctl2);
	GpioWrite(&TinySDR.D2_CTL3, ctl3);
}
