/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: TinySDR Utils

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#ifndef _UTILS_H_
#define _UTILS_H_
#include <settings.h>
#include <stdio.h>

/**#############################Vatiables and Types#############################**/
#define ERROR_Modem			1
#define ERROR_Frequency		2

/**#############################Functions#############################**/
void delay_ms(uint32_t msTime);
void bitSet(uint8_t *value, uint8_t bit);
void bitClear(uint8_t *value, uint8_t bit);


/**#############################Static Functions#############################**/
static void PrintError(uint32_t error)
{
	switch(error)
	{
		case ERROR_Modem:
			printf("Error: Wrong Modem!");
			break;
		case(ERROR_Frequency):
			printf("Error: Wrong Frequency!");
			break;
		default:
			break;
	}
}

#endif
