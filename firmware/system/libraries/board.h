/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: TinySDR board functionality implementations

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#ifndef _SYSTEM_BOARD_H_
#define _SYSTEM_BOARD_H_

#include <at86rf215/at86rf215.h>
#include <sx1276/sx1276.h>
#include <libraries/spi.h>
#include <libraries/timer.h>
#include <libraries/power.h>
#include <libraries/gpio.h>
#include <settings.h>

/**#############################Vatiables and Types#############################**/
typedef struct
{
	Gpio_t		Reset;
	Gpio_t		CFG0;
	Gpio_t		CFG1;
	Gpio_t		CFG2;
	Gpio_t		V_FPGA_EN;	//same as LED1 in this board
	Spi_t		SPI;
}FPGA_t;

typedef struct
{
	Spi_t		SPI;
}FLASH_t;

typedef struct
{
	Gpio_t		A0;
	Gpio_t		A1;
	Switch_t	RF09_Mode;
}RF_SW_t;

typedef struct RF_PA_s
{
	Gpio_t		CPS;
	Gpio_t		CTX;
	Gpio_t		CRX;
	Gpio_t		CSD;
}RF_PA_t;

typedef struct Clock_s
{
	bool		LFX;
	bool		HFX;
	uint32_t	Source;
	uint32_t	HFX_Freq;
	uint32_t	LFX_Freq;

	uint32_t	MCLK;
	uint32_t	ACLK;
	uint32_t	HSMCLK;
	uint32_t	SMCLK;
	uint32_t	BCLK;
}Clock_t;

typedef struct Board_s
{
	Gpio_t              LED_D1;
	Gpio_t              LED_D2;

    FPGA_t				FPGA;
    FLASH_t				FLASH;

    Gpio_t              D2_CTL1;
    Gpio_t              D2_CTL2;
    Gpio_t              D2_CTL3;
    Gpio_t				PWR_3P0;

    RF_SW_t				RF09_SW;
	RF_PA_t				PA09;
	RF_PA_t				PA24;

	I2C_t				IF_I2C;
	Clock_t				Clock;
}Board_t;


typedef enum
{
    PWR_D2_0_00 = 0,
	PWR_D2_1_00,
	PWR_D2_1_40,
	PWR_D2_1_60,
	PWR_D2_1_85,
	PWR_D2_2_00,
	PWR_D2_2_50,
	PWR_D2_3_00
}PWR_D2_t;

extern AT86RF215_t AT86RF215;
extern SX1276_t SX1276;
extern Board_t TinySDR;

/**#############################External Functions#############################**/\
void BoardPWRInit(void);
void BoardInitMcu( void );
void delay_ns(uint32_t nsTime);

void delay_us(uint32_t msTime);
void delay_ms(uint32_t msTime);

void BoardIoInit( void );
void BoardIoIrqInit( DioIrqHandler **irqHandlers );
void AT86RF215SetAntSw( uint8_t rxTx );
bool AT86RF215CheckRfFrequency( uint32_t frequency );
void setSwitchMode(At86rf215_RadioModems_t modem);
void setPA(PA_Types_t pa, PA_Settings_t setting);



#endif
