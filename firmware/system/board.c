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

Maintainer:	Mehrdad Hessar, Ali Najafi
*/
#include <libraries/board.h>
#include <libraries/gpio.h>
#include <stdio.h>

/**#############################Vatiables and Types#############################**/
/* IQ Radio driver structure initialization */
const struct At86rf215_Radio_s At86rf215Radio =
{
    AT86RF215Init,
    AT86RF215GetStatus,
    AT86RF215SetModem,
    AT86RF215SetChannel,
    AT86RF215IsChannelFree,
    AT86RF215Random,
    AT86RF215RxSetConfig,
    AT86RF215TxSetConfig,
    AT86RF215CheckRfFrequency,
    AT86RF215GetTimeOnAir,
    AT86RF215Send,
    AT86RF215SetSleep,
    AT86RF215SetStby,
    AT86RF215RxSet,
    AT86RF215StartCad,
    AT86RF215ReadRssi,
    AT86RF215Write,
    AT86RF215Read,
    AT86RF215WriteBuffer,
    AT86RF215ReadBuffer,
    AT86RF215SetMaxPayloadLength
};

/* LoRa Radio driver structure initialization */
const struct sx1276_Radio_s sx1276Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength
};

DioIrqHandler *DioIrq2[] = { NULL, NULL,
                            NULL, NULL,
                            NULL, NULL };

/**#############################Functions#############################**/
void BoardPWRInit(void)
{
	PWR_D2_Set(PWR_D2_2_50);
	GpioWrite(&TinySDR.PWR_3P0, 0);
	GpioWrite(&TinySDR.LED_D1, 1);
}

void BoardInitMcu( void )
{
    SpiInit();

    BoardIoInit();
    BoardIoIrqInit(DioIrq2);
    TimerInit();
}

void delay_ns(uint32_t nsTime)
{
	nsTime = (uint32_t) ceil(nsTime / 1000.0);
	int i;
	for(i=0; i<nsTime; i++)
		__delay_cycles(1);
}

void delay_us(uint32_t usTime)
{
    int i;
    for (i=0; i<usTime; i++)
        __delay_cycles(CYCLES_us);
}

void BoardIoInit( void )
{
	/* Special Pins */
#ifdef PLATFORM_TINYSDR
	PJ->SEL0 &= ~BIT4;
	PJ->SEL1 &= ~BIT4;

	PJ->SEL0 &= ~BIT5;
    PJ->SEL1 &= ~BIT5;

    P6->SEL0 &= ~(BIT6 + BIT7);
    P6->SEL1 |= (BIT6 + BIT7);
#endif

/*#############################################################################################################################
//	IQ Radio
#############################################################################################################################*/
    /* IQ Radio SPI NSS pin */
    GpioInit(&AT86RF215.Spi.NSS, PIN_AT86RF215_SPI_NSS.portOut, PIN_AT86RF215_SPI_NSS.portIn, PIN_AT86RF215_SPI_NSS.portDir, PIN_AT86RF215_SPI_NSS.pin, 	PIN_AT86RF215_SPI_NSS.portNumber, 	PIN_OUTPUT, true);

    /* added for power test */
    GpioInit(&AT86RF215.Spi.MISO, PIN_AT86RF215_SPI_MISO.portOut, PIN_AT86RF215_SPI_MISO.portIn, PIN_AT86RF215_SPI_MISO.portDir, PIN_AT86RF215_SPI_MISO.pin, 	PIN_AT86RF215_SPI_MISO.portNumber, 	PIN_INPUT, false);
    GpioInit(&AT86RF215.Spi.MOSI, PIN_AT86RF215_SPI_MOSI.portOut, PIN_AT86RF215_SPI_MOSI.portIn, PIN_AT86RF215_SPI_MOSI.portDir, PIN_AT86RF215_SPI_MOSI.pin, 	PIN_AT86RF215_SPI_MOSI.portNumber, 	PIN_OUTPUT, false);
    GpioInit(&AT86RF215.Spi.SCLK, PIN_AT86RF215_SPI_SCLK.portOut, PIN_AT86RF215_SPI_SCLK.portIn, PIN_AT86RF215_SPI_SCLK.portDir, PIN_AT86RF215_SPI_SCLK.pin, 	PIN_AT86RF215_SPI_SCLK.portNumber, 	PIN_OUTPUT, false);

    /* IQ Radio Pins */
    GpioInit(&AT86RF215.Reset, PIN_AT86RF215_NRESET.portOut, PIN_AT86RF215_NRESET.portIn, PIN_AT86RF215_NRESET.portDir, PIN_AT86RF215_NRESET.pin, PIN_AT86RF215_NRESET.portNumber, PIN_OUTPUT, false);
    GpioInit(&AT86RF215.IRQ, PIN_AT86RF215_IRQ.portOut, PIN_AT86RF215_IRQ.portIn, PIN_AT86RF215_IRQ.portDir, PIN_AT86RF215_IRQ.pin, PIN_AT86RF215_IRQ.portNumber, PIN_INPUT, false);

//    PJSEL1 = 1;

#ifdef PLATFORM_TINYSDR
/*#############################################################################################################################
//	LoRa
#############################################################################################################################*/
    // LoRa Radio SPI NSS pin
	GpioInit(&SX1276.Spi.NSS, PIN_SX1276_SPI_NSS.portOut, PIN_SX1276_SPI_NSS.portIn, PIN_SX1276_SPI_NSS.portDir, PIN_SX1276_SPI_NSS.pin, PIN_SX1276_SPI_NSS.portNumber, PIN_OUTPUT, true);

	//added for power test
	GpioInit(&SX1276.Spi.MISO, PIN_SX1276_SPI_MISO.portOut, PIN_SX1276_SPI_MISO.portIn, PIN_SX1276_SPI_MISO.portDir, PIN_SX1276_SPI_MISO.pin, PIN_SX1276_SPI_MISO.portNumber, PIN_INPUT, false);
	GpioInit(&SX1276.Spi.MOSI, PIN_SX1276_SPI_MOSI.portOut, PIN_SX1276_SPI_MOSI.portIn, PIN_SX1276_SPI_MOSI.portDir, PIN_SX1276_SPI_MOSI.pin, PIN_SX1276_SPI_MOSI.portNumber, PIN_OUTPUT, false);
	GpioInit(&SX1276.Spi.SCLK, PIN_SX1276_SPI_SCLK.portOut, PIN_SX1276_SPI_SCLK.portIn, PIN_SX1276_SPI_SCLK.portDir, PIN_SX1276_SPI_SCLK.pin, PIN_SX1276_SPI_SCLK.portNumber, PIN_OUTPUT, false);

	// LoRa Radio Pins
	GpioInit(&SX1276.Reset, PIN_SX1276_NRESET.portOut, PIN_SX1276_NRESET.portIn, PIN_SX1276_NRESET.portDir, PIN_SX1276_NRESET.pin, PIN_SX1276_NRESET.portNumber, PIN_OUTPUT, true);
	GpioInit(&SX1276.DIO0, PIN_SX1276_DIO0.portOut, PIN_SX1276_DIO0.portIn, PIN_SX1276_DIO0.portDir, PIN_SX1276_DIO0.pin, PIN_SX1276_DIO0.portNumber, PIN_INPUT, false);

/*#############################################################################################################################
//	XXX
#############################################################################################################################*/
    //LEDs
    GpioInit(&TinySDR.LED_D1, PIN_LED_D1.portOut, PIN_LED_D1.portIn, PIN_LED_D1.portDir, PIN_LED_D1.pin, PIN_LED_D1.portNumber, PIN_OUTPUT, false);
    GpioInit(&TinySDR.LED_D2, PIN_LED_D2.portOut, PIN_LED_D2.portIn, PIN_LED_D2.portDir, PIN_LED_D2.pin, PIN_LED_D2.portNumber, PIN_OUTPUT, false);

    //D2 Voltage domain control
    GpioInit(&TinySDR.D2_CTL1, PIN_D2_CTL1.portOut, PIN_D2_CTL1.portIn, PIN_D2_CTL1.portDir, PIN_D2_CTL1.pin, PIN_D2_CTL1.portNumber, PIN_OUTPUT, false);
    GpioInit(&TinySDR.D2_CTL2, PIN_D2_CTL2.portOut, PIN_D2_CTL2.portIn, PIN_D2_CTL2.portDir, PIN_D2_CTL2.pin, PIN_D2_CTL2.portNumber, PIN_OUTPUT, false);
    GpioInit(&TinySDR.D2_CTL3, PIN_D2_CTL3.portOut, PIN_D2_CTL3.portIn, PIN_D2_CTL3.portDir, PIN_D2_CTL3.pin, PIN_D2_CTL3.portNumber, PIN_OUTPUT, false);

    //RF 900MHz switch
    GpioInit(&TinySDR.RF09_SW.A0, PIN_SW_A0.portOut, PIN_SW_A0.portIn, PIN_SW_A0.portDir, PIN_SW_A0.pin, PIN_SW_A0.portNumber, PIN_OUTPUT, false);
    GpioInit(&TinySDR.RF09_SW.A1, PIN_SW_A1.portOut, PIN_SW_A1.portIn, PIN_SW_A1.portDir, PIN_SW_A1.pin, PIN_SW_A1.portNumber, PIN_OUTPUT, false);

    //RF 900MHz PA
    GpioInit(&TinySDR.PA09.CPS, PIN_PA09_CPS.portOut, PIN_PA09_CPS.portIn, PIN_PA09_CPS.portDir, PIN_PA09_CPS.pin, PIN_PA09_CPS.portNumber, PIN_OUTPUT, false);
    GpioInit(&TinySDR.PA09.CTX, PIN_PA09_CTX.portOut, PIN_PA09_CTX.portIn, PIN_PA09_CTX.portDir, PIN_PA09_CTX.pin, PIN_PA09_CTX.portNumber, PIN_OUTPUT, false);

    //RF 2.4GHz PA
    GpioInit(&TinySDR.PA24.CRX, PIN_PA24_CRX.portOut, PIN_PA24_CRX.portIn, PIN_PA24_CRX.portDir, PIN_PA24_CRX.pin, PIN_PA24_CRX.portNumber, PIN_OUTPUT, false);
    GpioInit(&TinySDR.PA24.CTX, PIN_PA24_CTX.portOut, PIN_PA24_CTX.portIn, PIN_PA24_CTX.portDir, PIN_PA24_CTX.pin, PIN_PA24_CTX.portNumber, PIN_OUTPUT, false);
    GpioInit(&TinySDR.PA24.CPS, PIN_PA24_CPS.portOut, PIN_PA24_CPS.portIn, PIN_PA24_CPS.portDir, PIN_PA24_CPS.pin, PIN_PA24_CPS.portNumber, PIN_OUTPUT, false);


/*#############################################################################################################################
//	FPGA
#############################################################################################################################*/
    GpioInit(&TinySDR.FPGA.Reset,	PIN_FPGA_Reset.portOut, PIN_FPGA_Reset.portIn,	PIN_FPGA_Reset.portDir,	PIN_FPGA_Reset.pin,	PIN_FPGA_Reset.portNumber, 	PIN_OUTPUT, true);
    GpioInit(&TinySDR.FPGA.CFG0, 	PIN_FPGA_CFG0.portOut, 	PIN_FPGA_CFG0.portIn, 	PIN_FPGA_CFG0.portDir, 	PIN_FPGA_CFG0.pin, 	PIN_FPGA_CFG0.portNumber, 	PIN_OUTPUT, false);
    GpioInit(&TinySDR.FPGA.CFG1, 	PIN_FPGA_CFG1.portOut, 	PIN_FPGA_CFG1.portIn, 	PIN_FPGA_CFG1.portDir, 	PIN_FPGA_CFG1.pin, 	PIN_FPGA_CFG1.portNumber, 	PIN_OUTPUT, false);
    GpioInit(&TinySDR.FPGA.CFG2, 	PIN_FPGA_CFG2.portOut, 	PIN_FPGA_CFG2.portIn, 	PIN_FPGA_CFG2.portDir, 	PIN_FPGA_CFG2.pin, 	PIN_FPGA_CFG2.portNumber, 	PIN_OUTPUT, false);
    GpioInit(&TinySDR.FPGA.SPI.NSS, PIN_FPGA_CS.portOut, 	PIN_FPGA_CS.portIn, 	PIN_FPGA_CS.portDir, 	PIN_FPGA_CS.pin, 	PIN_FPGA_CS.portNumber, 	PIN_OUTPUT, true);

    //added for power measurement
//    GpioInit(&TinySDR.FPGA.SPI.MISO, 	PIN_FPGA_MISO.portOut, 	PIN_FPGA_MISO.portIn, 	PIN_FPGA_MISO.portDir, 	PIN_FPGA_MISO.pin, 	PIN_FPGA_MISO.portNumber, 	PIN_INPUT, 	false);
//    GpioInit(&TinySDR.FPGA.SPI.MOSI, 	PIN_FPGA_MOSI.portOut, 	PIN_FPGA_MOSI.portIn, 	PIN_FPGA_MOSI.portDir, 	PIN_FPGA_MOSI.pin, 	PIN_FPGA_MOSI.portNumber, 	PIN_OUTPUT, false);
//    GpioInit(&TinySDR.FPGA.SPI.SCLK, 	PIN_FPGA_SCLK.portOut, 	PIN_FPGA_SCLK.portIn, 	PIN_FPGA_SCLK.portDir, 	PIN_FPGA_SCLK.pin, 	PIN_FPGA_SCLK.portNumber, 	PIN_OUTPUT, false);

/*#############################################################################################################################
//	External Flash
#############################################################################################################################*/
    GpioInit(&TinySDR.FLASH.SPI.NSS, 	PIN_FLASH_CS.portOut, 	PIN_FLASH_CS.portIn,	PIN_FLASH_CS.portDir, 	PIN_FLASH_CS.pin, 	PIN_FLASH_CS.portNumber, 	PIN_OUTPUT,	true);

//    GpioInit(&TinySDR.FLASH.SPI.MOSI, PIN_FLASH_MOSI.portOut, PIN_FLASH_MOSI.portIn,	PIN_FLASH_MOSI.portDir, PIN_FLASH_MOSI.pin, PIN_FLASH_MOSI.portNumber, 	PIN_OUTPUT,	false);
//    GpioInit(&TinySDR.FLASH.SPI.MISO, PIN_FLASH_MISO.portOut, PIN_FLASH_MISO.portIn,	PIN_FLASH_MISO.portDir, PIN_FLASH_MISO.pin, PIN_FLASH_MISO.portNumber, 	PIN_INPUT,	false);
//#ifdef MX25R6435F
//    GpioInit(&TinySDR.FLASH.SPI.SCLK, PIN_FLASH_SCLK.portOut, PIN_FLASH_SCLK.portIn,	PIN_FLASH_SCLK.portDir, PIN_FLASH_SCLK.pin, PIN_FLASH_SCLK.portNumber, 	PIN_OUTPUT,	false);
//#endif

/*#############################################################################################################################
//	XXX
#############################################################################################################################*/
    //Power
    GpioInit(&TinySDR.PWR_3P0, PIN_PWR_3P0.portOut, PIN_PWR_3P0.portIn, PIN_PWR_3P0.portDir, PIN_PWR_3P0.pin, PIN_PWR_3P0.portNumber, PIN_OUTPUT, false);

    //External interface
//    GpioInit(&AT86RF215.IF_I2C.SDA, PIN_IF_I2C_SDA.portOut, PIN_IF_I2C_SDA.portIn, PIN_IF_I2C_SDA.portDir, PIN_IF_I2C_SDA.pin, PIN_IF_I2C_SDA.portNumber, PIN_OUTPUT, true);
//    GpioInit(&AT86RF215.IF_I2C.SCL, PIN_IF_I2C_SCL.portOut, PIN_IF_I2C_SCL.portIn, PIN_IF_I2C_SCL.portDir, PIN_IF_I2C_SCL.pin, PIN_IF_I2C_SCL.portNumber, PIN_OUTPUT, true)

#endif

}

void BoardIoIrqInit( DioIrqHandler **irqHandlers )
{
    GpioSetInterrupt(&AT86RF215.IRQ, IRQ_RISING_EDGE, irqHandlers[0]);

    /* LoRa IRQ */
    GpioSetInterrupt(&SX1276.DIO0, IRQ_RISING_EDGE, irqHandlers[1]);

    /*
     * Enabling SRAM Bank Retention
     * Required for GPIO IRQ.
     * */
    MAP_SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);
}


bool AT86RF215CheckRfFrequency( uint32_t frequency )
{
    /* Implement check. Currently all frequencies are supported */
    return true;
}

void setSwitchMode(At86rf215_RadioModems_t modem)
{
    if(modem == MODEM_BR_RX)
    {
        GpioWrite(&TinySDR.RF09_SW.A0, 1);
        GpioWrite(&TinySDR.RF09_SW.A1, 0);
    }
    else if (modem == MODEM_BR_TX)
    {
    	GpioWrite(&TinySDR.RF09_SW.A0, 1);
    	GpioWrite(&TinySDR.RF09_SW.A1, 1);
    }
    else if (modem == MODEM_09)
    {
    	GpioWrite(&TinySDR.RF09_SW.A0, 0);
    	GpioWrite(&TinySDR.RF09_SW.A1, 0);
    }
    else if (modem == MODEM_24)
    {
    	GpioWrite(&TinySDR.RF09_SW.A0, 0);
    	GpioWrite(&TinySDR.RF09_SW.A1, 1);
    }
}

void setPA(PA_Types_t type, PA_Settings_t setting)
{
	if (type == PA_900)
	{
		if ((setting == PA_RX_Bypass) || (setting == PA_TX_Bypass))
		{
			/* TRX Bypass */
			GpioWrite(&TinySDR.PA09.CPS, 0);
			GpioWrite(&TinySDR.PA09.CTX, 0);
		}
		else if (setting == PA_RX_LNA)
		{
			/* RX LNA */
			GpioWrite(&TinySDR.PA09.CPS, 1);
			GpioWrite(&TinySDR.PA09.CTX, 0);
		}
		else if ((setting == PA_TX_High) || (setting == PA_TX_Low))
		{
			/* PA TX */
			GpioWrite(&TinySDR.PA09.CPS, 0);
			GpioWrite(&TinySDR.PA09.CTX, 1);
		}
		else
		{
			/* TRX Bypass */
			GpioWrite(&TinySDR.PA09.CPS, 0);
			GpioWrite(&TinySDR.PA09.CTX, 0);
		}
	}
	else
	{
		if (setting == PA_TX_Bypass)
		{
			/* TX Bypass */
			GpioWrite(&TinySDR.PA24.CPS, 1);
			GpioWrite(&TinySDR.PA24.CRX, 0);
			GpioWrite(&TinySDR.PA24.CTX, 1);
		}
		else if (setting == PA_RX_Bypass)
		{
			/* RX Bypass */
			GpioWrite(&TinySDR.PA24.CPS, 1);
			GpioWrite(&TinySDR.PA24.CRX, 1);
			GpioWrite(&TinySDR.PA24.CTX, 0);
		}
		else if (setting == PA_RX_LNA)
		{
			/* RX LNA */
			GpioWrite(&TinySDR.PA24.CPS, 0);
			GpioWrite(&TinySDR.PA24.CRX, 1);
			GpioWrite(&TinySDR.PA24.CTX, 0);
		}
		else if ((setting == PA_TX_High) || (setting == PA_TX_Low))
		{
			/* PA TX */
			GpioWrite(&TinySDR.PA24.CPS, 0);
			GpioWrite(&TinySDR.PA24.CRX, 0);
			GpioWrite(&TinySDR.PA24.CTX, 1);
		}
		else
		{
			/* sleep */
			GpioWrite(&TinySDR.PA24.CPS, 0);
			GpioWrite(&TinySDR.PA24.CRX, 0);
			GpioWrite(&TinySDR.PA24.CTX, 0);

		}
	}
}
