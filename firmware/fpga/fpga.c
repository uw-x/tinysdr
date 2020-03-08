/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Generic FPGA functionality implementations on TinySDR

License: see LICENSE.TXT file include in the project

Maintainer:	Mehrdad Hessar, Ali Najafi
*/
#include "fpga.h"

/**########################External Functions############################**/
void fpgaReset(void)
{
    GpioWrite(&TinySDR.FPGA.Reset, true);
    /* Wait typical time of timer TR1. */
    delay_us(300);
    /* Set RESET pin to 0 */
    GpioWrite(&TinySDR.FPGA.Reset, false);
    /* Wait 10 us */
    delay_ms(10);
    GpioWrite(&TinySDR.FPGA.Reset, true);
    delay_ms(1000);
}

void fpgaSetConfig(uint8_t config)
{
	GpioWrite(&TinySDR.FPGA.CFG0, 0);
	GpioWrite(&TinySDR.FPGA.CFG1, 1);
	GpioWrite(&TinySDR.FPGA.CFG2, 0);
}

uint8_t fpgaWrite(uint8_t data)
{
	uint8_t res;
	/* CS = 0 */
	GpioWrite(&TinySDR.FPGA.SPI.NSS, 0);
	fpgaSpiInOut(0b00000000);
	res = fpgaSpiInOut(data);
	/* CS = 1 */
	GpioWrite(&TinySDR.FPGA.SPI.NSS, 1);
	return res;
}

void fpgaWriteBuffer(uint8_t *buffer, uint8_t size)
{
    uint8_t i;
    GpioWrite(&TinySDR.FPGA.SPI.NSS, 0);
    fpgaSpiInOut(0b00000000);

    for(i = 0; i<size; i++)
    {
    	fpgaSpiInOut(buffer[i]);
    }

    GpioWrite(&TinySDR.FPGA.SPI.NSS, 1);
}

//uint8_t fpgaRead(void)
//{
//    uint8_t data;
//    AT86RF215ReadBuffer(&data, 1);
//    return data;
//}

void fpgaReadBuffer(uint8_t *buffer, uint8_t size, uint8_t data)
{
//    uint8_t ii;
    GpioWrite(&TinySDR.FPGA.SPI.NSS, 0);

//    for(ii=0; ii<size-1; ii++)
//    {
//        buffer[ii] = fpgaSpiInOut(0b00000000);
//    }
//    buffer[ii] = fpgaSpiInOut(data);
//    fpgaSpiInOut(data);

    buffer[0] = fpgaSpiInOut(0b00000000);
    buffer[1] = fpgaSpiInOut(data);
//    buffer[2] = fpgaSpiInOut(0x02);
//    buffer[1] = fpgaSpiInOut(0b10110001);


    GpioWrite(&TinySDR.FPGA.SPI.NSS, 1);
//    printf("FPGA SPI IN: %x, %x\n", buffer[0], buffer[1]);
}

