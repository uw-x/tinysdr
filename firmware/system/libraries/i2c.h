/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements the generic i2c driver

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#ifndef _SYSTEM_I2C_H_
#define _SYSTEM_I2C_H_

#include <driverlib.h>
#include <settings.h>
#include <utils.h>
#include <stdio.h>

/**########################Variables and Types############################**/
/* SPI object type definition */
typedef struct I2C_s
{
    Gpio_t SDA;
    Gpio_t SCL;
}I2C_t;


/**########################Functions############################**/
void I2C_init(void);
void I2C_send8(uint32_t moduleInstance, uint8_t txData);
uint16_t I2C_EUSCI_B_Receive16(uint32_t moduleInstance);
void I2C_EUSCI_B_Write16(uint32_t moduleInstance, uint8_t addr, uint16_t txData);
void I2C_EUSCI_B_Write8(uint32_t moduleInstance, uint8_t addr, uint8_t txData);

#endif
