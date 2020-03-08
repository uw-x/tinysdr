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
#include <libraries/i2c.h>


/**########################Variables and Types############################**/
/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};

/**########################Functions############################**/
void I2C_init(void)
{
	/* Pin Assignment */
	GPIO_setAsPeripheralModuleFunctionInputPin(PIN_IF_I2C_SDA.portNumber,
				PIN_IF_I2C_SDA.pin + PIN_IF_I2C_SCL.pin, MCP9808_I2C_GPIO_FUNC);

	/* Initializing I2C Master to SMCLK at 100khz with no autostop */
	I2C_initMaster(MCP9808_I2C_Module, &i2cConfig);
	/* Specify slave address */
	I2C_setSlaveAddress(MCP9808_I2C_Module, MCP9808_SLAVE_ADDR);
	/* Set Master in transmit mode */
	I2C_setMode(MCP9808_I2C_Module, EUSCI_B_I2C_TRANSMIT_MODE);
	/* Enable I2C Module to start operations */
	I2C_enableModule(MCP9808_I2C_Module);
	/* Enable and clear the interrupt flag */
	I2C_clearInterruptFlag(MCP9808_I2C_Module,
			EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);

	Interrupt_enableInterrupt(MCP9808_I2C_INT);
	/* Making sure the last transaction has been completely sent out */
	while (I2C_masterIsStopSent(MCP9808_I2C_Module) == EUSCI_B_I2C_SENDING_STOP);

}

void I2C_send8(uint32_t moduleInstance, uint8_t txData)
{
	I2C_setMode(moduleInstance, EUSCI_B_I2C_TRANSMIT_MODE);

	/* original */
	I2C_masterSendSingleByte(moduleInstance, txData);

	while (I2C_masterIsStopSent(moduleInstance));
}

uint16_t I2C_EUSCI_B_Receive16(uint32_t moduleInstance)
{
	uint16_t data1;
	uint16_t data2;
	uint16_t data;

	//Set USCI in Receive mode
	BITBAND_PERI(EUSCI_B_CMSIS(moduleInstance)->CTLW0, EUSCI_B_CTLW0_TR_OFS) = 0;
	//Send start
	EUSCI_B_CMSIS(moduleInstance)->CTLW0 |= (EUSCI_B_CTLW0_TXSTT);
	while(!(EUSCI_B_CMSIS(moduleInstance)->CTLW0 & EUSCI_B_CTLW0_TXSTT));

	data1 = EUSCI_B_CMSIS(moduleInstance)->RXBUF;

	while(!(EUSCI_B_CMSIS(moduleInstance)->IFG & EUSCI_B_IFG_RXIFG));

	// set stop immediately to ensure only one byte is read
	EUSCI_B_CMSIS(moduleInstance)->CTLW0 |= (EUSCI_B_CTLW0_TXNACK + EUSCI_B_CTLW0_TXSTP);

	// poll RX flag
    while(!(EUSCI_B_CMSIS(moduleInstance)->IFG & EUSCI_B_IFG_RXIFG));

    data2 = EUSCI_B_CMSIS(moduleInstance)->RXBUF;

    EUSCI_B_CMSIS(MCP9808_I2C_Module)->IFG &= ~(EUSCI_B_IFG_RXIFG);


//    printf("data1: 0X%x,%x\n", (data1>>8)&0xFF, data1&0xFF);
//    printf("data2: 0X%x,%x\n", (data2>>8)&0xFF, data2&0xFF);

    data = ((data2 & 0xFF) << 8) + (data1 & 0xFF);
    while (I2C_masterIsStopSent(moduleInstance));

    return data;
}

void I2C_EUSCI_B_Write16(uint32_t moduleInstance, uint8_t addr, uint16_t txData)
{
	uint8_t data_L, data_H;
	data_L = (txData) & 0xFF;
	data_H = (txData >> 8) & 0xFF;

	//original
	I2C_masterSendMultiByteStart(moduleInstance, addr);
	I2C_masterSendMultiByteNext(moduleInstance, data_H);
	I2C_masterSendMultiByteFinish(moduleInstance, data_L);
	I2C_masterSendMultiByteStop(moduleInstance);
	while (I2C_masterIsStopSent(moduleInstance));
}

void I2C_EUSCI_B_Write8(uint32_t moduleInstance, uint8_t addr, uint8_t txData)
{
	//original
	I2C_setMode(moduleInstance, EUSCI_B_I2C_TRANSMIT_MODE);
	I2C_masterSendMultiByteStart(moduleInstance, addr);
	I2C_masterSendMultiByteFinish(moduleInstance, txData);
	I2C_masterSendMultiByteStop(moduleInstance);
	while (I2C_masterIsStopSent(moduleInstance));
}


/*******************************************************************************
 *******************************************************************************/
void EUSCIB0_IRQHandler(void)
{
	printf("INT\n");
    uint_fast16_t status;

    status = I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);
    I2C_clearInterruptFlag(EUSCI_B0_BASE, status);

    /* If we reached the transmit interrupt, it means we are at index 1 of
     * the transmit buffer. When doing a repeated start, before we reach the
     * last byte we will need to change the mode to receive mode, set the start
     * condition send bit, and then load the final byte into the TXBUF.
     */
    if (status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0)
    {
        printf("TX int\n");
    }

    /* Receives bytes into the receive buffer. If we have received all bytes,
     * send a STOP condition */
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        printf("RX\n");
    }
}

void EUSCIB3_IRQHandler(void)
{
	printf("INT\n");
	uint_fast16_t status;

	status = I2C_getEnabledInterruptStatus(EUSCI_B3_BASE);
	I2C_clearInterruptFlag(EUSCI_B3_BASE, status);

	return;
}
