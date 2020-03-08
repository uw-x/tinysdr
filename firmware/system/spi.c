/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements the generic SPI driver

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#include <libraries/spi.h>

/**#############################Functions#############################**/
void SpiInit(void) {
//######### IQ Radio SPI ###############
//	Configure SPI pins
//	Configure Pins for UCA2CLK
//######################################
    GPIO_setAsPeripheralModuleFunctionInputPin(PIN_AT86RF215_SPI_SCLK.portNumber,
    		PIN_AT86RF215_SPI_SCLK.pin, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(PIN_AT86RF215_SPI_MISO.portNumber,
    		PIN_AT86RF215_SPI_MISO.pin + PIN_AT86RF215_SPI_MOSI.pin, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize Master
    eUSCI_SPI_MasterConfig param = {0};
    param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = 500000;
    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    param.spiMode = EUSCI_A_SPI_3PIN;

    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_A2_BASE, &param);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    SPI_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA2);

    //EUSCI_A2 TX buffer ready?
    while (!(SPI_getInterruptStatus(EUSCI_A2_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)));

    //Enable SPI module and Interupt
//    SPI_enableModule(EUSCI_A2_BASE);
//    SPI_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
//    Interrupt_enableInterrupt(INT_EUSCIA2);
//    EUSCI_B_SPI_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);


#ifdef PLATFORM_TINYSDR
////######### FPGA SPI ##################
//	Configure SPI pins
//	Configure Pins for UCA0CLK
////#####################################
    GPIO_setAsPeripheralModuleFunctionInputPin(PIN_FPGA_SCLK.portNumber,
        	PIN_FPGA_SCLK.pin, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(PIN_FPGA_MISO.portNumber,
    		PIN_FPGA_MISO.pin + PIN_FPGA_MOSI.pin, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize Master
    eUSCI_SPI_MasterConfig fpga_param = {0};
    fpga_param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
    fpga_param.clockSourceFrequency = CS_getSMCLK();
    fpga_param.desiredSpiClock = 100000;
    fpga_param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
//    fpga_param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    fpga_param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    fpga_param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    fpga_param.spiMode = EUSCI_A_SPI_3PIN;

    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_A0_BASE, &fpga_param);

	/* Enable SPI module */
	SPI_enableModule(EUSCI_A0_BASE);

	/* Enabling interrupts */
	SPI_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
	Interrupt_enableInterrupt(INT_EUSCIA0);

	//TX buffer ready?
	while (!(SPI_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)));

////######### LoRa Radio SPI #############
//	Configure SPI pins
//	Configure Pins for UCA0CLK
////######################################
	GPIO_setAsPeripheralModuleFunctionInputPin(PIN_SX1276_SPI_SCLK.portNumber,
	        	PIN_SX1276_SPI_SCLK.pin, GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(PIN_SX1276_SPI_MISO.portNumber,
			PIN_SX1276_SPI_MISO.pin + PIN_SX1276_SPI_MOSI.pin, GPIO_PRIMARY_MODULE_FUNCTION);

	//Initialize Master
	eUSCI_SPI_MasterConfig lora_param = {0};
	lora_param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
	lora_param.clockSourceFrequency = CS_getSMCLK();
	lora_param.desiredSpiClock = 100000;
	lora_param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
	lora_param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
	lora_param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
	lora_param.spiMode = EUSCI_B_SPI_3PIN;

	/* Configuring SPI in 3wire master mode */
	SPI_initMaster(EUSCI_B2_BASE, &lora_param);

	/* Enable SPI module */
	SPI_enableModule(EUSCI_B2_BASE);

	/* Enabling interrupts */
	SPI_enableInterrupt(EUSCI_B2_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
	Interrupt_enableInterrupt(INT_EUSCIB2);

	//TX buffer ready?
	while (!(SPI_getInterruptStatus(EUSCI_B2_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

#ifdef MX25R6435F
	////######### External Flash SPI #############
	//	Configure SPI pins
	//	Configure Pins for UCA0CLK
	////######################################
	GPIO_setAsPeripheralModuleFunctionInputPin(PIN_FLASH_SCLK.portNumber,
				PIN_FLASH_SCLK.pin, GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(PIN_FLASH_MISO.portNumber,
			PIN_FLASH_MISO.pin + PIN_FLASH_MOSI.pin, GPIO_PRIMARY_MODULE_FUNCTION);

	//Initialize Master
	eUSCI_SPI_MasterConfig flash_param = {0};
	flash_param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
	flash_param.clockSourceFrequency = CS_getSMCLK();
	flash_param.desiredSpiClock = 100000;
	flash_param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
	flash_param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
	flash_param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
	flash_param.spiMode = EUSCI_A_SPI_3PIN;

	/* Configuring SPI in 3wire master mode */
	SPI_initMaster(EUSCI_A1_BASE, &flash_param);

	/* Enable SPI module */
	SPI_enableModule(EUSCI_A1_BASE);

	/* Enabling interrupts */
	SPI_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
	Interrupt_enableInterrupt(INT_EUSCIA1);

	//TX buffer ready?
	while (!(SPI_getInterruptStatus(EUSCI_A1_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT)));
#endif
#endif

}

uint8_t SpiInOut_IQRadio(Spi_t *obj, uint8_t outData) {
    /* SPI send data */
	EUSCI_A_CMSIS(EUSCI_A2_BASE)->TXBUF = outData;
    __no_operation();

    __enable_irq();
    PCM_gotoLPM0();
    return SPI_RXData_IQ;
}

uint8_t SpiInOut_LoRa(Spi_t *obj, uint8_t outData) {
    /* SPI send data */
	EUSCI_B_CMSIS(EUSCI_B2_BASE)->TXBUF = outData;
    __no_operation();

    __enable_irq();
    PCM_gotoLPM0();

    return SPI_RXData_LoRa;
}

uint8_t fpgaSpiInOut(uint8_t outData) {
    /* SPI send data */
	EUSCI_A_CMSIS(EUSCI_A0_BASE)->TXBUF = outData;
    __no_operation();
    __enable_irq();

    PCM_gotoLPM0();

    return SPI_RXData_FPGA;
}

uint8_t flashSpiInOut(uint8_t outData) {
    /* SPI send data */
	EUSCI_A_CMSIS(EUSCI_A1_BASE)->TXBUF = outData;
    __no_operation();

    __enable_irq();

    PCM_gotoLPM0();

    return SPI_RXData_Flash;
}

//******************************************************************************
//
//This is the EUSCI_A0 interrupt vector service routine.
//
//******************************************************************************
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = EUSCI_A_CMSIS(EUSCI_A0_BASE)->IFG &
    		((EUSCI_SPI_TRANSMIT_INTERRUPT | EUSCI_SPI_RECEIVE_INTERRUPT)
                    & EUSCI_A_CMSIS(EUSCI_A0_BASE)->IE);

    EUSCI_A_CMSIS(EUSCI_A0_BASE)->IFG &= ~status;


    while(!(EUSCI_A_CMSIS(EUSCI_A0_BASE)->IFG & EUSCI_A_SPI_TRANSMIT_INTERRUPT));


    SPI_RXData_FPGA = EUSCI_A_CMSIS(EUSCI_A0_BASE)->RXBUF;

    /* exit from LPM0 */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

//******************************************************************************
//
//This is the EUSCI_A2 interrupt vector service routine.
//
//******************************************************************************
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = EUSCI_A_CMSIS(EUSCI_A2_BASE)->IFG &
    		((EUSCI_SPI_TRANSMIT_INTERRUPT | EUSCI_SPI_RECEIVE_INTERRUPT)
                    & EUSCI_A_CMSIS(EUSCI_A2_BASE)->IE);

    EUSCI_A_CMSIS(EUSCI_A2_BASE)->IFG &= ~status;

    while(!(EUSCI_A_CMSIS(EUSCI_A2_BASE)->IFG & EUSCI_A_SPI_TRANSMIT_INTERRUPT));

    SPI_RXData_IQ = EUSCI_A_CMSIS(EUSCI_A2_BASE)->RXBUF;

    /* exit from LPM0 */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

//******************************************************************************
//
//This is the EUSCI_A2 interrupt vector service routine.
//
//******************************************************************************
void EUSCIB2_IRQHandler(void)
{
    uint32_t status = EUSCI_B_CMSIS(EUSCI_B2_BASE)->IFG &
    		((EUSCI_SPI_TRANSMIT_INTERRUPT | EUSCI_SPI_RECEIVE_INTERRUPT)
                    & EUSCI_B_CMSIS(EUSCI_B2_BASE)->IE);

    EUSCI_B_CMSIS(EUSCI_B2_BASE)->IFG &= ~status;

    while(!(EUSCI_B_CMSIS(EUSCI_B2_BASE)->IFG & EUSCI_B_SPI_TRANSMIT_INTERRUPT));

    SPI_RXData_LoRa = EUSCI_B_CMSIS(EUSCI_B2_BASE)->RXBUF;

    /* exit from LPM0 */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//    __enable_irq();
}


//******************************************************************************
//
//This is the EUSCI_A1 interrupt vector service routine.
//
//******************************************************************************
void EUSCIA1_IRQHandler(void)
{
    uint32_t status = EUSCI_A_CMSIS(EUSCI_A1_BASE)->IFG &
    		((EUSCI_SPI_TRANSMIT_INTERRUPT | EUSCI_SPI_RECEIVE_INTERRUPT)
                    & EUSCI_A_CMSIS(EUSCI_A1_BASE)->IE);

    EUSCI_A_CMSIS(EUSCI_A1_BASE)->IFG &= ~status;

    while(!(EUSCI_A_CMSIS(EUSCI_A1_BASE)->IFG & EUSCI_A_SPI_TRANSMIT_INTERRUPT));

    SPI_RXData_Flash = EUSCI_A_CMSIS(EUSCI_A1_BASE)->RXBUF;

    /* exit from LPM0 */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//    __enable_irq();
}
