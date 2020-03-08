/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements the generic gpio driver

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#include <libraries/board.h>
#include <libraries/gpio.h>


/**########################Variables and Types############################**/
static GpioIrqHandler* Irq_P1[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static GpioIrqHandler* Irq_P2[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static GpioIrqHandler* Irq_P3[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static GpioIrqHandler* Irq_P4[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

static uint8_t lora_irq_mask = 0x0080;
uint32_t port6_irq_status;

/**########################Functions############################**/
uint8_t getMSB(uint8_t num)
{
    uint8_t tmp = num;  // Temporary copy of the variable
    uint8_t cnt = 0;      // Bit counter
    if (tmp != 0)    // To avoid infinite loops
        while (!(tmp & 0x01))
        {
            tmp >>= 1;
            cnt++;
        }
    return cnt;
}
void GpioInit(Gpio_t* g, volatile uint8_t* portOut, volatile uint8_t* portIn, volatile uint8_t* portDir, uint8_t pin, uint8_t portNumber, PinModes mode, bool value) {
    /* Save to struct */
    if (g != NULL) {
        g->out = portOut;
        g->in  = portIn;
        g->pin = pin;
    }

    /* Set Value */
    if (value) *portOut |=  pin;
    else       *portOut &= ~pin;

    /* Set Direction */
    if (mode == PIN_INPUT)  *portDir &= ~pin;
    if (mode == PIN_OUTPUT) *portDir |=  pin;

    /* Set port number */
    g->portNumber = portNumber;
}

void GpioSetInterrupt(Gpio_t* g, IrqModes irqMode, GpioIrqHandler* irqHandler) {
	if (irqMode == IRQ_RISING_EDGE)
	{
		MAP_GPIO_setAsInputPinWithPullDownResistor(g->portNumber, g->pin);
		MAP_GPIO_interruptEdgeSelect(g->portNumber, g->pin, GPIO_LOW_TO_HIGH_TRANSITION);
	}
	else
	{
		MAP_GPIO_setAsInputPinWithPullUpResistor(g->portNumber, g->pin);
		MAP_GPIO_interruptEdgeSelect(g->portNumber, g->pin, GPIO_HIGH_TO_LOW_TRANSITION);
	}
	MAP_GPIO_clearInterruptFlag(g->portNumber, g->pin);
	MAP_GPIO_enableInterrupt(g->portNumber, g->pin);
	MAP_Interrupt_enableInterrupt(INT_PORT1 + (g->portNumber-GPIO_PORT_P1));
}

void GpioRemoveInterrupt(Gpio_t* g){

}

void GpioWrite(Gpio_t* g, bool value) {
    if (value) *(g->out) |=   g->pin;
    else       *(g->out) &= ~(g->pin);
}

void GpioToggle(Gpio_t* g) {
    *(g->out) ^= g->pin;
}

bool GpioRead(Gpio_t* g) {
    return !!(*(g->in) & g->pin);
}

/* GPIO ISR, LoRa DIO0 */
void PORT6_IRQHandler(void)
{
//	MAP_Interrupt_disableMaster();
	port6_irq_status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, port6_irq_status);

//    printf("INT \n");

    if (port6_irq_status & lora_irq_mask)
    {
    	if (MCU_State == MCU_STATE_BR_RX_WAIT)
    	{
    		MCU_State = MCU_STATE_BR_RX;
    	}
//    	else if (MCU_State == MCU_STATE_BR_TX_WAIT)
//    	{
//    		MCU_State = MCU_STATE_BR_TX_INIT;
//    	}
    }
    GpioToggle(&TinySDR.LED_D2);
//    MAP_Interrupt_enableMaster();
}




