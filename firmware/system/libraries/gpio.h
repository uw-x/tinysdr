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
#ifndef _SYSTEM_GPIO_H_
#define _SYSTEM_GPIO_H_
#include <msp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <driverlib.h>

/**########################Variables and Types############################**/
#define P1_0 &P1OUT , &P1IN , &P1DIR , BIT0
#define P1_1 &P1OUT , &P1IN , &P1DIR , BIT1
#define P1_2 &P1OUT , &P1IN , &P1DIR , BIT2
#define P1_3 &P1OUT , &P1IN , &P1DIR , BIT3
#define P1_4 &P1OUT , &P1IN , &P1DIR , BIT4
#define P1_5 &P1OUT , &P1IN , &P1DIR , BIT5
#define P1_6 &P1OUT , &P1IN , &P1DIR , BIT6
#define P1_7 &P1OUT , &P1IN , &P1DIR , BIT7

#define P2_0 &P2OUT , &P2IN , &P2DIR , BIT0
#define P2_1 &P2OUT , &P2IN , &P2DIR , BIT1
#define P2_2 &P2OUT , &P2IN , &P2DIR , BIT2
#define P2_3 &P2OUT , &P2IN , &P2DIR , BIT3
#define P2_4 &P2OUT , &P2IN , &P2DIR , BIT4
#define P2_5 &P2OUT , &P2IN , &P2DIR , BIT5
#define P2_6 &P2OUT , &P2IN , &P2DIR , BIT6
#define P2_7 &P2OUT , &P2IN , &P2DIR , BIT7

#define P3_0 &P3OUT , &P3IN , &P3DIR , BIT0
#define P3_1 &P3OUT , &P3IN , &P3DIR , BIT1
#define P3_2 &P3OUT , &P3IN , &P3DIR , BIT2
#define P3_3 &P3OUT , &P3IN , &P3DIR , BIT3
#define P3_4 &P3OUT , &P3IN , &P3DIR , BIT4
#define P3_5 &P3OUT , &P3IN , &P3DIR , BIT5
#define P3_6 &P3OUT , &P3IN , &P3DIR , BIT6
#define P3_7 &P3OUT , &P3IN , &P3DIR , BIT7

#define P4_0 &P4OUT , &P4IN , &P4DIR , BIT0
#define P4_1 &P4OUT , &P4IN , &P4DIR , BIT1
#define P4_2 &P4OUT , &P4IN , &P4DIR , BIT2
#define P4_3 &P4OUT , &P4IN , &P4DIR , BIT3
#define P4_4 &P4OUT , &P4IN , &P4DIR , BIT4
#define P4_5 &P4OUT , &P4IN , &P4DIR , BIT5
#define P4_6 &P4OUT , &P4IN , &P4DIR , BIT6
#define P4_7 &P4OUT , &P4IN , &P4DIR , BIT7

#define P5_0 &P5OUT , &P5IN , &P5DIR , BIT0
#define P5_1 &P5OUT , &P5IN , &P5DIR , BIT1
#define P5_2 &P5OUT , &P5IN , &P5DIR , BIT2
#define P5_3 &P5OUT , &P5IN , &P5DIR , BIT3
#define P5_4 &P5OUT , &P5IN , &P5DIR , BIT4
#define P5_5 &P5OUT , &P5IN , &P5DIR , BIT5
#define P5_6 &P5OUT , &P5IN , &P5DIR , BIT6
#define P5_7 &P5OUT , &P5IN , &P5DIR , BIT7

#define P6_0 &P6OUT , &P6IN , &P6DIR , BIT0
#define P6_1 &P6OUT , &P6IN , &P6DIR , BIT1
#define P6_2 &P6OUT , &P6IN , &P6DIR , BIT2
#define P6_3 &P6OUT , &P6IN , &P6DIR , BIT3
#define P6_4 &P6OUT , &P6IN , &P6DIR , BIT4
#define P6_5 &P6OUT , &P6IN , &P6DIR , BIT5
#define P6_6 &P6OUT , &P6IN , &P6DIR , BIT6
#define P6_7 &P6OUT , &P6IN , &P6DIR , BIT7

#define P7_0 &P7OUT , &P7IN , &P7DIR , BIT0
#define P7_1 &P7OUT , &P7IN , &P7DIR , BIT1
#define P7_2 &P7OUT , &P7IN , &P7DIR , BIT2
#define P7_3 &P7OUT , &P7IN , &P7DIR , BIT3
#define P7_4 &P7OUT , &P7IN , &P7DIR , BIT4
#define P7_5 &P7OUT , &P7IN , &P7DIR , BIT5
#define P7_6 &P7OUT , &P7IN , &P7DIR , BIT6
#define P7_7 &P7OUT , &P7IN , &P7DIR , BIT7

#define P8_0 &P8OUT , &P8IN , &P8DIR , BIT0
#define P8_1 &P8OUT , &P8IN , &P8DIR , BIT1
#define P8_2 &P8OUT , &P8IN , &P8DIR , BIT2
#define P8_3 &P8OUT , &P8IN , &P8DIR , BIT3
#define P8_4 &P8OUT , &P8IN , &P8DIR , BIT4
#define P8_5 &P8OUT , &P8IN , &P8DIR , BIT5
#define P8_6 &P8OUT , &P8IN , &P8DIR , BIT6
#define P8_7 &P8OUT , &P8IN , &P8DIR , BIT7

#define PJIN_L		(HWREG8(0x40004D20))           /*!< Port J Input */
#define PJOUT_L		(HWREG8(0x40004D22))           /*!< Port J Output */
#define PJDIR_L		(HWREG8(0x40004D24))           /*!< Port J Direction */

#define PJ_0 &PJOUT_L , &PJIN_L , &PJDIR_L , BIT0
#define PJ_1 &PJOUT_L , &PJIN_L , &PJDIR_L , BIT1
#define PJ_2 &PJOUT_L , &PJIN_L , &PJDIR_L , BIT2
#define PJ_3 &PJOUT_L , &PJIN_L , &PJDIR_L , BIT3
#define PJ_4 &PJOUT_L , &PJIN_L , &PJDIR_L , BIT4
#define PJ_5 &PJOUT_L , &PJIN_L , &PJDIR_L , BIT5


/* Operation Mode for the GPIO */
typedef enum {
    PIN_INPUT = 0,
    PIN_OUTPUT,
    PIN_ALTERNATE_FCT,
    PIN_ANALOGIC
} PinModes;

/* Add a pull-up, a pull-down or nothing on the GPIO line */
typedef enum {
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN
} PinTypes;

/* Define the GPIO IRQ on a rising, falling or both edges */
typedef enum {
    NO_IRQ = 0,
    IRQ_RISING_EDGE,
    IRQ_FALLING_EDGE,
    IRQ_RISING_FALLING_EDGE
} IrqModes;

/* Define the IRQ priority on the GPIO */
typedef enum {
    IRQ_VERY_LOW_PRIORITY = 0,
    IRQ_LOW_PRIORITY,
    IRQ_MEDIUM_PRIORITY,
    IRQ_HIGH_PRIORITY,
    IRQ_VERY_HIGH_PRIORITY
} IrqPriorities;

/* Structure for the GPIO */
typedef struct {
    volatile uint8_t*   out;
    volatile uint8_t*   in;
    uint8_t             pin;
    uint8_t             portNumber;
} Gpio_t;

/* GPIO IRQ handler function prototype */
typedef void( GpioIrqHandler )( void );

/**########################External Functions############################**/
void GpioInit(Gpio_t* g, volatile uint8_t* portOut, volatile uint8_t* portIn,
		volatile uint8_t* portDir, uint8_t pin, uint8_t portNumber, PinModes mode, bool value);

void GpioSetInterrupt(Gpio_t* g, IrqModes irqMode, GpioIrqHandler* irqHandler);
void GpioRemoveInterrupt(Gpio_t* g);
void GpioWrite(Gpio_t* g, bool value);
void GpioToggle(Gpio_t* g);
bool GpioRead(Gpio_t* g);

#endif
