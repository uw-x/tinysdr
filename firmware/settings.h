/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: TinySDR Settings

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#ifndef _SETTINGS_H_
#define _SETTINGS_H_
#include <libraries/gpio.h>


/* Structure for the MSP pins */
typedef struct {
    volatile uint8_t*   portOut;
    volatile uint8_t*   portIn;
    volatile uint8_t*   portDir;
    uint8_t             pin;
    uint8_t             portNumber;
} MSP_PIN_t;

/**#############################Debug#############################**/
//#define DEBUG

/**#############################Platform#############################**/
#define PLATFORM_TINYSDR		1
//#define PLATFORM_REB215			2
//#define PLATFORM			platform_tinySDR



/**#############################CONFIGURATION#############################**/
/* Pin definition based on the board design */
const static MSP_PIN_t PIN_MSP_P1_0		= {P1_0, 0x01};
const static MSP_PIN_t PIN_MSP_P1_1		= {P1_1, 0x01};
const static MSP_PIN_t PIN_MSP_P1_2     = {P1_2, 0x01};
const static MSP_PIN_t PIN_MSP_P1_3     = {P1_3, 0x01};
const static MSP_PIN_t PIN_MSP_P1_4		= {P1_4, 0x01};
const static MSP_PIN_t PIN_MSP_P1_5     = {P1_5, 0x01};
const static MSP_PIN_t PIN_MSP_P1_6     = {P1_6, 0x01};
const static MSP_PIN_t PIN_MSP_P1_7     = {P1_7, 0x01};

const static MSP_PIN_t PIN_MSP_P2_0		= {P2_0, 0x02};
const static MSP_PIN_t PIN_MSP_P2_1		= {P2_1, 0x02};
const static MSP_PIN_t PIN_MSP_P2_2		= {P2_2, 0x02};
const static MSP_PIN_t PIN_MSP_P2_3		= {P2_3, 0x02};
const static MSP_PIN_t PIN_MSP_P2_4		= {P2_4, 0x02};
const static MSP_PIN_t PIN_MSP_P2_5		= {P2_5, 0x02};
const static MSP_PIN_t PIN_MSP_P2_6		= {P2_6, 0x02};
const static MSP_PIN_t PIN_MSP_P2_7		= {P2_7, 0x02};


const static MSP_PIN_t PIN_MSP_P3_0		= {P3_0, 0x03};
const static MSP_PIN_t PIN_MSP_P3_1		= {P3_1, 0x03};
const static MSP_PIN_t PIN_MSP_P3_2		= {P3_2, 0x03};
const static MSP_PIN_t PIN_MSP_P3_3		= {P3_3, 0x03};
const static MSP_PIN_t PIN_MSP_P3_4     = {P3_4, 0x03};
const static MSP_PIN_t PIN_MSP_P3_5     = {P3_5, 0x03};
const static MSP_PIN_t PIN_MSP_P3_6     = {P3_6, 0x03};
const static MSP_PIN_t PIN_MSP_P3_7		= {P3_7, 0x03};


const static MSP_PIN_t PIN_MSP_P4_0		= {P4_0, 0x04};
const static MSP_PIN_t PIN_MSP_P4_1		= {P4_1, 0x04};
const static MSP_PIN_t PIN_MSP_P4_2		= {P4_2, 0x04};
const static MSP_PIN_t PIN_MSP_P4_3		= {P4_3, 0x04};
const static MSP_PIN_t PIN_MSP_P4_4		= {P4_4, 0x04};
const static MSP_PIN_t PIN_MSP_P4_5		= {P4_5, 0x04};
const static MSP_PIN_t PIN_MSP_P4_6		= {P4_6, 0x04};
const static MSP_PIN_t PIN_MSP_P4_7		= {P4_7, 0x04};


const static MSP_PIN_t PIN_MSP_P5_0     = {P5_0, 0x05};
const static MSP_PIN_t PIN_MSP_P5_1     = {P5_1, 0x05};
const static MSP_PIN_t PIN_MSP_P5_2     = {P5_2, 0x05};
const static MSP_PIN_t PIN_MSP_P5_3     = {P5_3, 0x05};
const static MSP_PIN_t PIN_MSP_P5_4     = {P5_4, 0x05};
const static MSP_PIN_t PIN_MSP_P5_5     = {P5_5, 0x05};
const static MSP_PIN_t PIN_MSP_P5_6     = {P5_6, 0x05};
const static MSP_PIN_t PIN_MSP_P5_7     = {P5_7, 0x05};

const static MSP_PIN_t PIN_MSP_P6_0		= {P6_0, 0x06};
const static MSP_PIN_t PIN_MSP_P6_1		= {P6_1, 0x06};
const static MSP_PIN_t PIN_MSP_P6_2		= {P6_2, 0x06};
const static MSP_PIN_t PIN_MSP_P6_3		= {P6_3, 0x06};
const static MSP_PIN_t PIN_MSP_P6_4		= {P6_4, 0x06};
const static MSP_PIN_t PIN_MSP_P6_5		= {P6_5, 0x06};
const static MSP_PIN_t PIN_MSP_P6_6		= {P6_6, 0x06};
const static MSP_PIN_t PIN_MSP_P6_7		= {P6_7, 0x06};

const static MSP_PIN_t PIN_MSP_P7_0		= {P7_0, 0x07};
const static MSP_PIN_t PIN_MSP_P7_1		= {P7_1, 0x07};
const static MSP_PIN_t PIN_MSP_P7_2		= {P7_2, 0x07};
const static MSP_PIN_t PIN_MSP_P7_3		= {P7_3, 0x07};
const static MSP_PIN_t PIN_MSP_P7_4		= {P7_4, 0x07};
const static MSP_PIN_t PIN_MSP_P7_5		= {P7_5, 0x07};
const static MSP_PIN_t PIN_MSP_P7_6		= {P7_6, 0x07};
const static MSP_PIN_t PIN_MSP_P7_7		= {P7_7, 0x07};

const static MSP_PIN_t PIN_MSP_P8_0		= {P8_0, 0x08};
const static MSP_PIN_t PIN_MSP_P8_1		= {P8_1, 0x08};
const static MSP_PIN_t PIN_MSP_P8_2		= {P8_2, 0x08};

const static MSP_PIN_t PIN_MSP_PJ_0		= {PJ_0, 0x0B};
const static MSP_PIN_t PIN_MSP_PJ_1		= {PJ_1, 0x0B};
const static MSP_PIN_t PIN_MSP_PJ_2		= {PJ_2, 0x0B};
const static MSP_PIN_t PIN_MSP_PJ_3		= {PJ_3, 0x0B};
const static MSP_PIN_t PIN_MSP_PJ_4		= {PJ_4, 0x0B};
const static MSP_PIN_t PIN_MSP_PJ_5		= {PJ_5, 0x0B};

/* Mapping MSP pins to the design */
#ifdef PLATFORM_TINYSDR
	/* AT86RF215 radio */
	#define PIN_AT86RF215_SPI_NSS           PIN_MSP_P3_0
	#define PIN_AT86RF215_SPI_MOSI       	PIN_MSP_P3_3
	#define PIN_AT86RF215_SPI_MISO          PIN_MSP_P3_2
	#define PIN_AT86RF215_SPI_SCLK          PIN_MSP_P3_1
	#define PIN_AT86RF215_NRESET            PIN_MSP_P7_2
	#define PIN_AT86RF215_IRQ               PIN_MSP_P7_3

	/* SX1276 radio */
	#define PIN_SX1276_SPI_NSS           	PIN_MSP_P3_4
	#define PIN_SX1276_SPI_MOSI       		PIN_MSP_P3_6
	#define PIN_SX1276_SPI_MISO          	PIN_MSP_P3_7
	#define PIN_SX1276_SPI_SCLK          	PIN_MSP_P3_5
	#define PIN_SX1276_NRESET            	PIN_MSP_P7_1
	//#define PIN_SX1276_DIO0               	PIN_MSP_PJ_4
	#define PIN_SX1276_DIO0               	PIN_MSP_P6_7


	/* tinySDR board LEDs */
	#define PIN_LED_D1						PIN_MSP_P4_4
	#define PIN_LED_D2						PIN_MSP_P4_3

	/* tinySDR D2 voltage domain */
	#define PIN_D2_CTL1						PIN_MSP_P4_5
	#define PIN_D2_CTL2						PIN_MSP_P4_6
	#define PIN_D2_CTL3						PIN_MSP_P5_7

	/* tinySDR 900MHz Switch */
	#define	PIN_SW_A0						PIN_MSP_P1_4
	#define	PIN_SW_A1						PIN_MSP_P7_0

	/* tinySDR PA09 */
	#define PIN_PA09_CPS					PIN_MSP_P5_2
	#define PIN_PA09_CTX					PIN_MSP_P5_3

	/* tinySDR PA24 */
	#define PIN_PA24_CRX					PIN_MSP_P1_7
	#define PIN_PA24_CTX					PIN_MSP_PJ_5
	#define PIN_PA24_CPS					PIN_MSP_P4_2


	/* FPGA and MSP SPI connections */
	#define	PIN_FPGA_Reset					PIN_MSP_P1_5
	#define	PIN_FPGA_CFG0					PIN_MSP_P8_1
	#define	PIN_FPGA_CFG1					PIN_MSP_P8_0
	#define	PIN_FPGA_CFG2					PIN_MSP_P1_6

	#define PIN_FPGA_MISO					PIN_MSP_P1_2
	#define PIN_FPGA_MOSI					PIN_MSP_P1_3
	#define PIN_FPGA_CS						PIN_MSP_P1_0
	#define PIN_FPGA_SCLK					PIN_MSP_P1_1

	/* External Interface */
	#define PIN_IF_I2C_SDA					PIN_MSP_P6_6
	#define PIN_IF_I2C_SCL					PIN_MSP_P6_7

	/* External Flash */
	#define	PIN_FLASH_CS					PIN_MSP_P2_0
	#define	PIN_FLASH_MOSI					PIN_MSP_P2_3
	#define	PIN_FLASH_MISO					PIN_MSP_P2_2
	#define	PIN_FLASH_SCLK					PIN_MSP_P2_1

	/* Power */
	#define	PIN_PWR_3P0						PIN_MSP_P5_6

	/* Clock */
	#define PIN_CLK_LFXIN					PIN_MSP_PJ_0
	#define PIN_CLK_LFXOUT					PIN_MSP_PJ_1
	#define PIN_CLK_HFXIN					PIN_MSP_PJ_3
	#define PIN_CLK_HFXOUT					PIN_MSP_PJ_2

#endif

#ifdef PLATFORM_REB215
	// SPI for tinySDR
	#define PIN_AT86RF215_SPI_NSS           PIN_MSP_P3_0
	#define PIN_AT86RF215_SPI_MOSI       	PIN_MSP_P3_3
	#define PIN_AT86RF215_SPI_MISO          PIN_MSP_P3_2
	#define PIN_AT86RF215_SPI_SCLK          PIN_MSP_P3_1

	// AT86RF215
	#define PIN_AT86RF215_NRESET            PIN_MSP_P7_2
	#define PIN_AT86RF215_IRQ               PIN_MSP_P7_3
/*
	// SPI for AT86RF215
	#define PIN_SPI_NSS                 	PIN_MSP_P1_3
	// AT86RF215
	#define PIN_AT86RF215_NRESET            PIN_MSP_P1_2
	#define PIN_AT86RF215_IRQ               PIN_MSP_P2_6
	//XSDR board LEDs
	#define PIN_XSDRv0_LED_Power			PIN_MSP_P4_6
	#define PIN_XSDRv0_LED_TXRX				PIN_MSP_P4_5
*/

	#define PIN_IF_I2C_SDA					PIN_MSP_P1_6
	#define PIN_IF_I2C_SCL					PIN_MSP_P1_7
#endif

/**#############################SETTINGS#############################**/
/* Frequency */
#define USE_BAND_892    892000000 // Hz
#define USE_BAND_895    895000000 // Hz
#define USE_BAND_910    910000000 // Hz
#define USE_BAND_914    914000000 // Hz
#define USE_BAND_915    915000000 // Hz

#define BW_125      0
#define BW_250      1
#define BW_500      2

/* Spreading Factor     [SF7..SF12] */
#define SF_7        7
#define SF_8        8
#define SF_9        9
#define SF_10       10
#define SF_11       11
#define SF_12       12

/* Coding Rate          [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#define CR_5        1
#define CR_6        2
#define CR_7        3
#define CR_8        4


#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

/* Radio modes */
#define Radio_Mode_IQ_RX			0
#define Radio_Mode_IQ_TX			1
#define Radio_Mode_SingleTone		2
#define Radio_Mode_BB_PHY_RX		3


/**############################# BOARD #############################**/
typedef enum
{
    MCU_STATE_INIT 	= 0,
    MCU_STATE_TXRX ,
    MCU_STATE_INTR ,
	MCU_STATE_BR_INIT,
	MCU_STATE_BR_RX_INIT,
	MCU_STATE_BR_RX_WAIT,
	MCU_STATE_BR_RX,
	MCU_STATE_BR_TX_INIT,
	MCU_STATE_BR_TX,
	MCU_STATE_BR_TX_WAIT,
	MCU_STATE_BR_ACK,
	MCU_STATE_BR_FLASH,
	MCU_STATE_UP_INIT,
	MCU_STATE_UP_DONE
}MCU_States_t;

typedef enum
{
    SW_RF09_IQ = 0,
    SW_RF09_BR_TX,
	SW_RF09_BR_RX,
	SW_RF24_IQ,
}Switch_t;

typedef enum
{
	PA_Sleep = 0,
	PA_RX_Bypass,
	PA_RX_LNA,
	PA_TX_Bypass,
	PA_TX_High,
	PA_TX_Low,
}PA_Settings_t;

typedef enum
{
	PA_900,
	PA_2400
}PA_Types_t;


extern MCU_States_t MCU_State;

/* Radio settings */
#define RX_BW_IF					RF_BW2000KHZ_IF2000KHZ
#define Radio_Mode					Radio_Mode_IQ_TX

/**#############################Clock#############################**/
#define CLK_FREQ_8M     1
#define CLK_FREQ_16M    2
#define CLK_FREQ_24M    3

#define CLK_FREQ        CLK_FREQ_8M

#if CLK_FREQ == CLK_FREQ_8M
    #define CYCLES_mS      8000
#elif CLK_FREQ == CLK_FREQ_16M
    #define CYCLES_mS      16000
#elif CLK_FREQ == CLK_FREQ_24M
    #define CYCLES_mS      24000
#else
    #define CYCLES_mS      8000
#endif

#define CYCLES_us   CYCLES_mS/1000

/**#############################Radio#############################**/
#define RADIO_09    1
#define RADIO_24    2

#define RADIO_TYPE  RADIO_09


/**#############################Interfaces#############################**/
#ifdef PLATFORM_TINYSDR
#define	MCP9808_I2C_Module		EUSCI_B3_BASE
#define MCP9808_I2C_INT			INT_EUSCIB3
#define MCP9808_I2C_GPIO_FUNC	GPIO_SECONDARY_MODULE_FUNCTION
#endif

#ifdef PLATFORM_REB215
#define	MCP9808_I2C_Module		EUSCI_B0_BASE
#define MCP9808_I2C_INT			INT_EUSCIB0
#define MCP9808_I2C_GPIO_FUNC	GPIO_PRIMARY_MODULE_FUNCTION
#endif

#define MCP9808_SLAVE_ADDR	0x18

/**#############################External Flash#############################**/
//Note: If you enable, FPGA flash programming does not work.MX25R6435F
#define MX25R6435F
extern const uint8_t flash[99962];

#endif
