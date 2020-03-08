/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements different tasks in TinySDR.

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#include <at86rf215/at86rf215.h>
#include <libraries/board.h>
#include <stdio.h>
#include "msp.h"
#include <stdbool.h>
#include <driverlib.h>
#include <settings.h>
#include <libraries/clock.h>
#include <testmini.h>
#include <fpga.h>
#include <ble.h>
#include <libraries/i2c.h>
#include <mcp9808/mcp9808.h>
#include <task.h>
#include <sx1276/sx1276.h>
#include <flash/mx25r6435f.h>
#include <ota.h>

/**########################Variables and Types############################**/
/* IQ radio events function pointer */
void At86rf215_OnTxDone(void);
void At86rf215_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void At86rf215_OnTxTimeout(void);
void At86rf215_OnRxTimeout(void);
void At86rf215_OnRxError(void);
void At86rf215_FhssChangeChannel(uint8_t currentChannel);
void At86rf215_CadDone(bool channelActivityDetected);

/* LoRa radio events function pointer */
void SX1276_OnTxDone(void);
void SX1276_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void SX1276_OnTxTimeout(void);
void SX1276_OnRxTimeout(void);
void SX1276_OnRxError(void);
void SX1276_FhssChangeChannel(uint8_t currentChannel);
void SX1276_CadDone(bool channelActivityDetected);

RadioEvents_t at86rf215 =
{
		At86rf215_OnTxDone,
		At86rf215_OnTxTimeout,
		At86rf215_OnRxDone,
		At86rf215_OnRxTimeout,
		At86rf215_OnRxError,
		At86rf215_FhssChangeChannel,
		At86rf215_CadDone
};

RadioEvents_t sx1276 =
{
		SX1276_OnTxDone,
		SX1276_OnTxTimeout,
		SX1276_OnRxDone,
		SX1276_OnRxTimeout,
		SX1276_OnRxError,
		SX1276_FhssChangeChannel,
		SX1276_CadDone
};

/* LoRa Settings */
#define RX_TIMEOUT_VALUE    			10000
#define LORA_BANDWIDTH                  BW_500
#define LORA_SPREADING_FACTOR           SF_8
#define LORA_CODINGRATE                 CR_6
#define LORA_PREAMBLE_LENGTH            8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT             5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON      false
#define LORA_IQ_INVERSION            	false
#define TX_OUTPUT_POWER					0

static uint8_t BR_buffer[RFLR_PAYLOADMAXLENGTH];
uint16_t data_sequence;
uint16_t ack_sequence;
uint8_t br_miss;
uint8_t flash_buffer[FLASH_Page_Size];
bool BR_last_packet;


/**########################External functions############################**/
void task_fpga(void)
{
	MAP_WDT_A_holdTimer();

	ClockInit();
    BoardInitMcu();
	PWR_D2_Set(PWR_D2_2_50);
	GpioWrite(&TinySDR.PWR_3P0, 1);
	GpioWrite(&TinySDR.LED_D1, 1);
//	GpioWrite(&TinySDR.LED_D2, 1);

	fpgaSetConfig(0);
	delay_ms(2000);
	fpgaReset();
}


void task_ext_flash(void)
{
	/* Initialization */
	ClockInit();
    BoardInitMcu();
    BoardPWRInit();
    GpioWrite(&TinySDR.LED_D1, 0);

    fpgaSetConfig(0);
	fpgaReset();

	printf("Flash\n");

	while(FlashInit() == false)
	{
		GpioWrite(&TinySDR.LED_D2, true);
	}
	GpioWrite(&TinySDR.LED_D2, false);

	/* Write firmware */
//	FlashMassWrite(99962);
//	FlashMassCheck(99962);

	/* Reading Flash*/
	uint8_t flash_addr[3];
	flash_addr[0] = 0x00;
	flash_addr[1] = 0x01;
	flash_addr[2] = 0x00;
	uint8_t flash_data[1024];
	FlashReadData(flash_addr, flash_data, 1024);

	uint32_t ii;
	for(ii=0; ii<250; ii++)
	{
		printf("D: %x\n", flash_data[ii]);
	}
}


void task_fpga_lora_tx(void)
{
	uint32_t Frequency;
	uint8_t version;

	/* Initialization */
	ClockInit();
    BoardInitMcu();
    BoardPWRInit();

	/* Backbone Radio */
	SX1276Reset();
    version = SX1276Read(0x42);
    SX1276SetSleep();

	/* IQ Radio Initialization */
	 AT86RF215SetModem(MODEM_09);
	 setSwitchMode(AT86RF215.RF_Settings.Modem);
	 At86rf215Radio.Init(&at86rf215);

	 version = AT86RF215Read(REG_RF_VN);
	 while (version != 0x03)
	 {
		 GpioWrite(&TinySDR.LED_D2, true);
		 delay_ms(1);
		 version = AT86RF215Read(REG_RF_VN);
	 }
	 printf("IQ Radio Ver: 0h%x \n", version);
	 GpioWrite(&TinySDR.LED_D2, false);

	 /* PA Setup */
	 setPA(PA_900, PA_TX_Bypass);
	 setPA(PA_2400, PA_Sleep);

	 /* IQ Radio Mode */
	 Frequency = 900000000;
	 AT86RF215TxSetIQ(Frequency);
	 delay_ms(1);
	 AT86RF215SetState(RF_CMD_TX);

	 /* FPGA */
	 fpgaSetConfig(0);
	 delay_ms(2000);
	 fpgaReset();

	 while(1)
	 {

	 }
}

void task_ble(void)
{
	/* Stop watchdog timer */
	WDTCTL = WDTPW | WDTHOLD;
	uint32_t Frequency;

	ClockInit();
	BoardInitMcu();
	BoardPWRInit();

	AT86RF215SetModem(MODEM_24);
	setSwitchMode(AT86RF215.RF_Settings.Modem);
	//Channel 38
//    Frequency = 2478000000 + 23500;
	////Channel 36
	Frequency = 2402000000 + 23500;
	setPA(PA_2400, PA_TX_Bypass);

	RadioEvents_t dummy = {};
	At86rf215Radio.Init(&dummy);
	GpioWrite(&TinySDR.LED_D2, true);

	/* test SPI connection */
	uint8_t version = AT86RF215Read(REG_RF_VN);
	while (version != 0x03)
	{
		GpioWrite(&TinySDR.LED_D2, false);
		delay_ms(1);
		version = AT86RF215Read(REG_RF_VN);
	}
	printf("IQ Radio Ver: %x \n", version);
	GpioWrite(&TinySDR.LED_D2, true);


	/* Radio Mode */
	AT86RF215TxSetIQ(Frequency);
	delay_ms(1);
	AT86RF215SetState(RF_CMD_TX);
	fpgaSetConfig(0);

	/* FPGA firmware load */
	delay_ms(2000);
	fpgaReset();

	/* BLE TX */
	uint8_t dev_name [7] = {'T', 'i', 'n', 'y', 'S', 'D', 'R'};
	uint8_t UUID [2] = {0x1C, 0x18};

	uint8_t ind = 0;
	uint8_t message_len;
	/* Local name in packet */
//	message_len = 3 + 4 + 9;

	/* URL in packet */
	message_len = 19;

	uint8_t *message;
	message = (uint8_t *) malloc(message_len);

/* Local name in packet */
	/* Type: flag */
	message[0] = 0x02;
	message[1] = 0x01;
	message[2] = 0x06;
	ind = ind + 3;

	/* Type: UUID */
	message[ind] = 0x03;
	message[ind+1] = 0x03;
	message[ind+2] = UUID[0];
	message[ind+3] = UUID[1];
	ind = ind + 4;

	/* Type: Complete Name */
	message[ind] = 0x08;
	message[ind+1] = 0x09;
	ind = ind + 2;
	uint8_t ii;
	for (ii=0; ii<7; ii++)
		message[ind + ii] = dev_name[ii];
	ind = ind + 7;

/* URL in packet */
	uint8_t advdata[] =
	{
	  0x03,  // Length of Service List
	  0x03,  // Param: Service List
	  0xAA, 0xFE,  // Eddystone ID
	  0x0E,  // Length of Service Data
	  0x16,  // Service Data
	  0xAA, 0xFE, // Eddystone ID
	  0x10,  // Frame type: URL
	  0x00, // Power
	  0x03, // https://
	  'T',
	  'i',
	  'n',
	  'y',
	  'S',
	  'D',
	  'R',
	  0x08, // .org
	};

	int jj;
	for (jj=0; jj<message_len; jj++)
	{
		message[jj] = advdata[jj];
		printf("data: %d\n", advdata[jj]);
	}

	while(1)
	{
		ble_send(message, message_len);
		delay_ms(100);
	}
}

void task_power_measure(void)
{

	/* Initialization */
    BoardInitMcu();
    BoardPWRInit();

	setPA(PA_900, PA_Sleep);
	setPA(PA_2400, PA_Sleep);

	GpioWrite(&TinySDR.LED_D2, false);
	GpioWrite(&TinySDR.LED_D2, false);

    GpioWrite(&TinySDR.LED_D1, false);
    GpioWrite(&TinySDR.PWR_3P0, false);

    /* Turn off pins to I/Q Radio */
    GpioWrite(&AT86RF215.Spi.NSS, false);
    GpioWrite(&AT86RF215.Spi.MOSI, false);
    GpioWrite(&AT86RF215.Spi.MISO, false);
    GpioWrite(&AT86RF215.Spi.SCLK, false);
    GpioWrite(&AT86RF215.Reset, false);
    GpioWrite(&AT86RF215.IRQ, false);

    /* Turn off pins to LoRa Radio */
	GpioWrite(&SX1276.Spi.NSS, false);
	GpioWrite(&SX1276.Spi.MOSI, false);
	GpioWrite(&SX1276.Spi.MISO, false);
	GpioWrite(&SX1276.Spi.SCLK, false);
	GpioWrite(&SX1276.Reset, false);
	GpioWrite(&SX1276.DIO0, false);

	/* Turn off pins to FPGA */
	GpioWrite(&TinySDR.FPGA.Reset, false);
	GpioWrite(&TinySDR.FPGA.CFG0, false);
	GpioWrite(&TinySDR.FPGA.CFG1, false);
	GpioWrite(&TinySDR.FPGA.CFG2, false);
	GpioWrite(&TinySDR.FPGA.SPI.NSS, false);

	GpioWrite(&TinySDR.FPGA.SPI.MOSI, false);
	GpioWrite(&TinySDR.FPGA.SPI.MISO, false);
	GpioWrite(&TinySDR.FPGA.SPI.SCLK, false);

	/* Turn off pins to Flash */
	GpioWrite(&TinySDR.FLASH.SPI.NSS, false);

	/* Terminating all remaining pins to minimize power consumption. This is
	 * done by register accesses for simplicity and to minimize branching API
	 * calls */
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, PIN_ALL8);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_PB, PIN_ALL16);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_PC, PIN_ALL16);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_PD, PIN_ALL16);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_PE, PIN_ALL16);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_PJ, PIN_ALL16);
	MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PJ, GPIO_PIN4);

	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, PIN_ALL8);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PB, PIN_ALL16);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PC, PIN_ALL16);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PD, PIN_ALL16);
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PE, PIN_ALL16);

	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN4);

	/* Starting LFXT and sourcing ACLK and BCLK from it */
	MAP_CS_setExternalClockSourceFrequency(32000, 48000000);
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
			GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
	MAP_CS_startLFXT(false);
	MAP_CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
	MAP_CS_initClockSignal(CS_BCLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

	/* Disabling high side voltage monitor/supervisor */
	MAP_PSS_disableHighSide();

	/* Enabling "rude" mode which forces device to go to sleep regardless of
	 * outstanding clock requests
	 */
	MAP_PCM_enableRudeMode();

	/* Enabling MASTER interrupts */
	MAP_Interrupt_enableMaster();

	GpioWrite(&TinySDR.LED_D2, true);

	/* Going to LPM3 */
	while (1)
	{
		/* Note that while this examples just goes to LPM3, LPM4 is essentially
			just LPM3 with WDT_A/RTC_C disabled. For convenience, the user can
			use the MAP_PCM_gotoLPM4() function if they want the API to handle
			the disabling of these components */
		MAP_PCM_gotoLPM3();
	}
}

void task_i2c(void)
{
	static uint16_t Res;
	static uint16_t conf;

	mcp9808_init();

    Res = mcp9808_RegRead(MCP9808_REG_RESOLUTION);
    printf("Res1: %x\n", (Res) & 0xFF);
    printf("Res2: %x\n", (Res>>8) & 0xFF);

    conf = mcp9808_RegRead(MCP9808_REG_CONFIG);
    printf("conf1: %x\n", (conf) & 0xFF);
    printf("conf2: %x\n", (conf>>8) & 0xFF);

    float t;
    while(1)
    {
//    	mcp9808_wake();
    	t = mcp9808_readTempC();
    	printf("temp: %f\n", t);
//    	delay_ms(2000);

//    	mcp9808_shutdown(true);
    	delay_ms(10);
    }

}

void task_single_tone(void)
{
	uint32_t Frequency;
	uint8_t version;

	/* Initialization */
	ClockInit();
    BoardInitMcu();
    BoardPWRInit();

	/* Backbone Radio */
	SX1276Reset();
    version = SX1276Read(0x42);
    SX1276SetSleep();

	/* IQ Radio Initialization */
	 AT86RF215SetModem(MODEM_09);
	 setSwitchMode(AT86RF215.RF_Settings.Modem);
	 At86rf215Radio.Init(&at86rf215);

	 version = AT86RF215Read(REG_RF_VN);
	 while (version != 0x03)
	 {
		 GpioWrite(&TinySDR.LED_D2, true);
		 delay_ms(1);
		 version = AT86RF215Read(REG_RF_VN);
	 }
	 printf("IQ Radio Ver: 0h%x \n", version);
	 GpioWrite(&TinySDR.LED_D2, false);

	 /* PA Setup */
	 setPA(PA_900, PA_TX_Bypass);
	 setPA(PA_2400, PA_Sleep);

	 /* IQ Radio Mode */
	 Frequency = 900000000;
	 AT86RF215TxSetIQ(Frequency);
	 delay_ms(1);
	 AT86RF215SetState(RF_CMD_TX);

	 /* FPGA */
	 fpgaSetConfig(0);
	 delay_ms(2000);
	 fpgaReset();

	 while(1)
	 {

	 }

}


void task_lora_test(void)
{
	uint8_t version;
	uint16_t sequence_tmp;
	uint16_t packet_size;
	uint32_t flash_mem_ind;
	uint32_t flash_buff_ind;

	/*
	 * Initialization
	 */
	ClockInit();
    BoardInitMcu();
    BoardPWRInit();
    GpioWrite(&TinySDR.LED_D1, 0);

    /*
     * Backbone Radio
     */
    SX1276Reset();
    version = SX1276Read(REG_VERSION);
	while(version != 0x12)
	{
		GpioWrite(&TinySDR.LED_D1, true);
		delay_ms(1);
		version = SX1276Read(REG_VERSION);
	}
	GpioWrite(&TinySDR.LED_D1, false);
	printf("LoRa Ver: 0x%x \n", version);


	while(FlashInit() == false)
	{
		GpioWrite(&TinySDR.LED_D1, true);
	}
	GpioWrite(&TinySDR.LED_D1, false);


	MCU_State = MCU_STATE_BR_INIT;
    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();

    uint8_t irqFlags;

    data_sequence = 0;
    ack_sequence = 0;
	flash_mem_ind = 0;
	flash_buff_ind = 0;
	br_miss = 0;
	BR_last_packet = false;
	while(1)
	{
//		printf("irq: %x\n", SX1276Read(REG_LR_IRQFLAGS));

		switch (MCU_State)
		{
		case MCU_STATE_BR_INIT:
			{
				sx1276Radio.Init(&sx1276);
			    sx1276Radio.SetChannel(900000000);

			    sx1276Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
								   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
								   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
								   true, 0, 0, LORA_IQ_INVERSION, 3000);

			    sx1276Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
								   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
								   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
								   0, true, 0, 0, LORA_IQ_INVERSION, true);
			    MCU_State = MCU_STATE_BR_RX_INIT;
//			    SX1276SetIRQFlagMask(0xFF);

			    SX1276Write(REG_LR_IRQFLAGS, SX1276Read(REG_LR_IRQFLAGS) 			| 0xFF);
			    SX1276Write(REG_LR_IRQFLAGSMASK, SX1276Read(REG_LR_IRQFLAGSMASK) 	| 0xFF);

//			    MCU_State = MCU_STATE_BR_TX_INIT;
			    break;
			}
		case MCU_STATE_BR_RX_INIT:
			{
//				printf("RX INIT\n");
				setSwitchMode(MODEM_BR_RX);
				setPA(PA_900, PA_RX_Bypass);

				SX1276SetIRQFlagMask(RFLR_IRQFLAGS_RXDONE_MASK);

				sx1276Radio.Rx(RX_TIMEOUT_VALUE);
//				TimerA1SetStart(1000);
//				printf(" RegDio1: %x\n", SX1276Read(REG_LR_DIOMAPPING1));
//				printf(" RegDio2: %x\n", SX1276Read(REG_LR_DIOMAPPING2));
				MCU_State = MCU_STATE_BR_RX_WAIT;
				break;
			}
		case MCU_STATE_BR_RX_WAIT:
			{
//				MAP_PCM_gotoLPM3();
				break;
			}
		case MCU_STATE_BR_RX:
			{
				irqFlags = SX1276Read(REG_LR_IRQFLAGS);

//				if (irqFlags & RFLR_IRQFLAGS_RXTIMEOUT)
//					GpioWrite(&TinySDR.LED_D1, true);


				packet_size = SX1276ReceivePayload(BR_buffer);

				if((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) != RFLR_IRQFLAGS_PAYLOADCRCERROR)
				{
//					printf("RX\n");
					sequence_tmp = (BR_buffer[1] << 8) + (BR_buffer[2]);
//					printf("RX: seq: %x\n", sequence_tmp);

					switch(BR_buffer[0])
					{
					case PKT_Type_Data:
						{
							if (sequence_tmp == data_sequence)
							{
								data_sequence++;

//								MAP_Interrupt_disableMaster();

							/* storei n buffer, Write to flash */
								if (BR_buffer[3] < (FLASH_Page_Size - flash_buff_ind + 1))			/* space to store the whole packet */
								{
									memcpy(flash_buffer + flash_buff_ind, BR_buffer + 4, BR_buffer[3]);
									flash_buff_ind += BR_buffer[3];
								}
								else if (BR_buffer[3] == (FLASH_Page_Size - flash_buff_ind + 1))	/* space to only store this packet */
								{
									memcpy(flash_buffer + flash_buff_ind, BR_buffer + 4, BR_buffer[3]);
									FlashEraseWritePage(flash_mem_ind, flash_buffer, FLASH_Page_Size);
									flash_buff_ind = 0;
									flash_mem_ind += FLASH_Page_Size;
								}
								else																/* store, write to flash, store the rest */
								{
									memcpy(flash_buffer+flash_buff_ind, BR_buffer+4, FLASH_Page_Size-flash_buff_ind);
									FlashEraseWritePage(flash_mem_ind, flash_buffer, FLASH_Page_Size);
									flash_mem_ind += FLASH_Page_Size;
									memcpy(flash_buffer, BR_buffer + 4 + (FLASH_Page_Size-flash_buff_ind), BR_buffer[3] - (FLASH_Page_Size-flash_buff_ind));
									flash_buff_ind = BR_buffer[3] - (FLASH_Page_Size-flash_buff_ind);
								}

//								MAP_Interrupt_enableMaster();

								/* update ack */
								ack_sequence = sequence_tmp;
							}
							else if (sequence_tmp > data_sequence)
							{
							}
							else if (sequence_tmp < data_sequence)
							{
							}
							MCU_State = MCU_STATE_BR_TX_INIT;
							break;
						}
					case PKT_Type_Update_Done:
						{
							BR_last_packet = true;
							MCU_State = MCU_STATE_BR_TX_INIT;
							break;
						}
					default:
						{
							MCU_State = MCU_STATE_BR_TX_INIT;
							break;
						}
					}
				}
				else
				{
					SX1276SetIRQFlagMask(RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK + RFLR_IRQFLAGS_RXDONE_MASK);
					MCU_State = MCU_STATE_BR_RX_INIT;
				}

				break;
			}
		case MCU_STATE_BR_TX_INIT:
			{
				if (BR_last_packet == true)
				{
					BR_buffer[0] = PKT_Type_Update_Done;
					BR_buffer[1] = 0x01;
					BR_buffer[2] = PKT_Type_Update_Done;
					packet_size = 3;
				}
				else
				{
					BR_buffer[0] = PKT_Type_ACK;
					BR_buffer[1] = (ack_sequence >> 8) & 0xFF;
					BR_buffer[2] = ack_sequence & 0xFF;
					BR_buffer[3] = 0;
					packet_size = 4;
				}

				setSwitchMode(MODEM_BR_TX);
				setPA(PA_900, PA_TX_Bypass);

//				printf("mask: %d\n", SX1276Read(REG_LR_IRQFLAGSMASK));
//				printf("flags: %x\n", SX1276Read(REG_LR_IRQFLAGS));


				MCU_State = MCU_STATE_BR_TX;
				break;
			}
		case MCU_STATE_BR_TX:
			{
//				delay_ms(1000);
				sx1276Radio.Send(BR_buffer, packet_size);
//				TimerA1SetStart(500);
				MCU_State = MCU_STATE_BR_TX_WAIT;
				break;
			}
		case MCU_STATE_BR_TX_WAIT:
			{
				if (BR_last_packet == true)
				{
					MCU_State = MCU_STATE_UP_INIT;
				}
				else
				{
					MCU_State = MCU_STATE_BR_RX_INIT;
				}
				break;
			}
		case MCU_STATE_UP_INIT:
			{

				uint8_t flash_addr[3];
				flash_addr[0] = 0x00;
				flash_addr[1] = 0x00;
				flash_addr[2] = 0x00;
				uint8_t flash_data[1024];
				FlashReadData(flash_addr, flash_data, 1024);
				uint32_t ii;
				for(ii=0; ii<250; ii++)
				{
					printf("D: %x\n", flash_data[ii]);
				}
				MCU_State = MCU_STATE_UP_DONE;
				break;
			}
		case MCU_STATE_UP_DONE:
			{
//				printf("Done\n");
				break;
			}
		default:
			{
				MCU_State = MCU_STATE_BR_RX_INIT;
				break;
			}
		}
//		MAP_PCM_gotoLPM3();
//		sx1276Radio.Send(Buffer, 3);
//		delay_ms(500);
//		sx1276Radio.Rx(RX_TIMEOUT_VALUE);
//		op = SX1276Read(REG_OPMODE);
	}
}

void TX_RX_Transit(void)
{
//	GpioWrite(&AT86RF215.FPGA_GPIO1, false);
	AT86RF215SetState(RF_CMD_TXPREP);
	delay_ns(T_TRXOFF_TXPREP_ns);

//	uint8_t trim = 0x00;
//    while(1)
//    {
//    	if (trim > 0x0f)
//    		trim = 0x00;
//
//    	AT86RF215SetXOCTrim(trim);
//    	uint32_t regval = AT86RF215Read(REG_RF_XOC);
//    	printf("trim: %x\n: ", regval);
    	/////TX mode
//    	GpioWrite(&AT86RF215.FPGA_GPIO2, false);
//		AT86RF215SetState(RF_CMD_TXPREP);
//    	delay_ns(T_TRXOFF_TXPREP_ns*3);
    	AT86RF215SetState(RF_CMD_TX);
    	delay_us(T_TX_Start_Delay_us);
//    	GpioWrite(&AT86RF215.FPGA_GPIO2, true);

//    	delay_ms(10);
//    	GpioWrite(&AT86RF215.FPGA_GPIO2, false);
//    	AT86RF215SetState(RF_CMD_TXPREP);
//    	delay_us(T_TX_TXPREP_CMD_us);
//
//		AT86RF215SetState(RF_CMD_TRXOFF);
//    	delay_ns(T_TXPREP_TRXOFF_ns);

    	////RX mode
//    	delay_ns(T_TRXOFF_TRPREP_ns);
//    	AT86RF215SetState(RF_CMD_RX);
//    	delay_ns(T_TXPREP_RX_ns);

//    	GpioWrite(&AT86RF215.FPGA_GPIO1, true);

//    	delay_ms(1);
//    	trim += 0x01;

//    	AT86RF215SetState(RF_CMD_TXPREP);
//    	delay_ns(T_RX_TXPREP_ns);
//    }

    	while(1)
    	{
    		;
    	}
}

/**########################Variables and Types############################**/

/* AT86RF215 callbake functions */
void At86rf215_OnTxDone(void)
{
    return;
}
void At86rf215_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    return;
}
void At86rf215_OnTxTimeout(void)
{
    return;
}
void At86rf215_OnRxTimeout(void)
{
    return;
}
void At86rf215_OnRxError(void)
{
    return;
}
void At86rf215_FhssChangeChannel(uint8_t currentChannel)
{
    return;
}
void At86rf215_CadDone(bool channelActivityDetected)
{
    return;
}

/* SX1276 callbake functions */
void SX1276_OnTxDone(void)
{
    return;
}
void SX1276_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    return;
}
void SX1276_OnTxTimeout(void)
{
    return;
}
void SX1276_OnRxTimeout(void)
{
    return;
}
void SX1276_OnRxError(void)
{
    return;
}
void SX1276_FhssChangeChannel(uint8_t currentChannel)
{
    return;
}
void SX1276_CadDone(bool channelActivityDetected)
{
    return;
}
