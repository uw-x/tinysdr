/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Generic AT86RF215 driver implementation

License: see LICENSE.TXT file include in the project

Maintainers: 	Mehrdad Hessar, Ali Najafi
*/
#ifndef _AT86RF215_H_
#define _AT86RF215_H_

#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "at86rf215Regs.h"
#include <settings.h>
#include <libraries/spi.h>
#include <libraries/i2c.h>
#include <utils.h>
#include <radio.h>

/**########################Variables and Types############################**/
/*! Radio driver supported modems */
typedef enum {
    MODEM_09 = 0,
	MODEM_BR_TX,
	MODEM_BR_RX,
	MODEM_24,
}At86rf215_RadioModems_t;

/* Radio driver internal state machine states definition */
typedef enum {
    RF_TRXOFF = 0,
    RF_TXPREP,
    RF_TX,
    RF_RX,
    RF_TRANSITION,
    RF_RESET,
} At86rf215_RadioState_t;

/* Hardware IO IRQ callback function definition */
typedef void ( DioIrqHandler )( void );

/* AT86RF215 definitions */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP1                                  99.1821289063
#define FREQ_STEP2                                  198.364257813
#define FREQ_STEP3                                  396.728515625

/* Calibration */
/* Number of trim loops */
#define TRIM_LOOPS                          7
/* Number of sufficient measurements */
#define NUM_SUFFICIENT_MEASUREMENTS         3
/* Narrow trim threshold values */
#define NARROW_TRIM_THRESHOLD               2

/* Radio hardware and global parameters */
typedef enum {
    PHY_OFF = 0,
    PHY_FSK,
    PHY_OFDM,
    PHY_OQPSK,
} BBC_Phy_t;

typedef struct
{
    bool                        channelComplient;
    bool                        CTX;
    At86rf215_RadioModems_t               Modem;	//0->900MHz, 1->2.4GHz
    At86rf215_RadioState_t                State;
    uint32_t                    Channel;
    uint8_t                     Power;
    uint8_t                     IRQM;				//RF interrupt enables
    uint8_t                     TXSR;				//RF transmitter sampling frequency
    uint8_t                     RXSR;				//RF RECEIVER sampling frequency
    uint8_t                     RXCUTOFF;			//RF RECEIVER CUTOFF frequency
    uint8_t                     RXBW;				//RF RECEIVER BW
    uint8_t                     TXCUTOFF;			//RF TRANSMITTER CUTOFF frequency
}RF_Settings_t;

typedef struct
{
    BBC_Phy_t                   Phy;
//    CHPM_t                      CHPM;
    uint8_t                     BBEN_PT;
    uint8_t                     IRQM;		//Baseband interrupt enables
    bool                        directMod;
    bool                        dataWhite;
}BBC_Settings_t;

typedef struct AT86RF215_s
{
    Gpio_t              Reset;
    Gpio_t              IRQ;
    Spi_t               Spi;
    RF_Settings_t       RF_Settings;
    BBC_Settings_t      BBC_Settings;
    bool                Continuous;
}AT86RF215_t;

typedef struct
{
    int8_t   Power;
//    uint32_t Bandwidth;
//    uint32_t Datarate;
//    bool     LowDatarateOptimize;
//    uint8_t  Coderate;
//    uint16_t PreambleLen;
//    bool     FixLen;
//    uint8_t  PayloadLen;
//    bool     CrcOn;
//    bool     FreqHopOn;
//    uint8_t  HopPeriod;
//    bool     IqInverted;
//    bool     RxContinuous;
//    uint32_t TxTimeout;
}Radio09Settings_t;

struct At86rf215_Radio_s
{
    void    ( *Init )( RadioEvents_t *events );
    At86rf215_RadioState_t ( *GetStatus )( void );
    void    ( *SetModem )( At86rf215_RadioModems_t modem );
    void    ( *SetChannel )( uint32_t freq );
    bool    ( *IsChannelFree )( At86rf215_RadioModems_t modem, uint32_t freq, int16_t rssiThresh );
    uint32_t ( *Random )( void );
    void    ( *SetRxConfig )( At86rf215_RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint32_t bandwidthAfc, uint16_t preambleLen,
                              uint16_t symbTimeout, bool fixLen,
                              uint8_t payloadLen,
                              bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                              bool iqInverted, bool rxContinuous );
    void    ( *SetTxConfig )( At86rf215_RadioModems_t modem, int8_t power, uint32_t fdev,
                              uint32_t bandwidth, uint32_t datarate,
                              uint8_t coderate, uint16_t preambleLen,
                              bool fixLen, bool crcOn, bool FreqHopOn,
                              uint8_t HopPeriod, bool iqInverted, uint32_t timeout );
    bool    ( *CheckRfFrequency )( uint32_t frequency );
    uint32_t  ( *TimeOnAir )( At86rf215_RadioModems_t modem, uint8_t pktLen );
    void    ( *Send )( uint8_t *buffer, uint8_t size );
    void    ( *Sleep )( void );
    void    ( *Standby )( void );
    void    ( *Rx )( uint32_t timeout );
    void    ( *StartCad )( void );
    int16_t ( *Rssi )( At86rf215_RadioModems_t modem );
    void    ( *Write )( uint16_t addr, uint8_t data );
    uint8_t ( *Read )( uint16_t addr );
    void    ( *WriteBuffer )( uint16_t addr, uint8_t *buffer, uint8_t size );
    void    ( *ReadBuffer )( uint16_t addr, uint8_t *buffer, uint8_t size );
    void ( *SetMaxPayloadLength )( At86rf215_RadioModems_t modem, uint8_t max );
};


extern const struct At86rf215_Radio_s At86rf215Radio;


/**########################External Function############################**/
void AT86RF215Init( RadioEvents_t *events );
At86rf215_RadioState_t AT86RF215GetStatus( void );
void AT86RF215SetModem( At86rf215_RadioModems_t modem );
void AT86RF215SetChannel( uint32_t freq );
bool AT86RF215IsChannelFree( At86rf215_RadioModems_t modem, uint32_t freq, int16_t rssiThresh );
uint32_t AT86RF215Random( void );
void AT86RF215RxSetConfig( At86rf215_RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                         bool iqInverted, bool rxContinuous );

void AT86RF215TxSetConfig( At86rf215_RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool FreqHopOn,
                        uint8_t HopPeriod, bool iqInverted, uint32_t timeout );

uint32_t AT86RF215GetTimeOnAir( At86rf215_RadioModems_t modem, uint8_t pktLen );
void AT86RF215Send( uint8_t *buffer, uint8_t size );
void AT86RF215SetSleep( void );
void AT86RF215SetStby( void );
void AT86RF215RxSet( uint32_t timeout );
void AT86RF215StartCad( void );
int16_t AT86RF215ReadRssi( At86rf215_RadioModems_t modem );
void AT86RF215Write( uint16_t addr, uint8_t data );
uint8_t AT86RF215Read( uint16_t addr );
void AT86RF215WriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size);
void AT86RF215ReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size);
void AT86RF215WriteFifo( uint8_t *buffer, uint8_t size );
void AT86RF215SetMaxPayloadLength( At86rf215_RadioModems_t modem, uint8_t max );
void AT86RF215Reset( void );
void AT86RF215OnDio0Irq(void);
void AT86RF215SetOpMode( uint8_t mode );
void AT86RF215SetInfMode( uint8_t mode );
void AT86RF215SetPHYType(uint8_t BBEN_PT);
void AT86RF215SetRFIntr(uint8_t INTR_SET);
void AT86RF215SetBBIntr(uint8_t INTR_SET);
uint8_t AT86RF215GetState(void);
void AT86RF215SetState(uint8_t state);
void AT86RF215SetRFMode(uint8_t mode);\
void AT86RF215SetIQRX(void);
void AT86RF215IRQInit(void);\
void AT86RF215RxSetBW(uint8_t BW);
void AT86RF215RxSetSR(uint8_t RXSR);
void AT86RF215RxSetCutOff(uint8_t RXCUTOFF);
void AT86RF215RxSetIQ(uint32_t freq);
void AT86RF215SetCWSingleTone(uint32_t freq);
void AT86RF215Set09CWSingleToneTest(void);
void AT86RF215TxSetPwr(uint8_t PWR);
void AT86RF215TxSetPAC(uint8_t PAC);
void AT86RF215TxSetFrameLength(uint16_t FrameLen);
void AT86RF215TxSetContinuous(bool CTX);
void AT86RF215TxSetSR(uint8_t TXSR);
void AT86RF215TxSetDirectMod(bool DM);
void AT86RF215TxSetDataWhite(bool DW);
void AT86RF215TxSetCutOff(uint8_t TXCUTOFF);
void AT86RF215TxSetIQ(uint32_t freq);
void AT86RF215TxSet( uint32_t timeout );
void AT86RF215SetRxChannel(uint32_t freq);
void AT86RF215RxSetIFS(uint8_t IFS);
void AT86RF215SetTXCI(uint8_t txci);
void AT86RF215SetTXCQ(uint8_t txcq);
void AT86RF215SetCLKO(uint8_t clko);
void AT86RF215TxSetPAVC(uint8_t PAVC);
void AT86RF215RxSetBBFSK(uint32_t freq);
void AT86RF215Initialize(uint32_t freq);
void AT86RF215SetTXBBFSK(uint32_t freq);
void AT86RF215SetXOCTrim(uint8_t trim);

#endif
