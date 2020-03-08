/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: TinySDR Radio General Implementation

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#ifndef _RADIO_H_
#define _RADIO_H_

/**#############################Types and Variables#############################**/
typedef struct {
    void    ( *TxDone )( void );
    void    ( *TxTimeout )( void );
    void    ( *RxDone )( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
    void    ( *RxTimeout )( void );
    void    ( *RxError )( void );
    void ( *FhssChangeChannel )( uint8_t currentChannel );
    void ( *CadDone ) ( bool channelActivityDetected );
}RadioEvents_t;



#endif
