/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Implements the basic functionalities for BLE beacon demo.

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#ifndef _APP_BLE_H_
#define _APP_BLE_H_

#include <stdint.h>
#include <stdlib.h>
#include <fpga/fpga.h>
#include <utils/utils.h>

/**########################Variables and Types############################**/
#define CRC24_POLY	0x5a6000

//init value for BLE is 0x555555
#define CRC24_INIT	0x555555
#define CRC24_MASK	0xFFFFFF

#define BLE_CHANNEL_36	0x24
#define BLE_CHANNEL_37	0x25

#define BLE_PACKET_MAX	100

/**#######################External functions######################**/
uint8_t ble_gen_beacon(uint8_t* packet, uint8_t* adver_addr, uint8_t* message, uint8_t length, uint8_t channel);
void ble_send(uint8_t* message, uint8_t message_len);


#endif
