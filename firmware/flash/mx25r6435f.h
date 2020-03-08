/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Generic MX25R6435F Flash implementation

License: see LICENSE.TXT file include in the project

Maintainer:	Mehrdad Hessar, Ali Najafi
*/
#ifndef _MX25R6435F_H_
#define _MX25R6435F_H_

#include <settings.h>
#include <libraries/spi.h>
#include <libraries/gpio.h>
#include <libraries/board.h>

/**#############################CMD#############################**/
#define	FLASH_CMD_NOP		0x00
#define	FLASH_CMD_WRSR		0x01
#define	FLASH_CMD_PP		0x02
#define	FLASH_CMD_READ		0x03
#define	FLASH_CMD_WRDI		0x04
#define	FLASH_CMD_RDSR		0x05
#define	FLASH_CMD_WREN		0x06
#define FLASH_CMD_SE		0x20
#define	FLASH_CMD_RDSCUR	0x2B
#define	FLASH_CMD_RSTEN		0x66
#define	FLASH_CMD_RST		0x99
#define FLASH_CMD_RES		0xAB
#define FLASH_CMD_RDID		0x9F

/**#############################STATUS#############################**/
#define FLASH_Status_Mask_WIP	0x01
#define FLASH_Status_Mask_WEL	0x02
#define FLASH_Status_Mask_BP0	0x04
#define FLASH_Status_Mask_BP1	0x08
#define FLASH_Status_Mask_BP2	0x10
#define FLASH_Status_Mask_BP3	0x20
#define FLASH_Status_Mask_QE	0x40
#define FLASH_Status_Mask_SRWD	0x80


/**#############################Constants#############################**/
#define FLASH_Page_Size		256
#define FLASH_Block_Size	4096

/**#############################External Functions#############################**/
bool FlashInit(void);
uint8_t FlashTransactionRead(void);
void FlashTransactionReadBuffer(uint8_t *buffer, uint32_t size);
void FlashTransactionWriteBuffer(uint8_t *buffer, uint32_t size);
void FlashTransactionStart(uint8_t cmd);
void FlashTransactionFinish(void);
uint8_t FlashReadStatusReg(void);
uint8_t FlashReadSecurityReg(void);
void FlashWriteEnable(void);
bool FlashWritePage(uint32_t ind, uint8_t *page, uint32_t size);
void FlashEraseWritePage(uint32_t ind, uint8_t *page, uint32_t size);
void FlashReadData(uint8_t *addr, uint8_t *buffer, uint32_t size);
bool FlashEraseSector(uint32_t ind);
void FlashSoftwareReset(void);
void FlashMassWrite(uint32_t flashSize);
void FlashMassCheck(uint32_t flashSize);


#endif
