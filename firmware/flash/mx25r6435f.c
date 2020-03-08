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
#include "mx25r6435f.h"

/**#############################External Functions#############################**/
bool FlashInit(void)
{
	uint8_t flash_id[3];

	FlashTransactionStart(FLASH_CMD_RDID);
	FlashTransactionReadBuffer(flash_id, 3);
	FlashTransactionFinish();

	if((flash_id[0] != 0xc2) | (flash_id[1] != 0x28) | (flash_id[2] != 0x17))
	{
		return false;
	}
	return true;
}

uint8_t FlashTransactionRead(void)
{
    uint8_t data;
    FlashTransactionReadBuffer(&data, 1);
    return data;
}

void FlashTransactionStart(uint8_t cmd)
{
	GpioWrite(&TinySDR.FLASH.SPI.NSS, 0);
	flashSpiInOut(cmd);
}

void FlashTransactionReadBuffer(uint8_t *buffer, uint32_t size)
{
	uint32_t ii;
    for(ii = 0; ii<size; ii++)
    {
        buffer[ii] = flashSpiInOut(0);
    }
}

void FlashTransactionWriteBuffer(uint8_t *buffer, uint32_t size)
{
	uint32_t ii;
    for(ii = 0; ii<size; ii++)
    {
        flashSpiInOut(buffer[ii]);
    }
}

void FlashTransactionFinish(void)
{
	GpioWrite(&TinySDR.FLASH.SPI.NSS, 1);
}

void FlashWriteEnable(void)
{
	FlashTransactionStart(FLASH_CMD_WREN);
	FlashTransactionFinish();
}

uint8_t FlashReadStatusReg(void)
{
	uint8_t reg;
	FlashTransactionStart(FLASH_CMD_RDSR);
	reg = FlashTransactionRead();
	FlashTransactionFinish();
	return reg;
}

uint8_t FlashReadSecurityReg(void)
{
	uint8_t reg;
	FlashTransactionStart(FLASH_CMD_RDSCUR);
	reg = FlashTransactionRead();
	FlashTransactionFinish();
	return reg;
}

bool FlashWritePage(uint32_t ind, uint8_t *page, uint32_t size)
{
	uint8_t s_reg;
	uint8_t addr[3];

	addr[0] = (ind >> 16) & 0xFF;
	addr[1] = (ind >> 8) & 0xFF;
	addr[2] = ind & 0xFF;

	/* check last write */
	s_reg = FlashReadStatusReg();
	while(s_reg & FLASH_Status_Mask_WIP)
	{
		delay_us(1);
		s_reg = FlashReadStatusReg();
	}

	/* Enable write */
	FlashWriteEnable();
	/* check write enable */
	s_reg = FlashReadStatusReg();
	if (s_reg & FLASH_Status_Mask_WEL)
	{
		//Send Address
		FlashTransactionStart(FLASH_CMD_PP);
		FlashTransactionWriteBuffer(addr, 3);
		//Send Data
		FlashTransactionWriteBuffer(page, size);
		FlashTransactionFinish();
		return true;
	}
	else
	{
//		printf("page WE\n");
		return false;
	}
}


bool FlashEraseSector(uint32_t ind)
{
	uint8_t sReg;
	uint8_t addr[3];

	addr[0] = (ind >> 16) & 0xFF;
	addr[1] = (ind >> 8) & 0xFF;
	addr[2] = ind & 0xFF;

	/* check write in progress */
	sReg = FlashReadStatusReg();
	while(sReg & FLASH_Status_Mask_WIP)
	{
		delay_us(1);
		sReg = FlashReadStatusReg();
	}

	/* check protection */
	sReg = FlashReadStatusReg();
//	printf("protect: %x\n", sReg);
	if(sReg & (FLASH_Status_Mask_BP0 | FLASH_Status_Mask_BP1 | FLASH_Status_Mask_BP2 | FLASH_Status_Mask_BP3))
	{
		return false;
	}
	else
	{
		FlashWriteEnable();
		sReg = FlashReadStatusReg();
		if (sReg & FLASH_Status_Mask_WEL)
		{
			FlashTransactionStart(FLASH_CMD_SE);
			FlashTransactionWriteBuffer(addr, 3);
			FlashTransactionFinish();
			return true;
		}
		else
		{
			return false;
		}
	}
}

void FlashEraseWritePage(uint32_t ind, uint8_t *page, uint32_t size)
{
	bool status;
	status = false;

	if((ind % FLASH_Block_Size) == 0)
	{
		while(status == false)
			status = FlashEraseSector(ind);
	}
	status = false;

	while(status == false)
		status = FlashWritePage(ind, page, size);
}

void FlashReadData(uint8_t *addr, uint8_t *buffer, uint32_t size)
{
	uint8_t sReg;

	sReg = FlashReadStatusReg();
	while(sReg & FLASH_Status_Mask_WIP)
	{
		sReg = FlashReadStatusReg();
	}
	FlashTransactionStart(FLASH_CMD_READ);
	FlashTransactionWriteBuffer(addr, 3);
	FlashTransactionReadBuffer(buffer, size);
	FlashTransactionFinish();
}


void FlashSoftwareReset(void)
{
	FlashTransactionStart(FLASH_CMD_RSTEN);
	FlashTransactionFinish();
	FlashTransactionStart(FLASH_CMD_RST);
	FlashTransactionFinish();
}

void FlashMassWrite(uint32_t flashSize)
{
	uint32_t ind;
	uint32_t ii;
	uint8_t dataChunk[256];

	ind = 0;
	while(ind < flashSize)
	{

		/* update data */
		for(ii=0; ii<FLASH_Page_Size; ii++)
		{
			if (ind + ii < flashSize)
			{
				dataChunk[ii] = flash[ind + ii];
			}
			else
			{
				dataChunk[ii] = 0xFF;
			}
		}

		FlashEraseWritePage(ind, dataChunk, FLASH_Page_Size);

		/* update index */
		ind = ind + 256;
	}
}

void FlashMassCheck(uint32_t flashSize)
{
	uint32_t ind;
	uint32_t ii;
	uint8_t addr[3];
	uint8_t dataChunk[256];
	ind = 0;

	while(ind < flashSize)
	{
		/* update address */
		addr[0] = (ind >> 16) & 0xFF;
		addr[1] = (ind >> 8) & 0xFF;
		addr[2] = ind & 0xFF;

		#ifdef DEBUG
		if ((ind % 4096) == 0)
			printf("sec: %x\n", ind);
		#endif
		/* read page */
		FlashReadData(addr, dataChunk, 256);

		for(ii=0; ii<256; ii++)
		{
			if (ind + ii < flashSize)
			{
				if (dataChunk[ii] != flash[ind + ii])
				{
					printf("ind: %x\tii: %x\n", ind, ii);
					printf("m: %x\t orig:\t%x\n", dataChunk[ii], flash[ind + ii]);
				}
			}
			else
			{
				if (dataChunk[ii] != 0xFF)
				{
					printf("ind: %x\tii: %x\n", ind, ii);
					printf("m: %x\t orig:\t%x\n", dataChunk[ii], 0xFF);
				}
			}
			#ifdef DEBUG
			printf("m: %x\t orig:\t%x\n", dataChunk[ii], flash[ind + ii]);
			#endif
		}

		/* update index */
		ind = ind + 256;
	}
}

