/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: Generic MCP9808 implementation

License: see LICENSE.TXT file include in the project

Maintainer:	Mehrdad Hessar, Ali Najafi
*/
#include "mcp9808.h"

/**#############################External Functions#############################**/
void mcp9808_init(void)
{
	I2C_init();
	uint16_t MID, DevID;

	mcp9808_RegRead(MCP9808_REG_MANUF_ID);
	MID = mcp9808_RegRead(MCP9808_REG_MANUF_ID);
	if (MID != 0x0054)
		printf("MCP9808 Error!");

    DevID = mcp9808_RegRead(MCP9808_REG_DEVICE_ID);

    if (DevID != 0x0400)
    		printf("MCP9808 Error!");

    mcp9808_setResolution(0x03);
    mcp9808_RegWrite(MCP9808_REG_CONFIG, 0x0001);
}

void mcp9808_shutdown(bool sw)
{
	uint16_t config = mcp9808_RegRead(MCP9808_REG_CONFIG);
	if (sw == true)
	{
		config = config | (MCP9808_REG_CONFIG_SHUTDOWN);
	}
	else
	{
		config = config & (~MCP9808_REG_CONFIG_SHUTDOWN);
	}
	mcp9808_RegWrite(MCP9808_REG_CONFIG, config);
}

void mcp9808_wake(void)
{
	mcp9808_shutdown(false);
	delay_ms(250);
}

float mcp9808_readTempC(void)
{
	uint16_t ambientTemp;
	float temp;

	ambientTemp = mcp9808_RegRead(MCP9808_REG_AMBIENT_TEMP);
//	printf("tt: %x\n", ambientTemp);
	temp = ambientTemp & 0x0FFF;
	temp = temp / 16.0;
	if (ambientTemp & 0x1000)
		temp = temp - 256;

	return temp;
}

uint16_t mcp9808_RegRead(uint8_t addr)
{
	I2C_send8(MCP9808_I2C_Module, addr);
	I2C_EUSCI_B_Receive16(MCP9808_I2C_Module);

	I2C_send8(MCP9808_I2C_Module, addr);
	return I2C_EUSCI_B_Receive16(MCP9808_I2C_Module);
}

void mcp9808_setResolution(uint8_t res)
{
	res = res & 0x03;
	I2C_EUSCI_B_Write8(MCP9808_I2C_Module, MCP9808_REG_RESOLUTION, res);
}

void mcp9808_RegWrite(uint8_t addr, uint16_t data)
{
	I2C_EUSCI_B_Write16(MCP9808_I2C_Module, addr, data);
}


