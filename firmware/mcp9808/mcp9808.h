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
#ifndef _MCP9808_H_
#define _MCP9808_H_

#include <driverlib.h>
#include <libraries/i2c.h>
#include <settings.h>
#include <utils.h>

/**#############################Registers#############################**/
#define MCP9808_REG_CONFIG 			0x01	///< MCP9808 config register
#define MCP9808_REG_UPPER_TEMP 		0x02  	///< upper alert boundary
#define MCP9808_REG_LOWER_TEMP 		0x03  	///< lower alert boundery
#define MCP9808_REG_CRIT_TEMP 		0x04    ///< critical temperature
#define MCP9808_REG_AMBIENT_TEMP 	0x05 	///< ambient temperature
#define MCP9808_REG_MANUF_ID 		0x06   	///< manufacture ID
#define MCP9808_REG_DEVICE_ID 		0x07   	///< device ID
#define MCP9808_REG_RESOLUTION 		0x08	///< resolutin


#define MCP9808_REG_CONFIG_SHUTDOWN 	0x0100  ///< shutdown config
#define MCP9808_REG_CONFIG_CRITLOCKED 	0x0080 	///< critical trip lock
#define MCP9808_REG_CONFIG_WINLOCKED 	0x0040  ///< alarm window lock
#define MCP9808_REG_CONFIG_INTCLR 		0x0020  ///< interrupt clear
#define MCP9808_REG_CONFIG_ALERTSTAT 	0x0010  ///< alert output status
#define MCP9808_REG_CONFIG_ALERTCTRL 	0x0008  ///< alert output control
#define MCP9808_REG_CONFIG_ALERTSEL 	0x0004  ///< alert output select
#define MCP9808_REG_CONFIG_ALERTPOL 	0x0002  ///< alert output polarity
#define MCP9808_REG_CONFIG_ALERTMODE 	0x0001 	///< alert output mode


/**#############################External Functions#############################**/
uint16_t mcp9808_RegRead(uint8_t addr);
void mcp9808_RegWrite(uint8_t addr, uint16_t data);
void mcp9808_init(void);
float mcp9808_readTempC(void);
void mcp9808_setResolution(uint8_t res);
void mcp9808_shutdown(bool sw);
void mcp9808_wake(void);

#endif
