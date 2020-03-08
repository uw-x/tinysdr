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
#include <ble.h>

uint8_t packet[BLE_PACKET_MAX];
uint8_t adver_addr[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

/**#######################Internal functions######################**/
void lfsr7_next(uint8_t* data);
uint32_t crc24(uint8_t* data, uint32_t length);
void lfsr7_whitening(uint8_t* data, uint8_t channel, uint32_t length);

uint32_t crc24(uint8_t* data, uint32_t length)
{
    uint32_t ii, jj, byte, crc;
    uint32_t next_bit;

    /* because of LSB */
    crc = (~CRC24_INIT) & CRC24_MASK;

    for(ii=0; ii<length; ii++)
    {
        byte = data[ii];

        for (jj=0; jj<8; jj++)
        {
        	next_bit = (crc ^ byte) & 0x01;

        	byte = byte >> 1;
        	crc = crc >> 1;
        	if (next_bit)
        	{
        		crc |= (1 << 23);
        		crc ^= CRC24_POLY;
        	}
        }
    }
    return crc;
}

void lfsr7_whitening(uint8_t* data, uint8_t channel, uint32_t length)
{
	/* seed is 7 bits */
	uint8_t seed = (channel | 0x40) & 0x7F;
	uint32_t ii, jj;
	uint8_t next_byte, lfsr_next_bit;

	for(ii=0; ii<length; ii++)
	{
		next_byte = data[ii];
		for(jj=0; jj<8; jj++)
		{
			lfsr_next_bit = seed & 0x01;
			if (lfsr_next_bit)
			{
				next_byte = next_byte ^ (0x01 << jj);
			}
			lfsr7_next(&seed);
		}
		data[ii] = next_byte;
	}
}

void lfsr7_next(uint8_t* data)
{
	uint8_t tmp, next_bit;
	tmp = *data;

    next_bit = tmp & 0x01;

    tmp = (tmp >> 1) & 0x7F;
    if (next_bit)
    {
    	tmp |= (next_bit << 6);
    	tmp ^= 0x04;
    }
	*data = tmp;
}

/**#######################External functions######################**/
uint8_t ble_gen_beacon(uint8_t* packet, uint8_t* adver_addr, uint8_t* message, uint8_t length, uint8_t channel)
{
	uint8_t ii;


	/* generate CRC */
	uint32_t crc;
	uint8_t crc_length;
	uint8_t pdu_length;
	uint8_t index = 0;

	crc_length = 2 + 6 + length;
	pdu_length = crc_length + 3;

	uint8_t *pdu;
	pdu = (uint8_t *) malloc(pdu_length);

	pdu[0] = 0x02;
	pdu[1] = 6 + length;

	for(ii=0; ii<6; ii++)
	{
		pdu[ii+2] = adver_addr[ii];
	}

	for(ii=0; ii<length; ii++)
	{
		pdu[ii+8] = message[ii];
	}

	crc = crc24(pdu, crc_length);


	/* add CRC to PDU */
	pdu[crc_length] = crc & 0xFF;
	pdu[crc_length + 1] = (crc >> 8) & 0xFF;
	pdu[crc_length + 2] = (crc >> 16) & 0xFF;

	/* whitening */
	lfsr7_whitening(pdu, channel, pdu_length);

	/* create packet */
	//pading, TODO: Should be removed later
	packet[0] = 0x00;
	index = index + 1;

	//TODO
	packet[1] = 0xAA;
	packet[2] = 0xD6;
	packet[3] = 0xBE;
	packet[4] = 0x89;
	packet[5] = 0x8E;

	index = index + 5;
	for (ii=0; ii<pdu_length; ii++)
	{
		packet[index + ii] = pdu[ii];
	}
	index = index + pdu_length;
	free(pdu);

	//pading, TODO: Should be removed later
	packet[index] = 0x00;
	index = index + 1;

	return index;
}

void ble_send(uint8_t* message, uint8_t message_len)
{
	uint8_t len = ble_gen_beacon(packet, adver_addr, message, message_len, BLE_CHANNEL_37);
	uint8_t ii;

	/* clear SPI */
	fpgaWrite(0x00);
	/* BLE packet */
	fpgaWrite(0xAA);
	/* packet length */
	fpgaWrite(len);

	for (ii=0; ii<len; ii++)
	{
		fpgaWrite(~packet[ii]);
	}
}
