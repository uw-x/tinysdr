/*
 _____  _                _____ ______ ______
|_   _|(_)              /  ___||  _  \| ___ \
  | |   _  _ __   _   _ \ `--. | | | || |_/ /
  | |  | || '_ \ | | | | `--. \| | | ||    /
  | |  | || | | || |_| |/\__/ /| |/ / | |\ \
  \_/  |_||_| |_| \__, |\____/ |___/  \_| \_|
                   __/ |
                  |___/
Description: TinySDR Utils

License: see LICENSE.TXT file include in the project

Maintainer: Mehrdad Hessar, Ali Najafi
*/
#include <utils.h>

/**#############################Functions#############################**/
void delay_ms(uint32_t msTime)
{
//    unsigned long cycles;
//    if (CLK_FREQ == CLK_FREQ_8M)
//        cycles = 8000;
//    else if (CLK_FREQ == CLK_FREQ_16M)
//        cycles = 16000;
//    else if (CLK_FREQ == CLK_FREQ_24M)
//        cycles = 24000;
//    else
//        cycles = 8000;

    int i;
    for (i=0; i<msTime; i++)
        __delay_cycles(CYCLES_mS);
}

void bitSet(uint8_t *value, uint8_t bit)
{
    uint8_t mask = (0x01 << bit);
    *value |= mask;
}

void bitClear(uint8_t *value, uint8_t bit)
{
    uint8_t mask = ~(0x01 << bit);
     *value &= mask;
}

uint8_t reverseBits(uint8_t num)
{
    uint8_t NO_OF_BITS = sizeof(num) * 8;
    uint8_t reverse_num = 0, i, temp;

    for (i = 0; i < NO_OF_BITS; i++)
    {
        temp = (num & (1 << i));
        if(temp)
            reverse_num |= (1 << ((NO_OF_BITS - 1) - i));
    }

    return reverse_num;
}

