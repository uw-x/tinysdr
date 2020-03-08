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
#ifndef _APP_TASK_H_
#define _APP_TASK_H_

/**#######################External functions######################**/
void task_ble(void);
void task_fpga_lora_tx(void);
void task_i2c(void);
void task_lora_test(void);
void task_ext_flash(void);
void task_power_measure(void);
void task_single_tone(void);

/* This is for just programming and working with FPGA. */
void task_fpga(void);

#endif
