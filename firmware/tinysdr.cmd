/******************************************************************************
*
* Copyright (C) 2012 - 2017 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Default linker command file for Texas Instruments MSP432P401R
*
* File creation date: 12/06/17
*
*****************************************************************************/
/* Suppress warnings and errors:                                            */
/* #10199-D CRC table operator (crc_table_for_<>) ignored:
    CRC table operator cannot be associated with empty output section       */
--diag_suppress=10199

--retain=flashMailbox

MEMORY
{
    MAIN       (RX) : origin = 0x00000000, length = 0x00040000
    INFO       (RX) : origin = 0x00200000, length = 0x00004000
#ifdef  __TI_COMPILER_VERSION__
#if     __TI_COMPILER_VERSION__ >= 15009000
    ALIAS
    {
    SRAM_CODE  (RWX): origin = 0x01000000
    SRAM_DATA  (RW) : origin = 0x20000000
    } length = 0x00010000
#else
    /* Hint: If the user wants to use ram functions, please observe that SRAM_CODE             */
    /* and SRAM_DATA memory areas are overlapping. You need to take measures to separate       */
    /* data from code in RAM. This is only valid for Compiler version earlier than 15.09.0.STS.*/ 
    SRAM_CODE  (RWX): origin = 0x01000000, length = 0x00010000
    SRAM_DATA  (RW) : origin = 0x20000000, length = 0x00010000
#endif
#endif
}

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* A heap size of 1024 bytes is recommended when you plan to use printf()    */
/* for debug output to the console window.                                   */
/*                                                                           */
/* --heap_size=1024                                                          */
/* --stack_size=512                                                          */
/* --library=rtsv7M4_T_le_eabi.lib                                           */

/* Section allocation in memory */

SECTIONS
{
#ifndef gen_crc_table
    .intvecs:   > 0x00000000
    .text   :   > MAIN
    .const  :   > MAIN
    .cinit  :   > MAIN
    .pinit  :   > MAIN
    .init_array   :     > MAIN
    .binit        : {}  > MAIN

    /* The following sections show the usage of the INFO flash memory        */
    /* INFO flash memory is intended to be used for the following            */
    /* device specific purposes:                                             */
    /* Flash mailbox for device security operations                          */
    .flashMailbox : > 0x00200000
    /* TLV table for device identification and characterization              */
    .tlvTable     : > 0x00201000
    /* BSL area for device bootstrap loader                                  */
    .bslArea      : > 0x00202000
#else
    .intvecs:   > 0x00000000, crc_table(crc_table_for_intvecs)
    .text   :   > MAIN, crc_table(crc_table_for_text)
    .const  :   > MAIN, crc_table(crc_table_for_const)
    .cinit  :   > MAIN, crc_table(crc_table_for_cinit)
    .pinit  :   > MAIN, crc_table(crc_table_for_pinit)
    .init_array   :     > MAIN, crc_table(crc_table_for_init_array)
    .binit        : {}  > MAIN, crc_table(crc_table_for_binit)

    /* The following sections show the usage of the INFO flash memory        */
    /* INFO flash memory is intended to be used for the following            */
    /* device specific purposes:                                             */
    /* Flash mailbox for device security operations                          */
    .flashMailbox : > 0x00200000, crc_table(crc_table_for_flashMailbox)
    /* TLV table for device identification and characterization              */
    /* This one is read only memory in flash - generate no CRC               */
    .tlvTable     : > 0x00201000
    /* BSL area for device bootstrap loader                                  */
    .bslArea      : > 0x00202000, crc_table(crc_table_for_bslArea)
    .TI.crctab    : > MAIN
#endif

    .vtable :   > 0x20000000
    .data   :   > SRAM_DATA
    .bss    :   > SRAM_DATA
    .sysmem :   > SRAM_DATA
    .stack  :   > SRAM_DATA (HIGH)
    flashBuff:  > 0x00002000

#ifdef  __TI_COMPILER_VERSION__
#if     __TI_COMPILER_VERSION__ >= 15009000
    .TI.ramfunc : {} load=MAIN, run=SRAM_CODE, table(BINIT)
#endif
#endif
}

/* Symbolic definition of the WDTCTL register for RTS */
WDTCTL_SYM = 0x4000480C;

