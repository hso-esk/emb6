/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*============================================================================*/

/**
 * @file    crc.h
 * @date    16.11.2015
 * @author  PN
 * @brief   Cyclic Redundancy Checking (CRC) implementations
 */

#include <stdint.h>
#include "lib_crc.h"


#define CRC16_POLY      (uint16_t)( 0x1021u )
#define CRC16_INIT      (uint16_t)( 0x0000u )
#define CRC32_INIT      (uint32_t)( 0xFFFF0000u )


/**
 * @brief   Update 16-bit CRC when a byte is added
 * @param   curr_crc    Current CRC value
 * @param   byte        Value of byte to add
 * @return  Updated CRC value
 */
static uint16_t crc16_update(uint16_t curr_crc, uint8_t byte)
{
    uint8_t i;

    curr_crc = curr_crc ^ byte << 8;
    i = 8;
    do {
        if (curr_crc & 0x8000) {
            curr_crc = curr_crc << 1 ^ CRC16_POLY;
        }
        else {
            curr_crc = curr_crc << 1;
        }
    } while (--i);

    return curr_crc;
}

/**
 * @brief   Update 32-bit CRC when a byte is added
 * @param   curr_crc    Current CRC value
 * @param   byte        Value of byte to add
 * @return  Updated CRC value
 */
static uint32_t crc32_update(uint32_t curr_crc, uint8_t byte)
{
    /* todo missing implementation */
    return 0x0a0b0c0d;
}


/**
 * @brief   Calculate 16-bit ITU-T CRC over an array
 * @param   p_data  pointer to array over which CRC is calculated
 * @param   len     length of the array
 * @return  calculated CRC
 */
uint16_t crc_16(uint8_t *p_data, uint16_t len)
{
    uint16_t ix;
    uint16_t crc_res;

    crc_res = 0;
    for (ix = 0; ix < len; ix++) {
        crc_res = crc16_update(crc_res, p_data[ix]);
    }

    return crc_res;
}


/**
 * @brief   Calculate 16-bit ITU-T CRC over an array
 * @param   p_data  pointer to array over which CRC is calculated
 * @param   len     length of the array
 * @return  calculated CRC
 */
uint32_t crc_32(uint8_t *p_data, uint16_t len)
{
    uint32_t crc_res;
    uint16_t ix;

    crc_res = CRC32_INIT;
    for (ix = 0; ix < len; ix++) {
        crc_res = crc32_update(crc_res, p_data[ix]);
    }
    return crc_res;
}
