/**
 * @file    crc.h
 * @date    16.11.2015
 * @author  PN
 * @brief   Cyclic Redundancy Checking (CRC) implementation based on example
 *          codes from TI
 */

#include "lib_crc.h"



/*******************************************************************************
*   @fn         crc16Calc
*
*   @brief      A CRC-16/CCITT implementation optimized for small code size.
*
*               The function should be called once for each byte in the data the
*               CRC is to be performed on. For the invocation on the first byte
*               the value CRC_INIT should be given for _crcReg_. The value
*               returned is the CRC-16 of the data supplied so far.
*
*   @param      data
*                   The data to perform the CRC-16 operation on
*
*               curr_val
*                   The current value of the CRC register
*
*   @return     The updated value of the CRC16 register
*/
uint16_t crc16Calc(uint8_t data, uint16_t curr_val)
{
    uint8_t ix;
    for (ix = 0; ix < 8; ix++) {

        if (((curr_val & 0x8000) >> 8) ^ (data & 0x80)) {
            curr_val = (curr_val << 1) ^ CRC16_POLY;
        } else {
            curr_val = (curr_val << 1);
        }
        data <<= 1;
    }

#if 0     /* Example to generate CRC for the data packet */
    checksum = CRC_INIT;
    for(i = 0; i <= crc_data_len; i++) {
        checksum = calcCRC(crc_data_buffer[i], checksum);
    }
#endif

    return curr_val;
}
