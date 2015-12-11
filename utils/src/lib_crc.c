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
