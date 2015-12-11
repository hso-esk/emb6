/**
 * @file    crc.h
 * @date    16.11.2015
 * @author  PN
 * @brief   Cyclic Redundancy Checking (CRC) implementation based on example
 *          codes from TI
 */

#ifndef CRC_PRESENT
#define CRC_PRESENT

uint16_t crc_16(uint8_t *p_data, uint16_t len);
uint32_t crc_32(uint8_t *p_data, uint16_t len);

#endif /* CRC_PRESENT */
