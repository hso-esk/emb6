/**
 * @file    crc.h
 * @date    16.11.2015
 * @author  PN
 * @brief   Cyclic Redundancy Checking (CRC) implementation based on example
 *          codes from TI
 */

#ifndef CRC_PRESENT
#define CRC_PRESENT

#define CRC16_POLY      (uint16_t)( 0x1021u )
#define CRC16_INIT      (uint16_t)( 0x0000u )

#define CRC32_POLY      (uint32_t)( 0x04c11db7u )
#define CRC32_INIT      (uint32_t)( 0xffffffffu )

uint16_t crc_16_update(uint16_t curr_crc, uint8_t byte);
uint32_t crc_32_update(uint32_t curr_crc, uint8_t byte);

#endif /* CRC_PRESENT */
