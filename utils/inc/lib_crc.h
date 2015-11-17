/**
 * @file    crc.h
 * @date    16.11.2015
 * @author  PN
 * @brief   Cyclic Redundancy Checking (CRC) implementation based on example
 *          codes from TI
 */

#ifndef CRC_PRESENT
#define CRC_PRESENT


#include <stdint.h>


#define CRC16_POLY      (uint16_t)( 0x8005 )
#define CRC_INIT        (uint16_t)( 0xFFFF )


uint16_t crc16Calc(uint8_t data, uint16_t curr_val);

#endif /* CRC_PRESENT */
