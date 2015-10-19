/*
 * lib_port.h
 *
 *  Created on: 30.09.2015
 *      Author: nphuong
 */

#ifndef LIB_PORT_H_
#define LIB_PORT_H_

#ifdef IAR_COMPILER
#include "bsp_lums.h"
#define CPU_ENTER_CRITICAL()        bsp_enterCritical()
#define CPU_EXIT_CRITICAL()         bsp_exitCritical()
#define LED_RX_ON()                 bsp_led(E_BSP_LED_ORANGE, E_BSP_LED_ON)
#define LED_RX_OFF()                bsp_led(E_BSP_LED_ORANGE, E_BSP_LED_OFF)
#define LED_TX_ON()                 bsp_led(E_BSP_LED_YELLOW, E_BSP_LED_ON)
#define LED_TX_OFF()                bsp_led(E_BSP_LED_YELLOW, E_BSP_LED_OFF)
#define LED_SCAN_ON()               bsp_led(E_BSP_LED_GREEN, E_BSP_LED_ON)
#define LED_SCAN_OFF()              bsp_led(E_BSP_LED_GREEN, E_BSP_LED_OFF)
#define LED_MEAS_ON()               bsp_led(E_BSP_LED_RED, E_BSP_LED_ON)
#define LED_MEAS_OFF()              bsp_led(E_BSP_LED_RED, E_BSP_LED_OFF)
#define LED_ERROR()                 bsp_led(E_BSP_LED_RED, E_BSP_LED_ON)



#else
#define CPU_ENTER_CRITICAL()
#define CPU_EXIT_CRITICAL()
#define LED_RX_ON()
#define LED_RX_OFF()
#define LED_TX_ON()
#define LED_TX_OFF()
#define LED_SCAN_ON()
#define LED_SCAN_OFF()
#define LED_MEAS_ON()
#define LED_MEAS_OFF()
#define LED_ERROR()
#endif


#endif /* LIB_PORT_H_ */
