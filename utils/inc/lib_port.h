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
 * @file    lib_port.h
 * @date    30.09.2015
 * @author  PN
 */

#ifndef LIB_PORT_H_
#define LIB_PORT_H_


#include "bsp.h"
#define CPU_ENTER_CRITICAL()        bsp_enterCritical()
#define CPU_EXIT_CRITICAL()         bsp_exitCritical()
#define LED_RX_ON()                 bsp_led(E_BSP_LED_0, E_BSP_LED_ON)
#define LED_RX_OFF()                bsp_led(E_BSP_LED_0, E_BSP_LED_OFF)
#define LED_TX_ON()                 bsp_led(E_BSP_LED_1, E_BSP_LED_ON)
#define LED_TX_OFF()                bsp_led(E_BSP_LED_1, E_BSP_LED_OFF)
#define LED_SCAN_ON()               bsp_led(E_BSP_LED_2, E_BSP_LED_ON)
#define LED_SCAN_OFF()              bsp_led(E_BSP_LED_2, E_BSP_LED_OFF)
#define LED_MEAS_ON()               bsp_led(E_BSP_LED_4, E_BSP_LED_ON)
#define LED_MEAS_OFF()              bsp_led(E_BSP_LED_4, E_BSP_LED_OFF)
#define LED_ERROR()                 bsp_led(E_BSP_LED_4, E_BSP_LED_ON)
#define LED_ERR_ON()                bsp_led(E_BSP_LED_4, E_BSP_LED_ON)
#define LED_ERR_OFF()               bsp_led(E_BSP_LED_4, E_BSP_LED_OFF)

#endif /* LIB_PORT_H_ */
