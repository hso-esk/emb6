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

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       board_conf.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Definition of the Board Configuration.
 *
 *              The Board Configuration configures the underlying MCU and
 *              transceiver.
 */
#ifndef __BOARD_CONF_H__
#define __BOARD_CONF_H__

/*
 * --- Includes -------------------------------------------------------------*
 */


/*
 * --- Macro Definitions --------------------------------------------------- *
 */
/** Enable RF SPI */
#ifndef HAL_SUPPORT_RFSPI
#define HAL_SUPPORT_RFSPI                   FALSE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/* TODO add platform-specific SPIx configurations */

/* TODO add platform-specific RF_GPIOx configuration */

/** Dissable/Enable RF control 0 */
#ifndef HAL_SUPPORT_RFCTRL0
#define HAL_SUPPORT_RFCTRL0                 FALSE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/** Dissable/Enable RF control 1 */
#ifndef HAL_SUPPORT_RFCTRL1
#define HAL_SUPPORT_RFCTRL1                 FALSE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/** Dissable/Enable RF control 2 */
#ifndef HAL_SUPPORT_RFCTRL2
#define HAL_SUPPORT_RFCTRL2                 FALSE
#endif /* #ifndef HAL_SUPPORT_RFSPI */

/* TODO add platform-specific SLIPUART configuration */
#if defined(HAL_SUPPORT_SLIPUART)

#endif /* #if defined(HAL_SUPPORT_SLIPUART) */

/* TODO add number of platform-specific LEDs */

/** Number of supported LEDs */
#ifndef HAL_SUPPORT_LEDNUM
#define HAL_SUPPORT_LEDNUM                  ( 0 )
#endif /* #ifndef HAL_SUPPORT_LEDNUM */

/* TODO add platform-specific LEDs configuration */

/** Disable/Enable LED0 */
#ifndef HAL_SUPPORT_LED0
#define HAL_SUPPORT_LED0                    FALSE
#endif /* #ifndef HAL_SUPPORT_LED0 */

/** Disable/Enable LED1 */
#ifndef HAL_SUPPORT_LED1
#define HAL_SUPPORT_LED1                    FALSE
#endif /* #ifndef HAL_SUPPORT_LED1 */

/** Disable/Enable LED2 */
#ifndef HAL_SUPPORT_LED2
#define HAL_SUPPORT_LED2                    FALSE
#endif /* #ifndef HAL_SUPPORT_LED2 */


/*
 * --- Stack Macro Definitions ---------------------------------------------- *
 */
/** additional delay between consecutive iteration of emb6 process */
#define EMB6_PROC_DELAY                     ( 0 )


/*
 *  --- Global Functions Definition ------------------------------------------*
 */

/**
 * board_conf()
 *
 * \brief   Configure board.
 *
 *          This function is used to configure the specific board for usage
 *          with emb::6. Therefore e.g. the transceiver and the according
 *          layers will be selected.
 *
 * \param   p_ns  Pointer to global netstack struct.
 *
 * \return  0 on success or negative value on error.
 */
int8_t board_conf( s_ns_t* p_ns );

#endif /* __BOARD_CONF_H__ */

