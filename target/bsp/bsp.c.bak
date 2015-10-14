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
/**  \addtogroup emb6
 *      @{
 *      \addtogroup bsp Board Support Package
 *   @{
 */
/*============================================================================*/
/*! \file   bsp.c

    \author Artem Yushev, 

    \brief  Board support package implementation.

   \version 0.0.1
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "emb6_conf.h"
#include "bsp.h"
#include "board_conf.h"

#include "etimer.h"
#include "random.h"
#include "evproc.h"
/*==============================================================================
                                     MACROS
 =============================================================================*/
#define     LOGGER_ENABLE        LOGGER_BSP
#include    "logger.h"


/*==============================================================================
                                     ENUMS
 =============================================================================*/

/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
static    uint8_t    c_wtgStop = 1;
static    struct    etimer st_ledsTims[LEDS_SUPPORTED];
/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static    void _bsp_ledCallback(c_event_t c_event, p_data_t p_data);
/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/

static    void _bsp_ledCallback(c_event_t c_event, p_data_t p_data)
{
    uint8_t j;
    for (j = 0; j < LEDS_SUPPORTED; j++)
    {
        if (etimer_expired(&st_ledsTims[j]))
        {
            if ((&(st_ledsTims[j])) == (struct etimer *)p_data){
                // Timer has been found , so we can turn it off.
                hal_ledOff(j);
                etimer_stop((struct etimer *)p_data);
                break;
            }
        }
    }
        // Find etimer which was fired in the list
}
/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/


/*============================================================================*/
/*  bsp_init()                                                                */
/*============================================================================*/
uint8_t bsp_init (s_ns_t * ps_ns)
{
    /* Initialize hardware */
    if (!hal_init())
        return 0;

    /* Configure board */
    if (!board_conf(ps_ns))
        return 0;

    random_init(hal_getrand());

    /* Normal exit*/
    return 1;
}/* bsp_init() */

/*============================================================================*/
/*  bsp_start()                                                               */
/*============================================================================*/
void bsp_start (void)
{

}/* bsp_start() */

/*============================================================================*/
/*  bsp_entry()                                                               */
/*============================================================================*/
void bsp_entry (void)
{
    /* Nothing to do */
}/* bsp_entry() */

/*============================================================================*/
/*  bsp_entry()                                                               */
/*============================================================================*/
int bsp_getChar (void)
{
    /* Return the next character received using the console UART */
    /* Nothing to do */
    return 0;
} /* bsp_getChar() */

/*============================================================================*/
/*  bsp_led()                                                               */
/*============================================================================*/
uint16_t bsp_led(en_bspLedColor_t ui_led, en_bspLedAction_t en_ledAction)
{
    switch (en_ledAction) {
    case E_BSP_LED_ON:
        hal_ledOn(ui_led);
        break;
    case E_BSP_LED_OFF:
        hal_ledOff(ui_led);
        break;
    case E_BSP_LED_TOGGLE:
        hal_ledOn(ui_led);
        etimer_set(&st_ledsTims[ui_led],20, _bsp_ledCallback);

        break;
    default:
        break;
    }
    return 0;
} /* bsp_led() */

/*============================================================================*/
/*  bsp_extIntInit()                                                          */
/*============================================================================*/
uint8_t bsp_pin(en_bspPinAction_t e_pinAct, void * p_pin)
{
    switch (e_pinAct) {
    case E_BSP_PIN_SET:
        hal_pinSet(p_pin);
        break;
    case E_BSP_PIN_CLR:
        hal_pinClr(p_pin);
        break;
    case E_BSP_PIN_GET:
        return hal_pinGet(p_pin);
    default:
        return 1;
    }
    return 0;
}

uint8_t bsp_spiRegRead(void * p_spi, uint8_t c_addr)
{
    uint8_t c_data;
    if (!hal_spiSlaveSel(p_spi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - reagread.");
        return 0;
    }

    hal_spiWrite(&c_addr, 1);
    hal_spiRead(&c_data, 1);
    hal_spiSlaveSel(p_spi,false);
    return c_data;
}

void bsp_spiFrameRead(void * p_spi,uint8_t c_addr, uint8_t * pc_data, uint16_t * pi_len)
{
    if (!hal_spiSlaveSel(p_spi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - fread.");
        return;
    }

    hal_spiWrite(&c_addr, 1);
    hal_spiRead((uint8_t *)pi_len, 1);
    hal_spiRead(pc_data, *pi_len + 1);
    hal_spiSlaveSel(p_spi,false);
}

uint8_t bsp_spiBitRead(void * p_spi, uint8_t c_addr, uint8_t c_mask, uint8_t c_off )
{
    uint8_t c_data;
    if (!hal_spiSlaveSel(p_spi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - bitread.");
        return 0;
    }

    hal_spiWrite(&c_addr, 1);
    hal_spiRead(&c_data, 1);
    hal_spiSlaveSel(p_spi,false);

    c_data &= c_mask;
    c_data >>=  c_off;
    return c_data;
}

void bsp_spiRegWrite(void * p_spi, uint8_t c_addr, uint8_t  c_data)
{
    if (!hal_spiSlaveSel(p_spi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - regwrite.");
        return;
    }

    hal_spiWrite(&c_addr, 1);
    hal_spiWrite(&c_data, 1);
    hal_spiSlaveSel(p_spi,false);
}

void bsp_spiFrameWrite(void * p_spi, uint8_t c_addr, uint8_t * pc_data, uint8_t c_len)
{
    if (!hal_spiSlaveSel(p_spi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - fwrite.");
        return;
    }
    hal_spiWrite(&c_addr, 1);
    hal_spiWrite(&c_len, 1);
    hal_spiWrite(pc_data, (uint16_t)c_len);
    hal_spiSlaveSel(p_spi,false);
}

/*============================================================================*/
/*  bsp_wdt()                                                           */
/*============================================================================*/
void bsp_wdt(en_bspWdtAction_t en_wdtAct)
{
    switch (en_wdtAct) {
        case E_BSP_WDT_RESET:
            if (!c_wtgStop)
                hal_watchdogReset();
            break;
        case E_BSP_WDT_START:
            c_wtgStop--;
            if (!c_wtgStop)
                hal_watchdogStart();
            break;
        case E_BSP_WDT_STOP:
            c_wtgStop++;
            hal_watchdogStop();
            break;
        case E_BSP_WDT_PERIODIC:
            if (!c_wtgStop)
                hal_watchdogReset();
            break;
        default:
            break;
    }

}

uint32_t         bsp_get(en_bspParams_t en_param)
{
    uint32_t l_ret;
    switch (en_param)
    {
        case E_BSP_GET_TICK:
            l_ret = hal_getTick();
            break;
        case E_BSP_GET_SEC:
            l_ret = hal_getSec();
            break;
        case E_BSP_GET_TRES:
            l_ret = hal_getTRes();
            break;
        default:
            l_ret = 0;
            break;
    }
    return l_ret;
}

/** @} */
/** @} */
