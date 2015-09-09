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
 *      \addtogroup emb6
 *      @{
 *      \addtogroup demo
 *      @{
 *      \addtogroup demo_sniffer
 *      @{
*/
/*! \file   demo_sniffer.c

 \author Artem Yushev, 

 \brief  Sniffer application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/
#include "emb6.h"
#include "emb6_conf.h"
#include "bsp.h"
#include "demo_sniffer.h"

#define     LOGGER_ENABLE        LOGGER_DEMO_SNIFFER
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
s_ns_t* pst_ns = NULL;

/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*    demo_sniffInit()                                                           */
/*----------------------------------------------------------------------------*/

int8_t demo_sniffInit(void)
{

    LOG_INFO("Starting Sniffer");

    if ((pst_ns->inif != NULL) &&
        (pst_ns->inif->set_promisc != NULL)) {
        pst_ns->inif->set_promisc(1);
    }

    return 1;
} /* demo_sniffInit() */

/*----------------------------------------------------------------------------*/
/*    demo_sniffConf()                                                           */
/*----------------------------------------------------------------------------*/

uint8_t demo_sniffConf(s_ns_t* pst_netStack)
{
    uint8_t c_ret = 1;

    /*
     * By default stack
     */
    if (pst_netStack != NULL) {
        if (pst_netStack->hc == &sicslowpan_driver) {}
        else if (pst_netStack->llsec == &nullsec_driver) {}
        else if (pst_netStack->hmac == &nullmac_driver) {}
        else if (pst_netStack->lmac == &nullrdc_driver) {}
        else if (pst_netStack->frame == &no_framer) {}
        else {
            pst_netStack = NULL;
            c_ret = 0;
        }

        pst_netStack->hc     = &sicslowpan_driver;
        pst_netStack->llsec  = &nullsec_driver;
        pst_netStack->hmac   = &nullmac_driver;
        pst_netStack->lmac   = &nullrdc_driver;
        pst_netStack->frame  = &no_framer;
        /* Transceiver interface is defined by @ref board_conf function*/
        /*pst_netStack->inif   = $<some_transceiver>;*/

        pst_ns = pst_netStack;
    }
    return (c_ret);

} /*  demo_sniffConf() */

/** @} */
/** @} */
/** @} */
