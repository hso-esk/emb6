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
 *   \addtogroup emb6
 *      @{
 *      \addtogroup demo
 *      @{
 *      \addtogroup demo_coap
 *      @{
 *      \addtogroup demo_coap_client
 *      @{
*/
/*! \file   demo_coap_cli.c

 \author Peter Lehmann, peter.lehmann@hs-offenburg.de

 \brief  CoAP Client example application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "er-coap.h"
#include "rest-engine.h"
#include "demo_coap_cli.h"
#include "emb6.h"
#include "etimer.h"
#include "uip.h"
#include "bsp.h"

#include "mle_management.h"

#include "thrd-route.h"
#include "thrd-leader-db.h"
#include "thrd-addr-query.h"
#include "thrd-router-id.h"
#include "thrd-partition.h"
#include "thrd-adv.h"
#include "thrd-send-adv.h"

#include "thrd-iface.h"
#include "thrd-network-data.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

/*==============================================================================
                                     MACROS
 =============================================================================*/

#define     LOGGER_ENABLE        LOGGER_DEMO_COAP
#include    "logger.h"

#define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xfe80, 0x0000, 0x0000, 0x0000, 0x0250, 0xc2ff, 0xfea8, 0xdddd)
/* #define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 0x1) */

#define LOCAL_PORT      UIP_HTONS(COAP_DEFAULT_PORT + 1)
#define REMOTE_PORT     UIP_HTONS(COAP_DEFAULT_PORT)    /* CoAP Default Port: 5683 */

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

uip_ipaddr_t rfd_eid;

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/


/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/


/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/



/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/*==============================================================================
 demo_coapInit()
==============================================================================*/

int8_t demo_thrdCoapInit(void)
{
	// Initialize Thread device.
	thrd_dev_init();
	PRINTF("Initializing Thread Interface.\n");
	thrd_iface_init();

	thrd_network_data_init();

	if ( !mle_init() ){ return 0; }
	//mle_set_parent_mode();
	//mle_set_child_mode();

	// Initialize routing database.
	// thrd_rdb_init();

	// Initialize leader database.
	// thrd_ldb_init();

	// thrd_iface.router_id = 2;

	// thrd_eid_rloc_db_init();

	// thrd_trickle_init();

	// thrd_partition_start();

	/* set periodic timer */
	//etimer_set(&timer, SEND_INTERVAL * bsp_get(E_BSP_GET_TRES), timer_callback);

	return 1;
}

/*==============================================================================
 demo_coapConf()
==============================================================================*/

uint8_t demo_thrdCoapConf(s_ns_t* p_netstk)
{
    uint8_t c_ret = 1;

    /*
     * By default stack
     */
    if (p_netstk != NULL) {
        if (!p_netstk->c_configured) {
            p_netstk->hc    = &sicslowpan_driver;
            p_netstk->frame = &framer_802154;
            p_netstk->dllsec = &nullsec_driver;
            p_netstk->c_configured = 1;

        } else {
            if ((p_netstk->hc    == &sicslowpan_driver) &&
                (p_netstk->frame == &framer_802154)    	&&
                (p_netstk->dllsec == &nullsec_driver)) {
            }
            else {
                p_netstk = NULL;
                c_ret = 0;
            }
        }
    }

    return (c_ret);
}

/** @} */
/** @} */
/** @} */
/** @} */
