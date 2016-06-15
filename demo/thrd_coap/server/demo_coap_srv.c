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
 *      \addtogroup demo_coap
 *      @{
 *      \addtogroup demo_coap_server
 *      @{
*/
/*! \file   demo_coap_srv.c

 \author Peter Lehmann

 \brief  CoAP Server example application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "er-coap.h"
#include "rest-engine.h"
#include "demo_coap_srv.h"
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

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

#define     SEND_INTERVAL               5

static struct etimer timer;

uint8_t cnt = 0;
uip_ipaddr_t test_eid;

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

static void timer_callback(c_event_t c_event, p_data_t p_data);
static void execute_routine(void);

/*==============================================================================
                                       LOCAL FUNCTIONS
 =============================================================================*/

static void
timer_callback(c_event_t c_event, p_data_t p_data)
{
	if (etimer_expired(&timer))
	{
		execute_routine();
		etimer_restart(&timer);
	}
} /* _mcast_callback */

/*----------------------------------------------------------------------------*/
/*    demo_coapInit()                                                           */
/*----------------------------------------------------------------------------*/

static void
execute_routine(void)
{
	switch(cnt) {
	case 0:

		uip_ip6addr(&test_eid, 0xfe80, 0x0000, 0x0000, 0x0000, 0x0250, 0xc2ff, 0xfea8, 0xdddd);

		uint16_t rloc16 = 0x1010;
		uint8_t ml_eid[8] = { 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F };
		// clock_time_t time = 0x80000008;

		// thrd_addr_ntf_response(&test_eid, &rloc16, ml_eid, NULL);
		thrd_addr_qry_request(&test_eid);

		thrd_partition_start();

		tlv_leader_t *ld_tlv = thrd_generate_leader_data_tlv();
		print_leader_data_tlv(ld_tlv);

		PRINTF("----------------------------------------------------\n");
		break;
	case 1:
		thrd_addr_qry_request(&test_eid);
		PRINTF("----------------------------------------------------\n");
		break;
	case 2:
		thrd_addr_qry_request(&test_eid);
		PRINTF("----------------------------------------------------\n");
		break;
	case 3:
		thrd_addr_qry_request(&test_eid);
		PRINTF("----------------------------------------------------\n");
		break;
	default:
		exit(0);
		break;
	}
	cnt++;
}

/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/

int8_t demo_thrdCoapInit(void)
{
	PRINTF("Initializing Thread Interface.\n");
	thrd_iface_init();

	if ( !mle_init() ){ return 0; }
	//mle_set_parent_mode();
	//mle_set_child_mode();

	// Initialize routing database.
	thrd_rdb_init();

	// Initialize leader database.
	// thrd_ldb_init();

	thrd_iface.router_id = 2;

	thrd_eid_rloc_db_init();

	thrd_trickle_init();

	// thrd_partition_start();

	/* set periodic timer */
	//etimer_set(&timer, SEND_INTERVAL * bsp_get(E_BSP_GET_TRES), timer_callback);

    return 1;
}

/*----------------------------------------------------------------------------*/
/*    demo_coapConf()                                                           */
/*----------------------------------------------------------------------------*/

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
