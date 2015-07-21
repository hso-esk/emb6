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
 *      \addtogroup embetter6
 *      @{
 *   \addtogroup stack_API Stack API
 *   @{
*/
/*! \file   emb6.c

    \author Peter Lehmann peter.lehmann@hs-offenburg.de

    \brief  emb6 stack initialization source file

    \version 0.0.1
*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "emb6_conf.h"
#include "tcpip.h"
#include "bsp.h"
#include "queuebuf.h"
#include "linkaddr.h"
#include "ctimer.h"
#include "random.h"

#if NETSTACK_CONF_WITH_IPV6
#include "uip-ds6.h"
#endif

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
#endif

#define     LOGGER_ENABLE        LOGGER_CORE
#if            LOGGER_ENABLE     ==     TRUE
#define        LOGGER_SUBSYSTEM    "core"
#endif
#include    "logger.h"

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

uint8_t loc_emb6NetstackInit(s_ns_t * ps_netstack);

#if EMB6_DEMO_TOT
void loc_emb6ToutHandler(c_event_t c_event, p_data_t p_data );
#endif /* #if EMB6_DEMO_TOT */

#ifdef EMB6_INIT_DROOT
static int8_t   loc_emb6DagRootInit(void);
#endif

/*==============================================================================
                           TYPEDEFS
 =============================================================================*/
/* Don't use this variable directly, instead take emb6_get() */
static s_ns_t*  ps_emb6Stack;

#if EMB6_DEMO_TOT
struct     etimer             et_timeout;
#endif /* #if EMB6_DEMO_TOT */

/*---------------------------------------------------------------------------*/
/** @{ \name Layer 2 variables */
/*---------------------------------------------------------------------------*/
/** Host L2 address */
#if UIP_CONF_LL_802154
uip_lladdr_t uip_lladdr;
#else /*UIP_CONF_LL_802154*/
uip_lladdr_t uip_lladdr = {{0x00,0x06,0x98,0x00,0x02,0x32}};
#endif /*UIP_CONF_LL_802154*/
/** @} */


/** RPL default Configuration */
s_rpl_conf_t rpl_config = {
            .DIO_interval_min = 8,
            .DIO_interval_doublings = 12,
            .default_instance = 0x1e,                    /* This value decides which DAG instance we should participate in by default. */
            .init_link_metric = 2,                        /* Initial metric attributed to a link when the ETX is unknown */
            .default_route_lifetime_unit = 0xffff,
            .default_route_lifetime = 0xff,

};

/** MAC address default Configuration */
s_mac_phy_conf_t mac_phy_config = {
#if DEMO_USE_EXTIF
        .mac_address =  { 0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff },     /* set extif mac address */
#else
        .mac_address =  { 0x00,0x50,0xc2,0xff,0xfe,0xa8,0xdd,0xdd },     /* set default mac address */
#endif
        .pan_id = 0xABCD,                                                /* set default pan id */
        .init_power = 11,
        .init_sensitivity = -100,
        .modulation = MODULATION_BPSK20,
};


#if EMB6_INIT_ROOT==TRUE
static int8_t loc_emb6DagRootInit(void)
{
    uip_ipaddr_t un_ipaddr;
    struct uip_ds6_addr *root_if;
    uint16_t pi_netPrefix[4] = {NETWORK_PREFIX_DODAG};

    /* Mask future ip address with prefix of a net */
    uip_ip6addr(&un_ipaddr, pi_netPrefix[0],pi_netPrefix[1],\
                            pi_netPrefix[2],pi_netPrefix[3], 0, 0, 0, 0);
    /* Add MAC address in the end of IP address */
    uip_ds6_set_addr_iid(&un_ipaddr,(uip_lladdr_t *)&uip_lladdr.addr);
    /* Add new IP address to the list of interfaces */
    uip_ds6_addr_add(&un_ipaddr, 0, ADDR_MANUAL);

    root_if = uip_ds6_get_global(-1);
    if(root_if != NULL) {
        rpl_dag_t *dag;
        dag = rpl_set_root(rpl_config.default_instance,(uip_ip6addr_t *)&un_ipaddr);
        rpl_set_prefix(dag, &un_ipaddr, 64);
        LOG_INFO("created a new RPL dag");
    } else {
        LOG_INFO("failed to create a new RPL DAG");
        return 0;
    }
    return 1;
}
#endif /* DEMO_USE_DAG_ROOT  */

/*==============================================================================
                                 LOCAL FUNCTIONS
 =============================================================================*/

uint8_t loc_emb6NetstackInit(s_ns_t * ps_ns)
{
    uint8_t     c_err = 0;
    /* Initialize stack protocols */
    queuebuf_init();
    ctimer_init();
    if ((ps_ns->hc != NULL) && (ps_ns->llsec != NULL) && (ps_ns->hmac != NULL) &&
        (ps_ns->lmac != NULL) && (ps_ns->frame != NULL) && (ps_ns->inif != NULL)) {
        ps_ns->inif->init(ps_ns);
        ps_ns->frame->init(ps_ns);
        ps_ns->lmac->init(ps_ns);
        ps_ns->hmac->init(ps_ns);
        ps_ns->llsec->init(ps_ns);
        ps_ns->hc->init(ps_ns);
        tcpip_init();
        c_err = 1;
    }

#if EMB6_INIT_ROOT==TRUE
    if (!loc_emb6DagRootInit()) {
        c_err = 1;
    }
#endif /* DEMO_USE_DAG_ROOT */

#if EMB6_DEMO_TOT
    /* deinit tcpip stack after 30 minutes */
    etimer_set(&et_timeout, EMB6_DEMO_TOT * bsp_get(E_BSP_GET_TRES), loc_emb6ToutHandler);
#endif /* #if EMB6_DEMO_TOT */
    return (c_err);
}

#if EMB6_DEMO_TOT
void loc_emb6ToutHandler(c_event_t c_event, p_data_t p_data )
{
    if (etimer_expired(&et_timeout))
    {
        while(1);
    }
}
#endif /* #if EMB6_DEMO_TOT */

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/



void rimeaddr_emb6_set_node_addr(linkaddr_t *t)
{
    linkaddr_set_node_addr(t);
}

uint8_t emb6_init(s_ns_t * ps_ns)
{
    uint8_t c_err = 1;

    if (!bsp_init(ps_ns)) {
        ps_ns = NULL;
        c_err = 0;
    }

    if (!loc_emb6NetstackInit(ps_ns) && c_err) {
        ps_ns = NULL;
        LOG_ERR("Failed to initialise emb6 stack");
        c_err = 0;
    }

    ps_emb6Stack = ps_ns;

    return (c_err);
}

s_ns_t * emb6_get(void)
{
    return ps_emb6Stack;
}


void emb6_process(uint16_t us_delay)
{
    /* Attention: emb6 main process loop !! do not change !! */
    while(1)
    {
        evproc_nextEvent();
        etimer_request_poll();
        bsp_delay_us(us_delay);
    }
}

/** @} */
/** @} */
