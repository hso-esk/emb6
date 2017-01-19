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
 *   \addtogroup stack_API Stack API
 *   @{
*/
/*! \file   emb6.c

    \author Peter Lehmann peter.lehmann@hs-offenburg.de

    \brief  emb6 stack initialization source file

    \version 0.0.2
*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
#include "emb6.h"

#include "tcpip.h"
#include "bsp.h"
#include "queuebuf.h"
#include "linkaddr.h"
#include "ctimer.h"
#include "rt_tmr.h"
#include "random.h"

#if NETSTACK_CONF_WITH_IPV6
#include "uip-ds6.h"
#endif

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
#endif

#define     LOGGER_ENABLE        LOGGER_CORE
#include    "logger.h"

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

uint8_t loc_emb6NetstackInit(s_ns_t * ps_netstack);

#ifdef EMB6_INIT_DROOT
static int8_t   loc_emb6DagRootInit(void);
#endif

/*==============================================================================
                           TYPEDEFS
 =============================================================================*/
/* Don't use this variable directly, instead take emb6_get() */
static s_ns_t*  ps_emb6Stack;

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
    .DIOintmin          = 10,
    .DIOintdoub         = 12,
    /* This value decides which DAG instance we should
     * participate in by default. */
    .defInst            = 0x1e,
    /* Initial metric attributed to a link when the ETX is unknown */
    .linkMetric         = 2,
    .defRouteTimeUnit   = 0xffff,
    .defRouteTime       = 0xff,

};

/** MAC address default Configuration */
s_mac_phy_conf_t mac_phy_config = {
    /* by default the configuration is not yet saved */
    .is_saved         = FALSE,
#if DEMO_USE_EXTIF
    /* set extif mac address */
    .mac_address      = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
#else
    /* set default mac address */
    .mac_address      = {0x00, 0x50, 0xc2, 0xff, 0xfe, 0xa8, 0xdd, 0xdd},
#endif
    /* set default pan id */
    .pan_id           = 0xABCD,
    .init_power       = 11,
    .init_sensitivity = -100,
    .modulation       = MODULATION_BPSK20,    /* this attribute can be appropriately overwritten by BSP */
    .fcs_len          = 2,                    /* 16-bit CRC */
    .op_mode          = NETSTK_RF_OP_MODE_1,  /* 2-FSK */
    .chan_num         = 26,                   /* 868.325 MHz */

#if (NETSTK_CFG_WOR_EN == TRUE)
    .preamble_len     = 24,
#else
    .preamble_len     = 4,
#endif

#if (NETSTK_CFG_LOW_POWER_MODE_EN == TRUE)
    /** sleep period in ticks in Low-Power mode */
    .sleepTimeout     = 200,
#endif
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
        dag = rpl_set_root(rpl_config.defInst,(uip_ip6addr_t *)&un_ipaddr);
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
  uint8_t c_err = 0;
  uint8_t is_valid;
  e_nsErr_t err;

  /* Initialize stack protocols */
  queuebuf_init();
  ctimer_init();
  rt_tmr_init();

  /* initialize trace */
#if (TRACE_CFG_EN == TRUE)
  trace_init();
  trace_start();
  trace_printf("Trace started\n");
#endif

  /*
   * Verify stack submodule drivers
   */
  is_valid = (ps_ns->rf     != NULL) &&
             (ps_ns->phy    != NULL) &&
             (ps_ns->mac    != NULL) &&
             (ps_ns->dllc   != NULL) &&
             (ps_ns->dllsec != NULL) &&
             (ps_ns->hc     != NULL);
  if (is_valid) {
    /*
     * Netstack submodule initializations
     */
    ps_ns->rf->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->phy->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->mac->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->dllc->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->dllsec->init(ps_ns); /* logical link security    */
    ps_ns->hc->init(ps_ns);     /* header compressor        */
    ps_ns->frame->init(ps_ns);  /* sicslowpan driver        */

    /*
     * Initialize TCP/IP stack
     */
    tcpip_init();
    c_err = 1;
  }


#if EMB6_INIT_ROOT==TRUE
    if (!c_err || !loc_emb6DagRootInit()) {
        c_err = 1;
    }
#endif /* DEMO_USE_DAG_ROOT */

    return (c_err);
}

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/
void emb6_init(s_ns_t* ps_ns, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        emb6_errorHandler(p_err);
    }

    if (ps_ns == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif

    uint8_t ret;

    /* set return error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* initialize netstack */
    ret = loc_emb6NetstackInit(ps_ns);
    if (ret == 0) {
        *p_err = NETSTK_ERR_INIT;
        LOG_ERR("Failed to initialize emb6 stack");
    }else {
        ps_emb6Stack = ps_ns;
    }

    e_nsErr_t   err;

    /* turn the netstack on */
    ps_emb6Stack->dllc->on(&err);
    if (err != NETSTK_ERR_NONE) {
        emb6_errorHandler(&err);
    }
}

s_ns_t * emb6_get(void)
{
    return ps_emb6Stack;
}


void emb6_process(int32_t us_delay)
{
    uint8_t runLoop = (us_delay < 0) ? FALSE : TRUE;
    uint32_t delay = runLoop ? us_delay : 0;

    /* Attention: emb6 main process loop !! do not change !! */
    do
    {
        evproc_nextEvent();
        etimer_request_poll();
        bsp_delayUs(delay);
    }while(runLoop);
}

/** @} */
/** @} */

