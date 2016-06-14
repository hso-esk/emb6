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
 *
 *      \defgroup demo Demo applications
 *
 *   This is a set of applications to demonstrate behavior of different supported
 *   protocols. Files which start with demo_ are used for demonstrating
 *   functionality of a given protocol.
 *
 *      @{
 *
 */
/*! \file   demo_main.c

    \author Artem Yushev,

    \brief  Main function.

   \version 0.0.1
*/

/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/
#include "emb6.h"
#include "board_conf.h"
#include "bsp.h"
#include "etimer.h"

#define  LOGGER_ENABLE        LOGGER_MAIN
#include "logger.h"

#if DEMO_USE_UDP
#include "demo_udp.h"
#endif

#if DEMO_USE_UDP_SOCKET
#include "demo_udp_socket.h"
#endif

#if DEMO_USE_THREAD
#include "thread_demo.h"
#endif

#if DEMO_USE_COAP
#if CONF_USE_SERVER
#include "demo_coap_srv.h"
#else
#include "demo_coap_cli.h"
#endif
#endif

#if DEMO_USE_MDNS
#if CONF_USE_SERVER
#include "demo_mdns_srv.h"
#else
#include "demo_mdns_cli.h"
#endif
#endif

#if DEMO_USE_SNIFFER
#include "demo_sniffer.h"
#endif

#if DEMO_USE_UDPALIVE
#include "demo_udp_alive.h"
#endif

#if DEMO_USE_APTB
#include "demo_aptb.h"
#endif

#if DEMO_USE_MQTT
#include "mqtt.h"
#endif

#if DEMO_USE_TESTSUITE
#include "demo_tsemb6.h"
#endif

#if DEMO_USE_EXTIF
#include "slip_radio.h"
#include "slip.h"
#endif

#if DEMO_USE_DTLS
#if CONF_USE_SERVER
#include "demo_dtls_srv.h"
#else
#include "demo_dtls_cli.h"
#endif
#endif

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
#endif

#if DEMO_USE_MCAST_ROOT
#include "demo_mcast_root.h"
#endif

#if DEMO_USE_MCAST_SINK
#include "demo_mcast_sink.h"
#endif

#if DEMO_USE_ROUTE_ROOT
#include "demo_route_root.h"
#endif

#if DEMO_USE_ROUTE_SINK
#include "demo_route_sink.h"
#endif

#if DEMO_USE_THRD_COAP
#if CONF_USE_SERVER
#include "demo_coap_srv.h"
#else
#include "demo_coap_cli.h"
#endif
#endif

/*==============================================================================
                                     MACROS
 =============================================================================*/

#ifndef EMB6_PROC_DELAY
#define EMB6_PROC_DELAY                     500
#endif /* #ifndef EMB6_PROC_DELAY */

/*==============================================================================
                                     ENUMS
 =============================================================================*/

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static void loc_stackConf(void);
static void loc_demoAppsConf(s_ns_t* pst_netStack, e_nsErr_t *p_err);
static uint8_t loc_demoAppsInit(void);

/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/
static void loc_stackConf(void)
{
    /* set last byte of mac address */
    mac_phy_config.mac_address[7] = (uint8_t)(MAC_ADDR_WORD);           // low byte
    mac_phy_config.mac_address[6] = (uint8_t)(MAC_ADDR_WORD >> 8);      // high byte

    /* initial TX Power Output in dBm */
    mac_phy_config.init_power = TX_POWER;

    /* initial RX Sensitivity in dBm */
    mac_phy_config.init_sensitivity = RX_SENSITIVITY;

    /* initial wireless mode  */
    mac_phy_config.modulation = MODULATION;
}


static void loc_demoAppsConf(s_ns_t* pst_netStack, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        emb6_errorHandler(p_err);
    }

    if (pst_netStack == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif

    #if DEMO_USE_EXTIF
    demo_extifConf(pst_netStack);
    #endif

    #if DEMO_USE_COAP
    demo_coapConf(pst_netStack);
    #endif

    #if DEMO_USE_MDNS
    demo_mdnsConf(pst_netStack);
    #endif

    #if DEMO_USE_SNIFFER
    demo_sniffConf(pst_netStack);
    #endif

    #if DEMO_USE_UDPALIVE
    demo_udpAliveConf(pst_netStack);
    #endif

    #if DEMO_USE_UDP_SOCKET
    demo_udpSocketCfg(pst_netStack);
    #endif

	#if DEMO_USE_THREAD
	demo_threadCfg(pst_netStack);
	#endif

    #if DEMO_USE_APTB
    demo_aptbConf(pst_netStack);
    #endif

    #if DEMO_USE_UDP
    demo_udpSockConf(pst_netStack);
    #endif

    #if DEMO_USE_MQTT
    demo_mqttConf(pst_netStack);
    #endif

    #if DEMO_USE_TESTSUITE
    demo_testsuiteConf(pst_netStack);
    #endif

    #if DEMO_USE_DTLS
    demo_dtlsConf(pst_netStack);
    #endif

	#if DEMO_USE_MCAST_ROOT
    demo_mcastRootConf(pst_netStack);
	#endif

	#if DEMO_USE_MCAST_SINK
    demo_mcastSinkConf(pst_netStack);
	#endif

	#if DEMO_USE_ROUTE_ROOT
	demo_routeRootConf(pst_netStack);
	#endif

	#if DEMO_USE_ROUTE_SINK
	demo_routeSinkConf(pst_netStack);
	#endif

	#if DEMO_USE_THRD_COAP
	demo_thrdCoapConf(pst_netStack);
	#endif

    /* set returned error code */
    *p_err = NETSTK_ERR_NONE;
}

static uint8_t loc_demoAppsInit(void)
{

    #if DEMO_USE_EXTIF
    if (!demo_extifInit()) {
        return 0;
    }
    #endif

    #if DEMO_USE_COAP
    if (!demo_coapInit()) {
        return 0;
    }
    #endif

    #if DEMO_USE_MDNS
    if (!demo_mdnsInit()) {
    	return 0;
    }
    #endif

    #if DEMO_USE_SNIFFER
    if (!demo_sniffInit()) {
        return 0;
    }
    #endif

    #if DEMO_USE_UDPALIVE
    if (!demo_udpAliveInit()) {
        return 0;
    }
    #endif

    #if DEMO_USE_UDP_SOCKET
    if (!demo_udpSocketInit()) {
        return 0;
    }
    #endif

	#if DEMO_USE_THREAD
	if (!demo_threadInit()) {
		return 0;
	}
	#endif

    #if DEMO_USE_APTB
    if (!demo_aptbInit()) {
        return 0;
    }
    #endif

    #if DEMO_USE_UDP
    if (!demo_udpSockInit()) {
        return 0;
    }
    #endif

    #if DEMO_USE_MQTT
    if (!mqtt_init()) {
        return 0;
    }
    #endif

    #if DEMO_USE_TESTSUITE
    if (!demo_testsuiteInit()) {
        return 0;
    }
    #endif

    #if DEMO_USE_DTLS
    if (!demo_dtlsInit()) {
	    return 0;
    }
    #endif

	#if DEMO_USE_MCAST_ROOT
    if (!demo_mcastRootInit()) {
    	return 0;
    }
	#endif

	#if DEMO_USE_MCAST_SINK
    if (!demo_mcastSinkInit()) {
    	return 0;
    }
	#endif

	#if DEMO_USE_ROUTE_ROOT
	if (!demo_routeRootInit()) {
		return 0;
	}
	#endif

	#if DEMO_USE_ROUTE_SINK
	if (!demo_routeSinkInit()) {
		return 0;
	}
	#endif

	#if DEMO_USE_THRD_COAP
	if (!demo_thrdCoapInit()) {
		return 0;
	}
	#endif

    return 1;
}

/*==============================================================================
 emb6_errorHandler()
==============================================================================*/
void emb6_errorHandler(e_nsErr_t *p_err)
{
    /* turns LEDs on to indicate error */
    bsp_led(E_BSP_LED_0, E_BSP_LED_ON);
    LOG_ERR("Program failed");

    /* TODO missing error handling */
    while (1) {
    }
}

/*==============================================================================
 main()
==============================================================================*/
int main(void)
{
    s_ns_t st_netstack;
    uint8_t ret;
    e_nsErr_t err;


    /* Initialize variables */
    err = NETSTK_ERR_NONE;
    memset(&st_netstack, 0, sizeof(st_netstack));

    /* Initialize BSP */
    ret = bsp_init(&st_netstack);
    if (ret != 1) {
        err = NETSTK_ERR_INIT;
        emb6_errorHandler(&err);
    }

    /* Configure applications */
    loc_demoAppsConf(&st_netstack, &err);
    if (err != NETSTK_ERR_NONE) {
        emb6_errorHandler(&err);
    }

    /* Initialize stack */
    loc_stackConf();
    emb6_init(&st_netstack, &err);
    if (err != NETSTK_ERR_NONE) {
        emb6_errorHandler(&err);
    }

    /* Show that stack has been launched */
    bsp_led(E_BSP_LED_2, E_BSP_LED_ON);
    bsp_delay_us(2000000);
    bsp_led(E_BSP_LED_2, E_BSP_LED_OFF);

    /* Initialize applications */
    ret = loc_demoAppsInit();
    if(ret == 0) {
        LOG_ERR("Demo APP failed to initialize");
        err = NETSTK_ERR_INIT;
        emb6_errorHandler(&err);
    }


    /* Start the emb6 stack */
    emb6_process(EMB6_PROC_DELAY);

    /* the program should never come here */
    return -1;
}
/** @} */
/** @} */
