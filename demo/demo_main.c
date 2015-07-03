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

    \author Artem Yushev, artem.yushev@hs-offenburg.de

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

#if DEMO_USE_UDP
#include "demo_udp.h"
#endif

#if DEMO_USE_COAP
#if CONF_USE_SERVER
#include "demo_coap_srv.h"
#else
#include "demo_coap_cli.h"
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
#include "demo_mqtt.h"
#endif


#if DEMO_USE_EXTIF
#include "slip_radio.h"
#include "slip.h"
#endif

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
#endif

/*==============================================================================
                                     MACROS
 =============================================================================*/

/*==============================================================================
                                     ENUMS
 =============================================================================*/

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
struct     etimer             led_et;
/*==============================================================================
                                LOCAL CONSTANTS
 =============================================================================*/

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
 =============================================================================*/
static void loc_initialConfig(void);
static uint8_t loc_demoAppsConf(s_ns_t* pst_netStack);
static uint8_t loc_demoAppsInit(void);
/*==============================================================================
                                LOCAL FUNCTIONS
 =============================================================================*/
static void loc_initialConfig(void)
{
    /* set last byte of mac address */
    mac_phy_config.mac_address[7] = (uint8_t)MAC_ADR_WORD;            // low byte
    mac_phy_config.mac_address[6] = (uint8_t)(MAC_ADR_WORD >> 8);     // high byte

    /* initial TX Power Output in dBm */
    mac_phy_config.init_power = TX_POWER;

    /* initial RX Sensitivity in dBm */
    mac_phy_config.init_sensitivity = RX_SENSITIVITY;

    /* initial wireless mode  */
    mac_phy_config.modulation = MODULATION;
}

static uint8_t loc_demoAppsConf(s_ns_t* pst_netStack)
{
    #if DEMO_USE_EXTIF
    demo_extifConf(pst_netStack);
    #endif

    #if DEMO_USE_COAP
    demo_coapConf(pst_netStack);
    #endif

    #if DEMO_USE_SNIFFER
    demo_sniffConf(pst_netStack);
    #endif

    #if DEMO_USE_UDPALIVE
    demo_udpAliveConf(pst_netStack);
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

    if (pst_netStack == NULL)
        return 0;
    else
        return 1;
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
    if (!demo_mqttInit()) {
        return 0;
    }
    #endif

    return 1;
}

void loc_emb6_running_led_toggle(c_event_t c_event, p_data_t p_data)
{
    if (etimer_expired(&led_et))
    {
        bsp_led(E_BSP_LED_YELLOW,E_BSP_LED_TOGGLE);
        etimer_restart(&led_et);
    }
}

/*==============================================================================
 main()
==============================================================================*/
int main(void)
{
    s_ns_t st_netstack;

    /*
     * By default stack is disabled
     */
    st_netstack.c_configured = 0;
    /*
    * set initial stack parameters
    */
    loc_initialConfig();

    /*
    * set proper stack pointers
    * If ok, pointer will points to network stack structure
    * @ref s_ns_t
    */
    if (!loc_demoAppsConf(&st_netstack)) {
        return 0;
    }

    /*
    * Initialize emb6-stack - call all initialization functions
    */
    if (emb6_init(&st_netstack))
    {
        /* initialize demo apps */
        if(!loc_demoAppsInit())
        {
            return 0;
        }
        /* emb6 running LED toggle */
        etimer_set(&led_et, 1 * bsp_get(E_BSP_GET_TRES), loc_emb6_running_led_toggle);

        /* call process function with delay in us */
        emb6_process(500);  /* default: 500 us delay */
    }
    printf("Program failed.");
    return 0;
}
/** @} */
/** @} */
