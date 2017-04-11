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

/*!  \file   demo_coap_cli.c

     \brief  CoAP Client example application
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "emb6.h"
#include "bsp.h"
#include "demo_lwm2m_cli.h"
#include "lwm2m-device.h"
#include "lwm2m-engine.h"


/*==============================================================================
                                     MACROS
 =============================================================================*/

#define     LOGGER_ENABLE        				LOGGER_DEMO_LWM2M
#include    "logger.h"

#ifdef LWM2M_USE_BOOTSTRAP
#define LWM2M_BOOTSTRAP_SERVER_IP(ipaddr)   uip_ip6addr(ipaddr,  0xbbbb, 0x0000, 0x0000, 0x0000, 0x000a, 0x0bff,0xfe0c, 0x0d0e)
#define LWM2M_BOOTSTRAP_SERVER_PORT         UIP_HTONS(5583)
#else
#define LWM2M_SERVER_IP(ipaddr)             uip_ip6addr(ipaddr, 0xbbbb, 0x0000, 0x0000, 0x0000,0x6eec, 0xebff,0xfe67, 0xea04)
#define LWM2M_SERVER_PORT                   UIP_HTONS(5683)
#endif /* #ifdef LWM2M_USE_BOOTSTRAP /* #ifdef LWM2M_USE_BOOTSTRAP */

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

uip_ipaddr_t server_ipaddr;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/*==============================================================================
 demo_lwm2mInit()
==============================================================================*/

int8_t demo_lwm2mInit(void)
{
    LOG_INFO("%s\n\r","Starting LWM2M Example Client");

#ifdef LWM2M_USE_BOOTSTRAP
    LWM2M_BOOTSTRAP_SERVER_IP(&server_ipaddr);
    lwm2m_engine_use_bootstrap_server(1);
    lwm2m_engine_register_with_bootstrap_server( &server_ipaddr, LWM2M_BOOTSTRAP_SERVER_PORT );
#else
    LWM2M_SERVER_IP(&server_ipaddr);
    lwm2m_engine_use_registration_server(1);
    lwm2m_engine_register_with_server( &server_ipaddr, LWM2M_SERVER_PORT );
#endif /* #ifdef LWM2M_USE_BOOTSTRAP */

    /* Initialize the OMA LWM2M engine */
    lwm2m_engine_init( NULL, NULL, NULL );
    return 0;
}

/*==============================================================================
 demo_lwm2mConf()
==============================================================================*/

int8_t demo_lwm2mConf(s_ns_t* p_netstk)
{
  int8_t ret = -1;

  /*
   * By default stack
   */
  if (p_netstk != NULL) {
    if (!p_netstk->c_configured) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_driver_null;
      p_netstk->c_configured = 1;
      ret = 0;
    } else {
      if ((p_netstk->hc == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154) &&
          (p_netstk->dllsec == &dllsec_driver_null)) {
        ret = 0;
      } else {
        p_netstk = NULL;
        ret = -1;
      }
    }
  }

  return ret;
}

/** @} */
/** @} */
/** @} */
/** @} */
