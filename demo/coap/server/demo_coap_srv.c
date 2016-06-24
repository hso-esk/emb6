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


/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "er-coap.h"
#include "rest-engine.h"
#include "demo_coap_srv.h"
#include "emb6.h"

#define     LOGGER_ENABLE        LOGGER_DEMO_COAP
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
/*
* Resources to be activated need to be imported through the extern keyword.
* The build system automatically compiles the resources in the corresponding sub-directory.
*/
extern resource_t
  res_tick,
  res_radio_info,
  res_radio_ctrl,
  res_temp,
  res_led,
  res_lcd;

/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*    demo_coapInit()                                                           */
/*----------------------------------------------------------------------------*/

int8_t demo_coapInit(void)
{

//    LOG_INFO("%s\n\r","Starting CoAP Example Server");

    /* Initialize the REST engine. */
    rest_init_engine();
/*
* Bind the resources to their Uri-Path.
* WARNING: Activating twice only means alternatle path, not two instances!
* All static variables are the same for each URI path.
*/

    rest_activate_resource(&res_tick, "tick");
    rest_activate_resource(&res_temp, "dev/temp");
    rest_activate_resource(&res_led, "dev/led");
    rest_activate_resource(&res_lcd, "dev/lcd");
    rest_activate_resource(&res_radio_info, "radio/info");
    rest_activate_resource(&res_radio_ctrl, "radio/control");

    return 1;
}

/*----------------------------------------------------------------------------*/
/*    demo_coapConf()                                                           */
/*----------------------------------------------------------------------------*/

uint8_t demo_coapConf(s_ns_t* p_netstk)
{
  uint8_t c_ret = 1;

  /*
   * By default stack
   */
  if (p_netstk != NULL) {
    if (!p_netstk->c_configured) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_driver_null;
      p_netstk->c_configured = 1;

    } else {
      if ((p_netstk->hc == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154) &&
          (p_netstk->dllsec == &dllsec_driver_null)) {
      } else {
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
