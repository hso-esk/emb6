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
 *      \addtogroup net
 *      @{
 *      \addtogroup thread
 *      @{
 *      \addtogroup demo_thread_node
 *      @{
*/
/*! \file   demp_thread_node.h

    \author Lukas Zimmermann, lukas.zimmermann@hs-offenburg.de

    \brief  This is the header file of the demo Thread node application

    \version 0.0.1
*/
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "demo_thread_node.h"

#include "er-coap.h"
#include "rest-engine.h"
#include "emb6.h"
#include "etimer.h"
#include "uip.h"
#include "bsp.h"

#include "mle_management.h"
#include "thrd-dev.h"
#include "thrd-partition.h"
#include "thrd-iface.h"
#include "thrd-network-data.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/

/*----------------------------------------------------------------------------*/
/*    demo_threadNodeInit()                                                   */
/*----------------------------------------------------------------------------*/
int8_t demo_threadInit(void)
{
  PRINTF("|===============================================================================|\n\r");
  PRINTF("|                                                                               |\n\r");
	// Initialize Thread device.
  thrd_dev_init();
  // Initialize Thread interface.
  thrd_iface_init();
  // Initialize Thread Network Data.
  // thrd_network_data_init();

  PRINTF("|THREAD DEVICE INITIALIZED                                                      |\n\r");
  PRINTF("|===============================================================================|\n\r");

  if ( !mle_init() ){ return -1; }

  return 0;
}

/*----------------------------------------------------------------------------*/
/*    demo_threadNodeConf()                                                   */
/*----------------------------------------------------------------------------*/

int8_t demo_threadConf(s_ns_t* p_netstk)
{
  int8_t ret = -1;

  /*
   * By default stack
   */
  if (p_netstk != NULL) {
    if (!p_netstk->c_configured) {
      p_netstk->hc    = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_driver_null;
      p_netstk->c_configured = 1;
      ret = 0;
    } else {
      if ((p_netstk->hc    == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154)    	&&
          (p_netstk->dllsec == &dllsec_driver_null)) {
        ret = 0;
      } else {
        p_netstk = NULL;
        ret = -1;
      }
    }
  }

  return (ret);
}

/** @} */
/** @} */
/** @} */
/** @} */
