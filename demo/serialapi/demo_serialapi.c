/*
 * --- License --------------------------------------------------------------*
 */
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

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       demo_niki.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Demo to show how to use the simple UDP socket interface.
 *
 *              This Demo shows how to use the simple UDP socket interface. Therefore
 *              the demo is divided into a server and a client application. The
 *              client transmits data periodically to the server including a
 *              fixed payload and a sequence counter. The server replies with
 *              the same sequence number.
 *              The demo makes use of the simplified Berkley Sockets alike interface
 *              to transmit and receive the data.
 *              The server is defined as the DAG-Root within the network and
 *              its IP address is retrieved automatically. The Server just
 *              replies to the node it got the packet from.
 *              This demo is mainly used to show how to use the UDP socket
 *              interface and to show basic connectivity.
 */

/*
 * --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "demo_serialapi.h"
#include "serialapi.h"

/*
 *  --- Macros ------------------------------------------------------------- *
 */
#define LOGGER_ENABLE               LOGGER_DEMO_SERIALAPI
#if LOGGER_ENABLE == TRUE
#define LOGGER_SUBSYSTEM            "SERIALAPI"
#endif /* #if LOGGER_ENABLE == TRUE */
#include "logger.h"

/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* demo_serialAPIInit()
*/
int8_t demo_serialApiInit( void )
{

  /* initialize serial API */
  serialApiInit();

  /* Always success */
  return 1;
} /* demo_serialAPIInit() */


/*---------------------------------------------------------------------------*/
/*
* demo_serialApiCfg()
*/
int8_t demo_serialApiConf( s_ns_t *p_netstk )
{
  int8_t i_ret = -1;

  if (p_netstk != NULL) {
    if (p_netstk->c_configured == FALSE) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_driver_null;
      i_ret = 1;
    } else {
      if ((p_netstk->hc == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154) &&
          (p_netstk->dllsec == &dllsec_driver_null)) {
        i_ret = 1;
      } else {
        p_netstk = NULL;
        i_ret = 0;
      }
    }
  }
  return i_ret;
} /* demo_serialApiCfg() */
