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
 * @file    phy_null.c
 * @date    16.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"


#define     LOGGER_ENABLE        LOGGER_PHY
#include    "logger.h"

/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void phy_init(void *p_netstk, e_nsErr_t *p_err);
static void phy_on(e_nsErr_t *p_err);
static void phy_off(e_nsErr_t *p_err);
static void phy_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void phy_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void phy_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */
static s_ns_t *pphy_netstk;

/*
 ********************************************************************************
 *                               GLOBAL VARIABLES
 ********************************************************************************
 */
const s_nsPHY_t phy_driver_null = {
  "PHY NULL",
   phy_init,
   phy_on,
   phy_off,
   phy_send,
   phy_recv,
   phy_ioctl
};

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

/**
 * @brief   Initialize driver
 *
 * @param   p_netstk    Pointer to netstack structure
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if (p_netstk == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  pphy_netstk = (s_ns_t *) p_netstk;
  *p_err = NETSTK_ERR_NONE;
}

/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pphy_netstk->rf->on(p_err);
}

/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pphy_netstk->rf->off(p_err);
}

/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((len == 0) || (p_data == NULL)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

#if LOGGER_ENABLE
  /*
   * Logging
   */
  uint16_t data_len = len;
  uint8_t *p_dataptr = p_data;
  LOG_RAW("PHY_TX: ");
  while (data_len--) {
    LOG_RAW("%02x", *p_dataptr++);
  }
  LOG_RAW("\r\n====================\r\n");
#endif

  /*
   * Issue next lower layer to transmit the prepared frame
   */
  pphy_netstk->rf->send(p_data, len, p_err);
}

/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((len == 0) || (p_data == NULL)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

#if LOGGER_ENABLE
  /*
   * Logging
   */
  uint16_t data_len = len;
  uint8_t *p_dataptr = p_data;
  LOG_RAW("\r\n====================\r\n");
  LOG_RAW("PHY_RX: ");
  while (data_len--) {
    LOG_RAW("%02x", *p_dataptr++);
  }
  LOG_RAW("\n\r");
#endif

  /*
   * Inform the next higher layer
   */
  pphy_netstk->mac->recv(p_data, len, p_err);
}

/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
  /* Set returned error code to default */
  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    case NETSTK_CMD_PHY_RSVD:
      break;

    default:
      pphy_netstk->rf->ioctrl(cmd, p_val, p_err);
      break;
  }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
