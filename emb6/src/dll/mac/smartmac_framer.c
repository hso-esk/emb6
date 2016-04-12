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
 * @file    smartmac_framer.c
 * @author  PN
 * @brief   Smart-MAC framer for APSS
 */

/*
********************************************************************************
*                                  INCLUDES
********************************************************************************
*/
#include "emb6.h"

#include "rt_tmr.h"
#include "smartmac_framer.h"
#include "phy_framer_802154.h"

/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define SMARTMAC_FRAME_LEN              (uint8_t)( 5u )
#define SMARTMAC_BUFLEN                 (PHY_HEADER_LEN + SMARTMAC_FRAME_LEN + 4)

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static uint8_t smartmac_strobeCnt;
static uint8_t smartmac_broadcastCnt;
static uint8_t smartmac_rxBroadcast;
static rt_tmr_tick_t smartmac_strobeAckDelay;
static uint8_t smartmac_strobe[SMARTMAC_BUFLEN];
static uint8_t smartmac_strobeAck[SMARTMAC_BUFLEN];


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void smartmac_init(e_nsErr_t *p_err);
static uint8_t* smartmac_create(uint8_t frame_type, uint16_t *p_len, uint32_t *p_delay, e_nsErr_t *p_err);
static uint8_t* smartmac_createAck(uint16_t *p_len, rt_tmr_tick_t *p_delay, e_nsErr_t *p_err);
static uint8_t* smartmac_createStrobe(uint16_t *p_len, rt_tmr_tick_t *p_delay, e_nsErr_t *p_err);
static void smartmac_parse(uint8_t *p_frame_type, uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);
static void smartmac_parsesAck(uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);
static void smartmac_parseStrobe(uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);

static void smartmac_calcStrobeAckDelay(uint8_t ts_count);

#if (MAC_ULE_CFG_LOOSE_SYNC_EN == TRUE)
static void smartmac_calcStrobeDelay(s_nsMacUlePwrOnTblEntry_t *p_dev, uint32_t *p_delay);
#endif

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
s_nsMacUleFramerDrv_t SmartMACFramer = {
  "SmartMAC Framer",
   smartmac_init,
   smartmac_create,
   smartmac_parse,
};



/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * @brief   Calculate a time elapse before starting waking-up process
 *
 * @param   p_dev   Pointer to a structure holding power-on information of a
 *                  device
 */
#if (MAC_ULE_CFG_LOOSE_SYNC_EN == TRUE)
static void smartmac_calcStrobeDelay(s_nsMacUlePwrOnTblEntry_t *p_dev, uint32_t *p_delay)
{
  uint8_t n, i;
  uint32_t t0, t1;
  uint32_t tx_period;
  uint32_t tx_advance;
  uint32_t min_delay;

  t0 = p_dev->LastWakeup; /* Last power-on time interval */

  /*
   * Perform prediction on waking-up interval of the intended receiver
   */
  i = 1;
  do {
    n = (TmrCurTick - t0) / MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS; /* Number of power-on interval in between   */
    t1 = t0 + (n + i) * MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS;     /* Predicted next power-on time interval    */
    *p_delay = (t1 - TmrCurTick);

    /*
     * Note(s):
     *
     * (1)  Minimum possible delay shall be calculated as following:
     *      min_delay = T_off_to_on + T_on_to_off + T_scan + T_tx_smartpreamble
     */
    tx_period = MAC_ULE_PORT_STROBE_TX_TIME_IN_MS +
                MAC_ULE_PORT_STROBE_TX_GAP_TIME_IN_MS;

    tx_advance = MAC_ULE_CFG_QTY_STROBE_SENT_IN_ADVANCE * tx_period;

    min_delay = MAC_ULE_PORT_OFF_TO_ON_TIME_IN_MS +
                MAC_ULE_PORT_ON_TO_OFF_TIME_IN_MS +
                MAC_ULE_PORT_SCAN_DURATION_IN_MS;

    if (*p_delay < min_delay) {
      *p_delay = 0;
    } else {
      *p_delay -= min_delay;
      if (*p_delay > tx_advance) {
        *p_delay -= tx_advance;
      }
    }
    i++;
  } while (*p_delay == 0);
}
#endif

/**
 * @brief   Calculate a time elapse before replying with an ACK
 *
 * @param   ts_count    Time stamp count value
 */
static void smartmac_calcStrobeAckDelay(uint8_t ts_count)
{
  uint32_t min_delay;

  if (ts_count) {
    /*
     * Note:
     *
     * If calculated waiting time before sending an ACK is smaller than
     * or equal to minimum offset in the receiver caused by off-to-on
     * and on-to-off transition time, there is no need to go back to
     * sleep. Instead the receive should replay with an ACK immediately
     * as in SmartMAC framer.
     */
    smartmac_strobeAckDelay = ts_count * MAC_ULE_PORT_STROBE_TX_INTERVAL_IN_MS;
    min_delay = MAC_ULE_PORT_ON_TO_OFF_TIME_IN_MS + MAC_ULE_PORT_OFF_TO_ON_TIME_IN_MS;
    if (smartmac_strobeAckDelay <= min_delay) {
      smartmac_strobeAckDelay = 0;
    } else {
      smartmac_strobeAckDelay -= min_delay;
    }
  } else {
    smartmac_strobeAckDelay = 0;
  }
}


/**
 * @brief   Parse a waking-up strobe payload
 *
 * @param   p_pkt   Pointer to buffer holding strobe payload
 * @param   len     Length of the strobe payload
 * @param   p_err   Pointer to variable that stores the return error code
 */
static void smartmac_parseStrobe (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_pkt == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  netstk_devid_t dev_id;
  uint8_t ts_count;

  dev_id = (netstk_devid_t) (p_pkt[3]) | (netstk_devid_t) (p_pkt[4] << 8);
  if (dev_id == NetstkSrcId) {
    /* unicast strobe */
    *p_err = NETSTK_ERR_NONE;
    ts_count = p_pkt[2];
    smartmac_calcStrobeAckDelay(ts_count);

    /* set sequence number of the corresponding ACK equal to that in the
     * received strobe. This value is then used to improve the prediction
     * mechanism */
    smartmac_strobeAck[PHY_HEADER_LEN + 2] = ts_count;
    return;
  }

  if (dev_id == 0xFFFF) {
    /* broadcast strobe */

    ts_count = p_pkt[2];
    smartmac_rxBroadcast = 1;
    smartmac_calcStrobeAckDelay(ts_count);
    *p_err = NETSTK_ERR_NONE;
  }
}


/**
 * @brief   Parse a strobe ACK
 *
 * @param   p_pkt   Pointer to buffer holding ACK payload
 * @param   len     Length of the ACK payload
 * @param   p_err   Pointer to variable that stores the return error code
 */
static void smartmac_parsesAck (uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err)
{
  netstk_devid_t src_id;
  uint8_t is_valid_ack;

  src_id = (netstk_devid_t )(p_pkt[3]     ) |
           (netstk_devid_t )(p_pkt[4] << 8);

  /* valid ACK shall match device ID as well as counter value */
  is_valid_ack = (src_id == NetstkDstId) &&
                 (p_pkt[2] == (smartmac_strobeCnt + 1));
  if (is_valid_ack) {
    /* reset counter as the ACK is received */
    smartmac_strobeCnt = MAC_ULE_CFG_STROBE_TX_MAX;
    *p_err = NETSTK_ERR_NONE;

#if MAC_ULE_CFG_LOOSE_SYNC_EN
    uint8_t ix;

    for (ix = 0; ix < MAC_ULE_CFG_PWRON_TBL_SIZE ; ix++) {
      if (mac_ule_pwrOnTable[ix].DestId == NetstkDstId) {
        mac_ule_pwrOnTable[ix].LastWakeup = TmrCurTick;
        mac_ule_pwrOnTable[ix].StrobeSentQty = 0; /* Reset sent strobe quantity */
        break;
      }
    }
#endif
  } else {
    *p_err = NETSTK_ERR_MAC_ULE_INVALID_ACK;
  }
}


/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * @brief   Initialize Smart-MAC framer module
 *
 * @param   p_err   Pointer to a variable that stores a return error code
 */
static void smartmac_init (e_nsErr_t *p_err)
{
  /* pre-write SmartMAC ACK frame into buffer */
  smartmac_strobeAck[PHY_HEADER_LEN ] = MAC_ULE_FRAME_TYPE_SACK;
  smartmac_strobeAck[PHY_HEADER_LEN + 1] = 0;
  smartmac_strobeAck[PHY_HEADER_LEN + 2] = 0;
  smartmac_strobeAck[PHY_HEADER_LEN + 3] = NetstkSrcId;
  smartmac_strobeAck[PHY_HEADER_LEN + 4] = NetstkSrcId >> 8;

  /* clear SmartMAC Strobe frame buffer */
  smartmac_strobe[PHY_HEADER_LEN ] = 0;
  smartmac_strobe[PHY_HEADER_LEN + 1] = 0;
  smartmac_strobe[PHY_HEADER_LEN + 2] = 0;
  smartmac_strobe[PHY_HEADER_LEN + 3] = 0;
  smartmac_strobe[PHY_HEADER_LEN + 3] = 0;

  smartmac_strobeAckDelay = 0;
  smartmac_strobeCnt = MAC_ULE_CFG_STROBE_TX_MAX;
  smartmac_rxBroadcast = 0;
  smartmac_broadcastCnt = MAC_ULE_CFG_BROADCAST_TX_MAX;
  *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Create a waking-up strobe
 * @param   p_len
 * @param   p_delay
 * @param   p_err
 * @return
 */
static uint8_t* smartmac_createStrobe (uint16_t *p_len, rt_tmr_tick_t *p_delay, e_nsErr_t *p_err)
{
  uint8_t *p_pkt;

  /*
   * write strobe into buffer
   */
  smartmac_strobe[PHY_HEADER_LEN    ] = MAC_ULE_FRAME_TYPE_STROBE;
  smartmac_strobe[PHY_HEADER_LEN + 1] = 0;
  smartmac_strobe[PHY_HEADER_LEN + 2] = smartmac_strobeCnt;
  smartmac_strobe[PHY_HEADER_LEN + 3] = NetstkDstId;
  smartmac_strobe[PHY_HEADER_LEN + 4] = NetstkDstId >> 8;

  /* set returned values */
  *p_delay = 0u;
  *p_err = NETSTK_ERR_NONE;
  *p_len = SMARTMAC_FRAME_LEN;
  p_pkt = &smartmac_strobe[PHY_HEADER_LEN ];

  /* broadcast and unicast strobe are treated nearly similarly. They only
   * differ in how value of strobe counter field is calculated */
  if (NetstkDstId == (netstk_devid_t) (0xFFFFu)) {
    smartmac_strobe[PHY_HEADER_LEN + 2] = smartmac_broadcastCnt;

    /* decrease counter value of broadcast strobe by one until it reaches
     * 0. A broadcast strobe with counter value of 0 is considered the last
     * strobe, followed by actual broadcast packet */
    if (smartmac_broadcastCnt) {
      smartmac_broadcastCnt--;
    } else {
      smartmac_broadcastCnt = MAC_ULE_CFG_BROADCAST_TX_MAX;
      *p_err = NETSTK_ERR_MAC_ULE_LAST_STROBE;
    }
  } else {
#if MAC_ULE_CFG_LOOSE_SYNC_EN
    if (smartmac_strobeCnt == MAC_ULE_CFG_STROBE_TX_MAX) {
      /*
       * Estimate strobe transmission delay only applied to the first
       * strobe frame
       */
      uint8_t ix;
      uint8_t is_found = 0;
      uint8_t ix_free_slot = 0xFF;

      for (ix = 0; ix < MAC_ULE_CFG_PWRON_TBL_SIZE ; ix++) {
        if ((mac_ule_pwrOnTable[ix].DestId == NetstkDstId) &&
            (mac_ule_pwrOnTable[ix].LastWakeup != 0)) {
          is_found = 1;
          smartmac_calcStrobeDelay(&mac_ule_pwrOnTable[ix], p_delay);
          break;
        }

        if ((mac_ule_pwrOnTable[ix].DestId == 0) && (ix_free_slot == 0xFF)) {
          ix_free_slot = ix;
        }
      }

      if (is_found == 0) {
        mac_ule_pwrOnTable[ix_free_slot].DestId = NetstkDstId;
      }

      if (mac_ule_pwrOnTable[ix].StrobeSentQty > MAC_ULE_CFG_STROBE_TX_MAX) {
        mac_ule_pwrOnTable[ix].StrobeSentQty = 0;
      } else {
        mac_ule_pwrOnTable[ix].StrobeSentQty++;
      }
    }
#endif

    if (smartmac_strobeCnt) {
      smartmac_strobeCnt--;
    } else {
      smartmac_strobeCnt = MAC_ULE_CFG_STROBE_TX_MAX;
      *p_err = NETSTK_ERR_MAC_ULE_LAST_STROBE;
    }
  }

  return p_pkt;
}


/**
 * @brief   Create an ACK to a waking-up strobe. This frame is always the same
 *          regardless of originator of the waking-up strobe
 *
 * @param   p_len
 * @param   p_delay
 * @param   p_err
 * @return
 */
static uint8_t *smartmac_createAck (uint16_t *p_len, rt_tmr_tick_t *p_delay, e_nsErr_t *p_err)
{
  uint8_t *p_pkt;

  if (smartmac_rxBroadcast == 1) {
    smartmac_rxBroadcast = 0;
    p_pkt = NULL;
    *p_err = NETSTK_ERR_MAC_ULE_BROADCAST_NOACK;
  } else {
    p_pkt = &smartmac_strobeAck[PHY_HEADER_LEN ];
    *p_len = SMARTMAC_FRAME_LEN;
    *p_err = NETSTK_ERR_NONE;
  }

  *p_delay = smartmac_strobeAckDelay;
  smartmac_strobeAckDelay = 0; /* Reset SACK TX Delay */

  return p_pkt;
}


/**
 * @brief   Create a frame according to a given frame type
 *
 * @param   frame_type  Type of frame to create. This parameter is set to one of
 *                      following value:
 *                      @ref LPR_FRAME_TYPE_STROBE if a waking-up strobe is desired
 *                      @ref LPR_FRAME_TYPE_SACK if an ACK to waking-up strobe is desired
 *                      @ref LPR_FRAME_TYPE_BROADCAST if a broadcasting waking-up strobe is desired
 *
 * @param   p_len       Point to variable specifying length of frame to create
 * @param   p_delay     Point to variable specifying transmission delay
 * @param   p_err       Point to returned error code which is one of following value:
 *
 * @return  Pointer to buffer holding the created frame. If a NULL value is returned
 *          and type of frame is set to @ref LPR_FRAME_TYPE_BROADCAST, then
 *          the waking-up procedure is over and the module caller should start
 *          sending to broadcast frame(s)
 */
static uint8_t *smartmac_create (uint8_t frame_type, uint16_t *p_len, uint32_t *p_delay, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_len == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return NULL;
  }
#endif

  uint8_t *p_pkt;

  switch (frame_type) {
    case MAC_ULE_FRAME_TYPE_STROBE :
      p_pkt = smartmac_createStrobe(p_len, p_delay, p_err);
      break;

    case MAC_ULE_FRAME_TYPE_SACK :
      p_pkt = smartmac_createAck(p_len, p_delay, p_err);
      break;

    default:
      p_pkt = NULL;
      break;
  }
  return p_pkt;
}


/**
 * @brief   Parse a frame
 *
 * @param   p_pkt   Point to the frame to parse
 * @param   len     Length of the frame to parse
 * @param   p_err   Point to returned error code which is one following values:
 *                  @ref NETSTK_ERR_LPR_UNSUPPORTED_FRAME if the frame is not of APSS type
 *                  @ref NETSTK_ERR_NULL_POINTER if one of parameters is null
 *                  @ref NETSTK_ERR_NONE if the frame is well-formated.
 */
static void smartmac_parse (uint8_t *p_frame_type, uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_pkt == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  *p_frame_type = *p_pkt;
  switch (*p_frame_type) {
    case MAC_ULE_FRAME_TYPE_STROBE :
      smartmac_parseStrobe(p_pkt, len, p_err);
      break;

    case MAC_ULE_FRAME_TYPE_SACK :
      smartmac_parsesAck(p_pkt, len, p_err);
      break;

    default:
      *p_err = NETSTK_ERR_MAC_ULE_UNSUPPORTED_FRAME;
      break;
  }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
