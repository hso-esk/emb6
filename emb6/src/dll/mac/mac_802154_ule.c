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
 * @file    mac_802154_ule.c
 * @author  PN
 * @brief   IEEE802.15.4 Ultra-Low Energy MAC
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"

#include "evproc.h"
#include "packetbuf.h"
#include "bsp.h"
#include "logger.h"

#include "rt_tmr.h"
#include "mac_ule.h"
#include "framer_802154.h"
#include "framer_smartmac.h"


/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define MAC_ULE_CFG_STAT_EN                 TRUE

/*
********************************************************************************
*                               LOCAL ENUMS
********************************************************************************
*/
typedef enum
{
    MAC_ULE_STATE_NON_INIT = 0U,
    MAC_ULE_STATE_INIT,
    MAC_ULE_STATE_SLEEP,
    MAC_ULE_STATE_IDLE,

    /* SCAN Submachine states */
    MAC_ULE_STATE_SCAN_STARTED,
    MAC_ULE_STATE_SCAN_BUSY,
    MAC_ULE_STATE_SCAN_FINISHED,

    /* RX Submachine states */
    MAC_ULE_STATE_RX_DELAYED,
    MAC_ULE_STATE_RX_WFP,
    MAC_ULE_STATE_RX_FINISHED,

    /* TX Submachine states */
    MAC_ULE_STATE_TX_STARTED,
    MAC_ULE_STATE_TX_DELAYED,
    MAC_ULE_STATE_TX_STROBE,
    MAC_ULE_STATE_TX_FINISHED
} e_mac_ule_state_t;


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void mac_ule_init (void *p_netstk, e_nsErr_t *p_err);
static void mac_ule_start(e_nsErr_t *p_err);
static void mac_ule_stop(e_nsErr_t *p_err);
static void mac_ule_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_ule_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_ule_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static void mac_ule_on(e_nsErr_t *p_err);
static void mac_ule_off(e_nsErr_t *p_err);
static void mac_ule_csma(e_nsErr_t *p_err);
static void mac_ule_txBroadcast(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_ule_txUnicast(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_ule_txPayload(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);

static void mac_ule_idleListening(c_event_t c_event, p_data_t p_data);
static void mac_ule_channelScan(e_nsErr_t *p_err);
static void mac_ule_rxBroadcast(e_nsErr_t *p_err);
static void mac_ule_rxUnicast(e_nsErr_t *p_err);
static void mac_ule_txAck(uint8_t seq, e_nsErr_t *p_err);
static void mac_ule_tmrIsrPowerUp(void *p_arg);
static void mac_ule_rxBufTimeout(s_rt_tmr_t *p_tmr, e_nsErr_t *p_err);

static void mac_ule_txStrobe(uint8_t counter, e_nsErr_t *p_err);
static void mac_ule_txStrobeAck(uint8_t counter, uint16_t dest_addr, e_nsErr_t *p_err);

#if (MAC_ULE_CFG_LOOSE_SYNC_EN == TRUE)
static void mac_ule_updatePowerOnTable(void);
static void mac_ule_calcStrobeDelay(e_nsErr_t *p_err);
static void mac_ule_calcDataDelay(uint8_t counter, e_nsErr_t *p_err);
#endif

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t *pmac_ule_netstk;
static e_mac_ule_state_t mac_ule_state;

static void *pmac_ule_txCbArg;
static nsTxCbFnct_t mac_ule_txCbFnct;

static uint8_t *pmac_ule_rxPktPtr;
static uint16_t mac_ule_rxPktLen;

/*
 * Timers
 */
static s_rt_tmr_t mac_ule_tmrPowerUp;
static s_rt_tmr_t mac_ule_tmr1Scan;
static s_rt_tmr_t mac_ule_tmr1WFSA;
static s_rt_tmr_t mac_ule_tmr1WFD;
static s_rt_tmr_t mac_ule_tmr1WFA;

#if (MAC_ULE_CFG_STAT_EN == TRUE)
#define LOGGER_ENABLE       TRUE
#include "logger.h"

static uint32_t mac_ule_nUnicastTx;
static uint32_t mac_ule_nUnicastTxOk;
static uint32_t mac_ule_nUnicastRx;
static uint32_t mac_ule_nUnicastRxOk;
#endif


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
netstk_devid_t      NetstkSrcId;
netstk_devid_t      NetstkDstId;


#if MAC_ULE_CFG_LOOSE_SYNC_EN
s_nsMacUlePwrOnTblEntry_t mac_ule_pwrOnTable[MAC_ULE_CFG_PWRON_TBL_SIZE];
#endif

const s_nsMAC_t mac_driver_ule =
{
   "MAC IEEE802.15.4 Ultra-Low Energy",
    mac_ule_init,
    mac_ule_start,
    mac_ule_stop,
    mac_ule_send,
    mac_ule_recv,
    mac_ule_ioctl,
};

extern uip_lladdr_t uip_lladdr;


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
/**
 * @brief   Initialization of MAC ULE
 * @param   p_netstk    pointer to variable holding netstack structure
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_init (void *p_netstk, e_nsErr_t *p_err)
{
  mac_ule_state = MAC_ULE_STATE_NON_INIT;

#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if (p_netstk == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  /* set returned error code to default */
  *p_err = NETSTK_ERR_NONE;

  /*
   * Initialize internal attributes
   */
  pmac_ule_netstk = (s_ns_t *) p_netstk;
  pmac_ule_txCbArg = NULL;
  mac_ule_txCbFnct = 0;
  pmac_ule_rxPktPtr = NULL;
  mac_ule_rxPktLen = 0;
  mac_ule_state = MAC_ULE_STATE_SLEEP;

  /*
   * Initialize framer
   */
  NetstkDstId = MAC_ULE_ID_INVALID;
  memcpy(&NetstkSrcId, &mac_phy_config.mac_address[6], 2);
  TRACE_LOG_MAIN("source address %04x", NetstkSrcId);

  /*
   * Configure stack address
   */
  memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
  linkaddr_set_node_addr((linkaddr_t *) mac_phy_config.mac_address);

  /* Register event */
  evproc_regCallback(NETSTK_MAC_ULE_EVENT, mac_ule_idleListening);

  /*
   * Configure timers
   */
  rt_tmr_create(&mac_ule_tmrPowerUp, E_RT_TMR_TYPE_PERIODIC, MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS, mac_ule_tmrIsrPowerUp, NULL);
  rt_tmr_create(&mac_ule_tmr1Scan, E_RT_TMR_TYPE_ONE_SHOT, MAC_ULE_PORT_SCAN_DURATION_IN_MS, 0, NULL);
  rt_tmr_create(&mac_ule_tmr1WFD, E_RT_TMR_TYPE_ONE_SHOT, MAC_ULE_PORT_WFD_TIMEOUT_IN_MS, 0, NULL);
  rt_tmr_create(&mac_ule_tmr1WFSA, E_RT_TMR_TYPE_ONE_SHOT, MAC_ULE_PORT_STROBE_TX_GAP_TIME_IN_MS, 0, NULL);
  rt_tmr_create(&mac_ule_tmr1WFA, E_RT_TMR_TYPE_ONE_SHOT, MAC_ULE_PORT_WFA_TIMEOUT_IN_MS, 0, NULL);


#if (MAC_ULE_CFG_STAT_EN == TRUE)
  /* initialize local statistical variables */
  mac_ule_nUnicastTx = 0;
  mac_ule_nUnicastTxOk = 0;
  mac_ule_nUnicastRx = 0;
  mac_ule_nUnicastRxOk = 0;
#endif
}

static void mac_ule_start(e_nsErr_t *p_err)
{
  rt_tmr_start(&mac_ule_tmrPowerUp);
}

static void mac_ule_stop(e_nsErr_t *p_err)
{
  rt_tmr_stop(&mac_ule_tmrPowerUp);
}


/**
 * @brief   Turn MAC ULE on
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_on(e_nsErr_t *p_err)
{
  pmac_ule_netstk->phy->on(p_err);
}


/**
 * @brief   Turn MAC ULE off
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_off(e_nsErr_t *p_err)
{
  pmac_ule_netstk->phy->off(p_err);
}


/**
 * @brief   Transmission request handling
 *
 * @note    (1) A transmission request is accepted only if one of following
 *          conditions is met
 *            (a) MAC_ULE_State == MAC_ULE_STATE_IDLE, or
 *            (b) MAC_ULE_State == MAC_ULE_STATE_SLEEP
 *
 *          (2) In case MAC ULE in state MAC_ULE_STATE_IDLE, if the destination
 *          node is not the node with which the device is exchanging data (i.e.
 *          some local variables could be used to temporarily store those info),
 *          then the MAC ULE shall start the low-power transmission procedure
 *          over.
 *
 *          (3) While MAC ULE stay in sleep mode, waiting for the 'right' time
 *          from when it starts sending wake-up frames, it shall neither enable
 *          the transceiver nor perform channel scan. This allows simplicity in
 *          implementation.
 *          The simplicity, nevertheless, comes at costs of losing advantage of
 *          the loosely-synchronization feature in congested network. The
 *          problem occurs when the device is waiting until the moment it thinks
 *          the node of interest wakes up. Simultaneously another node desires
 *          to communicate with the device, and therefore it misses the 'right'
 *          time as the device is not able to receive any frames during this
 *          time.
 *          This problem could be solved via an appropriate retransmission
 *          operation.
 *
 */
static void mac_ule_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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

  int is_broadcast;

  if ((mac_ule_state != MAC_ULE_STATE_SLEEP)) {
    *p_err = NETSTK_ERR_BUSY;
    return;
  }

  /* set returned error to default */
  TRACE_LOG_FULL("MAC_TX: start");
  *p_err = NETSTK_ERR_NONE;
  mac_ule_state = MAC_ULE_STATE_TX_STARTED;
  mac_ule_on(p_err);

  /*
   * This function shall be operating in blocking manner.
   * The reason is that the sicslowpan module expects to learn the result of
   * transmission process right after the function is executed.
   */
  is_broadcast = packetbuf_holds_broadcast();
  if (is_broadcast == 1) {
    mac_ule_txBroadcast(p_data, len, p_err);
  } else {
    mac_ule_txUnicast(p_data, len, p_err);
  }
  mac_ule_state = MAC_ULE_STATE_TX_FINISHED;

  /*
   * Finalize the transmission process
   */
  mac_ule_off(p_err);
  mac_ule_state = MAC_ULE_STATE_SLEEP;
  TRACE_LOG_FULL("MAC_TX: stop");

  /* signal the caller of result of the transmission attempt */
  if (mac_ule_txCbFnct) {
    mac_ule_txCbFnct(pmac_ule_txCbArg, p_err);
  }
}


/**
 * @brief   MAC ULE frame reception handling
 * @param   p_data  point to buffer holding the received frame
 * @param   len     length of the received frame
 * @param   p_err   point to variable holding the received error code
 */
static void mac_ule_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  /* store information regarding the received packet */
  mac_ule_rxPktLen = len;
  pmac_ule_rxPktPtr = p_data;
}


/**
 * @brief   MAC ULE Input/Output Control
 * @param   cmd     indicates control command
 * @param   p_val   point to variable to be used by the command
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    case NETSTK_CMD_TX_CBFNCT_SET:
      mac_ule_txCbFnct = (nsTxCbFnct_t) p_val;
      break;

    case NETSTK_CMD_TX_CBARG_SET:
      pmac_ule_txCbArg = p_val;
      break;

    default:
      pmac_ule_netstk->phy->ioctrl(cmd, p_val, p_err);
      break;
  }
}


/**
 * @brief   Handles idle listening operation
 */
static void mac_ule_idleListening(c_event_t c_event, p_data_t p_data)
{
  e_nsErr_t err;

  if (c_event == NETSTK_MAC_ULE_EVENT) {
    TRACE_LOG_MAIN("MAC_SCAN: start");

    /*
     * (S1) Preparation
     */
    LED_SCAN_ON();
    mac_ule_on(&err);
    mac_ule_state = MAC_ULE_STATE_SCAN_BUSY;

    /*
     * (S2) Perform channel scan
     */
    err = NETSTK_ERR_NONE;
    mac_ule_channelScan(&err);
    if (err == NETSTK_ERR_NONE) {
      /* signal the upper layer if reception was successful */
      packetbuf_clear();
      packetbuf_set_datalen(mac_ule_rxPktLen);
      memcpy(packetbuf_dataptr(), pmac_ule_rxPktPtr, mac_ule_rxPktLen);
      pmac_ule_netstk->dllc->recv(pmac_ule_rxPktPtr, mac_ule_rxPktLen, &err);
    }

    /*
     * (S3) Finalize channel scan process
     */
    TRACE_LOG_MAIN("MAC_SCAN: stop (-%d)", err);
    mac_ule_off(&err);
    mac_ule_state = MAC_ULE_STATE_SLEEP;
    LED_SCAN_OFF();
  }
}


/**
 * @brief   Channel scan handling
 * @param   p_err Point to variable holding returned error code
 */
static void mac_ule_channelScan(e_nsErr_t *p_err)
{
  frame_smartmac_st frame;

  /*
   * (S1)     Channel scan handling
   *
   *          A channel scan attempt is declared as finished only if one of
   *          following conditions is met:
   *          (a)     A valid strobe has arrived
   *          (b)     Scan timeout is over
   */
  mac_ule_rxBufTimeout(&mac_ule_tmr1Scan, p_err);
  /* was a frame arrived? */
  if (*p_err == NETSTK_ERR_NONE) {
    /* then parse the received frame */
    frame_smartmac_parse(pmac_ule_rxPktPtr, mac_ule_rxPktLen, &frame);
    /* was the frame not STROBE? */
    if (frame.type != SMARTMAC_FRAME_STROBE) {
      /* then terminate channel scan following reception of invalid frame */
      *p_err = NETSTK_ERR_MAC_ULE_NO_STROBE;
    }
    /* was the frame STROBE */
    else {
      /* then start reception process */
      TRACE_LOG_MAIN("MAC_RX: Strobe c=%02x d=%04x s=%04x", frame.counter, frame.dest_addr, frame.src_addr);
      /* was the pending frame Broadcast? */
      if (frame.dest_addr == 0xFFFF) {
        /* then calculate waiting time and start broadcast reception process */
        mac_ule_calcDataDelay(frame.counter, p_err);
        mac_ule_rxBroadcast(p_err);
      }
      /* was the pending frame unicast */
      else if (frame.dest_addr == NetstkSrcId) {
        /* then replies with an ACK */
        mac_ule_txStrobeAck(frame.counter, frame.src_addr, p_err);
        /* was the ACK successfully transmitted? */
        if (*p_err == NETSTK_ERR_NONE) {
          /* then start unicast reception process */
          mac_ule_rxUnicast(p_err);
        }
        /* has the ACK failed to transmit? */
        else {
          /* then terminate the scanning process */
        }
      }
      /* was the pending frame neither broadcast nor unicast? */
      else {
        /* then terminate channel scan */
        *p_err = NETSTK_ERR_MAC_ULE_NO_STROBE;
      }
    }
  }
  /* was no frame arrived during channel scan duration? */
  else {
    /* then terminate the channel scan */
    *p_err = NETSTK_ERR_MAC_ULE_NO_STROBE;
  }
}


/**
 * @brief   Broadcast reception handling
 * @param   p_err   Point to variable holding returned error code
 */
static void mac_ule_rxBroadcast(e_nsErr_t *p_err)
{
  uint8_t has_data;
  uint8_t n_bad_rx_frame;
  frame_smartmac_st frame;

  TRACE_LOG_FULL("MAC_RX_BROADCAST: start");

  has_data = FALSE;
  n_bad_rx_frame = 3;
  while (n_bad_rx_frame--) {
    /* the device shall wait for incoming packets for a limited amount of time */
    mac_ule_rxBufTimeout(&mac_ule_tmr1WFD, p_err);
    if (*p_err != NETSTK_ERR_NONE) {
      /* no packet is received in time */
      break;
    } else {
      frame_smartmac_parse(pmac_ule_rxPktPtr, mac_ule_rxPktLen, &frame);
      if (frame.type == FRAME802154_DATAFRAME) {
        has_data = TRUE;
        break;
      }
    }
  }

  if (has_data == TRUE) {
    *p_err = NETSTK_ERR_NONE;
  } else {
    *p_err = NETSTK_ERR_MAC_ULE_NO_DATA;
  }
  TRACE_LOG_FULL("MAC_RX_BROADCAST: stop (-%d)", *p_err);
}


/**
 * @brief   Unicast reception handling
 * @param   p_err   Point to variable holding returned error code
 */
static void mac_ule_rxUnicast(e_nsErr_t *p_err)
{
  int hdrlen;
  frame802154_t frame;

  /*
   * The waiting for payload process shall be declared as done if one of
   * following conditions is met:
   * (a)  A valid data frame has arrived
   * (b)  Waiting timeout is over
   */
  mac_ule_rxBufTimeout(&mac_ule_tmr1WFD, p_err);
  if (*p_err == NETSTK_ERR_NONE) {
    /* parse the received data packet */
    TRACE_LOG_FULL("MAC_RX_DATA: %d bytes ", mac_ule_rxPktLen);
    hdrlen = frame802154_parse(pmac_ule_rxPktPtr, mac_ule_rxPktLen, &frame);
    if (hdrlen > 0) {
      /* Auto-ACK */
      *p_err = NETSTK_ERR_NONE;
      if ((frame.fcf.frame_type == FRAME802154_DATAFRAME) ||
          (frame.fcf.frame_type == FRAME802154_CMDFRAME)) {
        if (frame.fcf.ack_required) {
          mac_ule_txAck(frame.seq, p_err);
        }
      }
    } else {
      /* invalid frame */
      *p_err = NETSTK_ERR_INVALID_FRAME;
    }
  } else {
    *p_err = NETSTK_ERR_MAC_ULE_NO_DATA;
  }

#if (MAC_ULE_CFG_STAT_EN == TRUE)
  /* update local statistical variables */
  mac_ule_nUnicastRx++;
  if (*p_err == NETSTK_ERR_NONE) {
    mac_ule_nUnicastRxOk++;
  }
  /* output statistics */
  TRACE_LOG_MAIN("MAC_RX_Unicast: %5lu / %5lu / -%5d\n", mac_ule_nUnicastRxOk, mac_ule_nUnicastRx, *p_err);
#endif
}


static void mac_ule_txStrobe(uint8_t counter, e_nsErr_t *p_err)
{
  uint8_t strobe[15];
  uint8_t *p_hdr = &strobe[2];
  uint16_t strobe_len;
  frame_smartmac_st frame;

  /* clear all fields */
  memset(&frame, 0, sizeof(frame));

  /* set frame fields */
  frame.type = SMARTMAC_FRAME_STROBE;
  frame.counter = counter;
  frame.pan_id = mac_phy_config.pan_id;
  frame.src_addr = NetstkSrcId;
  frame.dest_addr = NetstkDstId;

  /* write strobe frame to buffer */
  strobe_len = frame_smartmac_create(&frame, p_hdr);

  /* send the frame */
  pmac_ule_netstk->phy->send(p_hdr, strobe_len, p_err);
}


static void mac_ule_txStrobeAck(uint8_t counter, uint16_t dest_addr, e_nsErr_t *p_err)
{
  uint8_t strobe_ack[15];
  uint8_t *p_hdr = &strobe_ack[2];
  uint16_t ack_len;
  frame_smartmac_st frame;

  /* clear all fields */
  memset(&frame, 0, sizeof(frame));

  /* set frame fields */
  frame.type = SMARTMAC_FRAME_STROBE_ACK;
  frame.counter = counter;
  frame.pan_id = mac_phy_config.pan_id;
  frame.src_addr = NetstkSrcId;
  frame.dest_addr = dest_addr;

  /* write strobe ACK frame to buffer */
  ack_len = frame_smartmac_create(&frame, p_hdr);

  /* send the frame */
  pmac_ule_netstk->phy->send(p_hdr, ack_len, p_err);
}


/**
 * @brief   ACK frame transmission
 * @param   seq     sequence number in the ACK frame
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_txAck(uint8_t seq, e_nsErr_t *p_err)
{
  uint8_t buf[9];
  uint8_t hdr_len;
  uint8_t *p_hdr;
  frame802154_t frame;

  /* clear buffer, 2 is maximum PHY header length */
  p_hdr = &buf[2];
  memset(buf, 0, sizeof(buf));
  memset(&frame, 0, sizeof(frame));

  /* Build the FCF. */
  frame.fcf.frame_type = FRAME802154_ACKFRAME;
  frame.fcf.security_enabled = 0;
  frame.fcf.frame_pending = 0;
  frame.fcf.ack_required = 0;
  frame.fcf.panid_compression = 0;

  /* Insert IEEE 802.15.4 (2003) version bits. */
  frame.fcf.frame_version = FRAME802154_IEEE802154_2003;

  /* Increment and set the data sequence number. */
  frame.seq = seq;

  /* Complete the addressing fields. */
  frame.fcf.src_addr_mode = FRAME802154_NOADDR;
  frame.fcf.dest_addr_mode = FRAME802154_NOADDR;

  /* write the header */
  hdr_len = frame802154_create(&frame, p_hdr);

  /* Issue next lower layer to transmit ACK */
  pmac_ule_netstk->phy->send(p_hdr, hdr_len, p_err);
}


/**
 * @brief   Broadcast transmission request handling
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_txBroadcast(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  uint8_t counter;

  /* set destination ID for outgoing broadcast strobes */
  NetstkDstId = MAC_ULE_DEV_ID_BROADCAST;
  TRACE_LOG_FULL("MAC_TX: broadcast");

  /* perform CSMA */
  mac_ule_csma(p_err);
  if (*p_err == NETSTK_ERR_NONE) {

    /* handle strobe transmission */
    counter = MAC_ULE_CFG_BROADCAST_TX_MAX;
    while (counter--) {
      /* transmit strobe frame */
      mac_ule_txStrobe(counter, p_err);
      if (*p_err != NETSTK_ERR_NONE) {
        /* strobe failed to transmit then terminate the strobe transmission */
        break;
      } else {
        /* insert time gap */
        rt_tmr_delay(MAC_ULE_PORT_STROBE_TX_GAP_TIME_IN_MS);

        /* transmit broadcast data packet following last strobe transmission */
        if (counter == 0) {
          pmac_ule_netstk->phy->send(p_data, len, p_err);
          /* transmission has finished */
          break;
        }
      }
    }
  }
}


/**
 * @brief   Unicast transmission request handling
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_txUnicast(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  uint8_t counter;
  frame_smartmac_st frame;
  const linkaddr_t *p_dstaddr;

  /* set destination ID for outgoing broadcast strobes */
  p_dstaddr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  memcpy(&NetstkDstId, &p_dstaddr->u8[6], 2);
  if (NetstkDstId == MAC_ULE_ID_INVALID) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }

#if (MAC_ULE_CFG_LOOSE_SYNC_EN == TRUE)
  mac_ule_calcStrobeDelay(p_err);
#endif

  /* perform CSMA */
  mac_ule_csma(p_err);
  if (*p_err == NETSTK_ERR_NONE) {

    /* reset counter to default maximum value */
    counter = MAC_ULE_CFG_STROBE_TX_MAX;

    /* transmit strobe frame until either counter reaches 0 or a valid strobe ACK is received */
    while (counter--) {
      mac_ule_txStrobe(counter, p_err);
      if (*p_err == NETSTK_ERR_NONE) {
        /* wait for strobe ACK */
        mac_ule_rxBufTimeout(&mac_ule_tmr1WFSA, p_err);
        if (*p_err == NETSTK_ERR_NONE) {
          /* parse the received frame */
          frame_smartmac_parse(pmac_ule_rxPktPtr, mac_ule_rxPktLen, &frame);
          TRACE_LOG_FULL("MAC_RX: Strobe ACK c=%02x d=%04x s=%04x", frame.counter, frame.dest_addr, frame.src_addr);
          if ((frame.type == SMARTMAC_FRAME_STROBE_ACK) &&
              (frame.counter == counter) &&
              (frame.src_addr == NetstkDstId) &&
              (frame.dest_addr == NetstkSrcId)) {
            /* transmit the actual data packet */
            mac_ule_txPayload(p_data, len, p_err);

            #if (MAC_ULE_CFG_LOOSE_SYNC_EN == TRUE)
            /* record power-on schedule of destination node */
            mac_ule_updatePowerOnTable();
            #endif

            break;
          }
        } else {
          /* no strobe ACK is arrived then continue strobe transmission */
          *p_err = NETSTK_ERR_MAC_ULE_NO_STROBE_ACK;
        }
      } else {
        /* strobe failed to transmit then terminate strobe transmission */
        break;
      }
    }

#if (MAC_ULE_CFG_STAT_EN == TRUE)
    uint8_t attempts;
    attempts = MAC_ULE_CFG_STROBE_TX_MAX - counter;

    /* update local statistical variables */
    mac_ule_nUnicastTx++;
    if (*p_err == NETSTK_ERR_NONE) {
      mac_ule_nUnicastTxOk++;
    } else {
      /* output statistics */
      TRACE_LOG_MAIN("MAC_TX_Unicast: %5lu / %5lu / -%5d\n", mac_ule_nUnicastTxOk, mac_ule_nUnicastTx, *p_err);
      TRACE_LOG_MAIN("MAC_TX_Unicast: attempts = %d", attempts);
    }
#endif
  }
}


/**
 * @brief   Actual data packet handling
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_txPayload(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
  int hdrlen;
  uint8_t last_dsn;
  uint8_t is_acked;
  frame802154_t frame;
  packetbuf_attr_t is_ack_req;


  /* send actual data packet */
  TRACE_LOG_FULL("MAC_TX_DATA: %d bytes", len);
  pmac_ule_netstk->phy->send(p_data, len, p_err);

  /* wait for data ACK if needed */
  is_ack_req = (1 == packetbuf_attr(PACKETBUF_ATTR_RELIABLE) &&
               (0 == packetbuf_holds_broadcast()));
  if (is_ack_req == 1) {
    /*
     * Auto-ACK is declared as done if one of following conditions
     * is met:
     * (a)  A corresponding ACK has arrived
     * (b)  Wait-for-ACK timeout is expired
     */
    is_acked = FALSE;
    last_dsn = (uint8_t) packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);

    mac_ule_rxBufTimeout(&mac_ule_tmr1WFA, p_err);
    if (*p_err == NETSTK_ERR_NONE) {
      TRACE_LOG_FULL("MAC_RX_ACK");
      hdrlen = frame802154_parse(pmac_ule_rxPktPtr, mac_ule_rxPktLen, &frame);
      if (hdrlen > 0) {
        if ((frame.fcf.frame_type == FRAME802154_ACKFRAME) && (frame.seq == last_dsn)) {
          /* valid ACK is received */
          is_acked = TRUE;
        }
      }
    }

    /* set return value */
    if (is_acked == TRUE) {
      *p_err = NETSTK_ERR_NONE;
    } else {
      *p_err = NETSTK_ERR_TX_NOACK;
    }
  }
}


/**
 * @brief   Power-Up timer interrupt handler
 */
static void mac_ule_tmrIsrPowerUp(void *p_arg)
{
  if (mac_ule_state == MAC_ULE_STATE_SLEEP) {
    mac_ule_state = MAC_ULE_STATE_SCAN_STARTED;
    evproc_putEvent(E_EVPROC_HEAD, NETSTK_MAC_ULE_EVENT, NULL);
  } else {
    TRACE_LOG_INT("MAC_ON: busy @state: %d, expected: %d", mac_ule_state, MAC_ULE_STATE_SLEEP);
  }
}


/**
 * @brief   MAC ULE CSMA handling
 * @param   p_err   point to variable holding returned error code
 */
static void mac_ule_csma(e_nsErr_t *p_err)
{
  uint8_t is_done;
  uint8_t attempt;

  /* CSMA handling */
  is_done = FALSE;
  attempt = 0;
  TRACE_LOG_FULL("MAC_CSMA: start");
  do {
    /* set return error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* perform CCA in critical section */
    pmac_ule_netstk->phy->ioctrl(NETSTK_CMD_RF_CCA_GET, NULL, p_err);
    if (*p_err == NETSTK_ERR_NONE) {
      attempt++;
      /* wait for a certain period of time before performing the next CCA */
      rt_tmr_delay(4);
    }

    /* check CSMA termination conditions */
    is_done = (attempt > 3) ||
              (*p_err != NETSTK_ERR_NONE);
  } while (is_done == FALSE);
  TRACE_LOG_FULL("MAC_CSMA: stop Error = %d", *p_err);
}



/**
 * @brief Polling for data until either the data is available or timeout is over
 * @param p_tmr
 * @param p_err NETSTK_ERR_NONE when data is available, otherwise NETSTK_ERR_MAC_ULE_NO_DATA
 *
 */
static void mac_ule_rxBufTimeout(s_rt_tmr_t *p_tmr, e_nsErr_t *p_err)
{
  e_rt_tmr_state_t tmr_state;

  /* start polling timer */
  rt_tmr_stop(p_tmr);
  rt_tmr_start(p_tmr);
  mac_ule_rxPktLen = 0;
  do {
    /* get answer to condition (S1a) */
    pmac_ule_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, NULL, p_err);
    if (*p_err == NETSTK_ERR_BUSY) {
      /* wait until RF has completed reception process */
      do {
        pmac_ule_netstk->phy->ioctrl(NETSTK_CMD_RX_BUF_READ, NULL, p_err);
        pmac_ule_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, NULL, p_err);
      } while (*p_err == NETSTK_ERR_BUSY);
      break;
    }

    /* get answer to condition (S1b) */
    tmr_state = rt_tmr_getState(p_tmr);
  } while (tmr_state == E_RT_TMR_STATE_RUNNING);

  /* stop polling timer */
  rt_tmr_stop(p_tmr);

  /* set return error code */
  if (mac_ule_rxPktLen == 0) {
    *p_err = NETSTK_ERR_MAC_ULE_NO_DATA;
  } else {
    *p_err = NETSTK_ERR_NONE;
  }
}


#if (MAC_ULE_CFG_LOOSE_SYNC_EN == TRUE)
static void mac_ule_updatePowerOnTable(void)
{
  uint8_t ix;

  for (ix = 0; ix < MAC_ULE_CFG_PWRON_TBL_SIZE ; ix++) {
    if (mac_ule_pwrOnTable[ix].DestId == NetstkDstId) {
      mac_ule_pwrOnTable[ix].LastWakeup = rt_tmr_getCurrenTick();
      break;
    }
  }
}


static void mac_ule_calcStrobeDelay(e_nsErr_t *p_err)
{
  uint8_t on_qty;
  uint8_t ix;
  uint32_t on_next;
  uint32_t tx_delay;
  uint32_t tx_advance;
  uint32_t min_delay;
  uint32_t last_seen;
  rt_tmr_tick_t curr_time;

  /*
   * look for records with regard to the destination node
   */
  tx_delay = 0;
  last_seen = 0;
  for (ix = 0; ix < MAC_ULE_CFG_PWRON_TBL_SIZE ; ix++) {
    if (mac_ule_pwrOnTable[ix].DestId == NetstkDstId) {
      last_seen = mac_ule_pwrOnTable[ix].LastWakeup;
      break;
    }
  }

  if (last_seen == 0) {
    /* the record has not found then allocate an entry for the destination node */
    for (ix = 0; ix < MAC_ULE_CFG_PWRON_TBL_SIZE ; ix++) {
      if (mac_ule_pwrOnTable[ix].LastWakeup == 0) {
        mac_ule_pwrOnTable[ix].DestId = NetstkDstId;
        break;
      }
    }
  } else {
    /*
     * Perform prediction on next waking-up interval of the intended receiver
     */
    ix = 0;
    curr_time = rt_tmr_getCurrenTick();
    while (tx_delay == 0) {
      /* go to next power-on schedule */
      ix++;

      /* estimation */
      on_qty = (curr_time - last_seen) / MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS;       /* Number of power-on interval in between   */
      on_next = last_seen + (on_qty + ix) * MAC_ULE_CFG_POWERUP_INTERVAL_IN_MS;  /* Predicted next power-on time interval    */
      tx_delay = (on_next - curr_time);

      /*
       * Note(s):
       *
       * (1)  Minimum possible delay shall be calculated as following:
       *      min_delay = T_off_to_on + T_on_to_off + T_scan + T_tx_smartpreamble
       */
      tx_advance = MAC_ULE_CFG_QTY_STROBE_SENT_IN_ADVANCE * MAC_ULE_PORT_STROBE_TX_INTERVAL_IN_MS;
      min_delay = MAC_ULE_PORT_OFF_TO_ON_TIME_IN_MS +
                  MAC_ULE_PORT_ON_TO_OFF_TIME_IN_MS +
                  MAC_ULE_PORT_SCAN_DURATION_IN_MS;

      if (tx_delay > min_delay) {
        tx_delay -= min_delay;
        if (tx_delay > tx_advance) {
          tx_delay -= tx_advance;
        }
      } else {
        tx_delay = 0;
      }
    }

    if (tx_delay > 0) {
      mac_ule_off(p_err);
      rt_tmr_delay(tx_delay);
      mac_ule_on(p_err);
    }
  }
}


static void mac_ule_calcDataDelay(uint8_t counter, e_nsErr_t *p_err)
{
  rt_tmr_tick_t rx_delay = 0;
  rt_tmr_tick_t delay_min;

  if (counter) {
    /*
     * Note:
     *
     * If calculated waiting time before sending an ACK is smaller than
     * or equal to minimum offset in the receiver caused by off-to-on
     * and on-to-off transition time, there is no need to go back to
     * sleep. Instead the receive should replay with an ACK immediately
     * as in SmartMAC framer.
     */
    rx_delay = counter * MAC_ULE_PORT_STROBE_TX_INTERVAL_IN_MS;
    delay_min = MAC_ULE_PORT_ON_TO_OFF_TIME_IN_MS + MAC_ULE_PORT_OFF_TO_ON_TIME_IN_MS;
    if (rx_delay > delay_min) {
      rx_delay -= delay_min;
    } else {
      rx_delay = 0;
    }
  }

  if (rx_delay > 0) {
    mac_ule_off(p_err);
    rt_tmr_delay(rx_delay);
    mac_ule_on(p_err);
  }
}
#endif


/*
********************************************************************************
*                                   END OF FILE
********************************************************************************
*/
