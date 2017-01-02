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
 * @file    mac_802154.c
 * @date    19.10.2015
 * @author  PN
 */


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"

#include "evproc.h"
#include "framer_802154.h"
#include "packetbuf.h"
#include "random.h"
#include "rt_tmr.h"


#define     LOGGER_ENABLE        LOGGER_MAC
#include    "logger.h"


/*
********************************************************************************
*                               LOCAL MACROS
********************************************************************************
*/
#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
  #if (NETSTK_CFG_WOR_EN == TRUE)
    #define MAC_CFG_TMR_WFA_IN_MS               (uint32_t )( 12 )
  #else
    #define MAC_CFG_TMR_WFA_IN_MS               (uint32_t )(  5 )
  #endif
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */


/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void mac_init(void *p_netstk, e_nsErr_t *p_err);
static void mac_on(e_nsErr_t *p_err);
static void mac_off(e_nsErr_t *p_err);
static void mac_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
static void mac_txAck(uint8_t seq, e_nsErr_t *p_err);
static void mac_rxBufTimeout(s_rt_tmr_t *p_tmr, e_nsErr_t *p_err);
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */

static void mac_csma(e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
/**  \brief The sequence number (0x00 - 0xff) added to the transmitted
 *   data or MAC command frame. The default is a random value within
 *   the range.
 */
static uint8_t          mac_isAckReq;
static s_ns_t          *pmac_netstk;
static void            *pmac_cbTxArg;
static nsTxCbFnct_t     mac_cbTxFnct;
static e_nsErr_t        mac_txErr;
static uint8_t          mac_hasData;

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
static s_rt_tmr_t       mac_tmrWfa;
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsMAC_t mac_driver_802154 =
{
 "MAC 802154",
  mac_init,
  mac_on,
  mac_off,
  mac_send,
  mac_recv,
  mac_ioctl,
};

extern uip_lladdr_t uip_lladdr;


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
void mac_init(void *p_netstk, e_nsErr_t *p_err)
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

  /* initialize local variables */
  pmac_netstk = (s_ns_t *) p_netstk;
  mac_cbTxFnct = 0;
  pmac_cbTxArg = NULL;
  mac_isAckReq = 0;
  mac_txErr = NETSTK_ERR_NONE;

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
  rt_tmr_create(&mac_tmrWfa, E_RT_TMR_TYPE_ONE_SHOT, MAC_CFG_TMR_WFA_IN_MS, 0, NULL);
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */

  /*
   * Configure stack address
   */
  memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
  linkaddr_set_node_addr((linkaddr_t *) mac_phy_config.mac_address);

  /* initialize MAC PIB attributes */
  packetbuf_attr_t macAckWaitDuration;
  packetbuf_attr_t macUnitBackoffPeriod;
  packetbuf_attr_t phyTurnaroundTime;
  packetbuf_attr_t phySHRDuration;
  packetbuf_attr_t phySymbolsPerOctet;
  packetbuf_attr_t phySymbolPeriod;

  phySHRDuration = packetbuf_attr(PACKETBUF_ATTR_PHY_SHR_DURATION);
  phyTurnaroundTime = packetbuf_attr(PACKETBUF_ATTR_PHY_TURNAROUND_TIME);
  phySymbolsPerOctet = packetbuf_attr(PACKETBUF_ATTR_PHY_SYMBOLS_PER_OCTET);
  phySymbolPeriod = packetbuf_attr(PACKETBUF_ATTR_PHY_SYMBOL_PERIOD);

  macUnitBackoffPeriod = 20 * phySymbolPeriod;
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_UNIT_BACKOFF_PERIOD, macUnitBackoffPeriod);

  /* compute and set macAckWaitDuration attribute, see IEEE Std. 802.15.4(g) */
  macAckWaitDuration = macUnitBackoffPeriod + phyTurnaroundTime + phySHRDuration;
#if (NETSTK_CFG_MR_FSK_PHY_EN == TRUE)
  macAckWaitDuration += 9 * phySymbolsPerOctet * phySymbolPeriod;
#else
  macAckWaitDuration += 6 * phySymbolsPerOctet * phySymbolPeriod;
#endif /* NETSTK_CFG_MR_FSK_PHY_EN */

  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION, macAckWaitDuration);

  /* set returned error */
  *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pmac_netstk->phy->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pmac_netstk->phy->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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

#if (EMB6_TEST_CFG_CONT_RX_EN == TRUE)
  *p_err = NETSTK_ERR_BUSY;

  /* was transmission callback function set? */
  if (mac_cbTxFnct) {
    /* then signal the upper layer of the result of transmission process */
    mac_cbTxFnct(pmac_cbTxArg, p_err);
  }
  return;
#endif

#if (EMB6_TEST_CFG_CONT_TX_EN == TRUE)
  uint16_t ix;

  packetbuf_attr_t waitForAckTimeout;
  waitForAckTimeout = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
#include "crc.h"
#define MINLEN        12u
#define MAXLEN      2049u

  uint8_t fcs_len = packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);

  uint8_t dummyData[2 + MAXLEN];  // maximum length
  uint16_t payloadLen;
  uint16_t checksumDataLen;
  uint16_t txPktCount = 0;
  uint8_t *p_mhr = &dummyData[2];
  uint32_t fcs;
  uint8_t *p_fcs;

  uint8_t dummyMhr[] = {0x41, 0xd8, 0x00,
                        0xcd, 0xab, 0xff, 0xff,
                        0xfe, 0xca}; //len = 9

  for (payloadLen = MINLEN; payloadLen <= MAXLEN; payloadLen++) {
    /* create data */
    memset(p_mhr, 0xab, sizeof(dummyData));
    memcpy(p_mhr, dummyMhr, sizeof(dummyMhr));

    /* allocate for CRC */
    checksumDataLen = payloadLen - fcs_len;

    /* added CRC */
    p_fcs = p_mhr + checksumDataLen;
    if (fcs_len == 4) {
      /* 32-bit CRC */
      fcs = crc_32_calc(p_mhr, checksumDataLen);
      p_fcs[0] = (fcs & 0xFF000000u) >> 24;
      p_fcs[1] = (fcs & 0x00FF0000u) >> 16;
      p_fcs[2] = (fcs & 0x0000FF00u) >> 8;
      p_fcs[3] = (fcs & 0x000000FFu);
    }
    else {
      /* 16-bit CRC */
      fcs = crc_16_calc(p_mhr, checksumDataLen);
      p_fcs[0] = (fcs & 0xFF00u) >> 8;
      p_fcs[1] = (fcs & 0x00FFu);
    }

    /* issue TX request */
    pmac_netstk->phy->send(p_mhr, payloadLen, p_err);
    bsp_delay_us(waitForAckTimeout);

    if (*p_err == NETSTK_ERR_NONE) {
      txPktCount++;
    }
  }

#else
  for (ix = 0; ix < 5000; ix++) {
    pmac_netstk->phy->send(p_data, len, p_err);
    bsp_delayUs(waitForAckTimeout);
  }
#endif

  while (1);

  *p_err = NETSTK_ERR_BUSY;

  /* was transmission callback function set? */
  if (mac_cbTxFnct) {
    /* then signal the upper layer of the result of transmission process */
    mac_cbTxFnct(pmac_cbTxArg, p_err);
  }
  return;
#endif


  LOG_INFO("MAC_TX: Transmit %d bytes.", len);

  uint8_t is_tx_done;
  uint8_t tx_retries;
  uint8_t tx_retriesMax;

  /* find out if ACK is required */
  mac_isAckReq = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);

  /*
   * Transmission handling
   */
  tx_retriesMax = packetbuf_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS);
  tx_retries = 0;
  is_tx_done = FALSE;

  /* set result of TX process to default */
  mac_txErr = NETSTK_ERR_TX_NOACK;

  /* perform CSMA-CA */
  mac_csma(p_err);

  /* was channel free? */
  if (*p_err == NETSTK_ERR_NONE) {
    /* then attempt to transmit the packet */
    pmac_netstk->phy->send(p_data, len, p_err);
  }

  /* is ACK required? */
  if (mac_isAckReq == TRUE) {

  /* is software Auto-ACK feature of MAC disabled? */
#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == FALSE)
    {
      /* has the frame not been acknowledged? */
      if (*p_err == NETSTK_ERR_TX_NOACK) {
        /* then perform retransmission */
        tx_retries++;

        /* iterates retransmission as long as number of retries does not exceed
        * the maximum retries and ACK was not arrived
        */
        while ((tx_retries < tx_retriesMax) && (is_tx_done == FALSE)) {
          TRACE_LOG_ERR("+ MAC_TX: seq=%02x; retry=%d; err=-%d", p_data[2], tx_retries, *p_err);

          /* perform unslotted CSMA-CA */
          mac_csma(p_err);

          /* is channel free? */
          if (*p_err == NETSTK_ERR_NONE) {
            /* then retransmit the frame */
            pmac_netstk->phy->send(p_data, len, p_err);
            /* was ACK not arrived? */
            if (*p_err == NETSTK_ERR_TX_NOACK) {
              /* then increase number of retries */
              tx_retries++;
            } else {
              /* otherwise terminate the transmission */
              is_tx_done = TRUE;
              TRACE_LOG_ERR("+ MAC_TX: TX done, r=%d, e=-%d", tx_retries, *p_err);
            }
          }
          /* was channel free or was the radio busy */
          else {
            /* then terminate transmission process */
            is_tx_done = TRUE;
            TRACE_LOG_ERR("+ MAC_TX: CCA failed, r=%d, e=%d", tx_retries, *p_err);
          }
        }
      }
    }
#else
    {
      /* was the packet successfully transmitted? */
      if (*p_err == NETSTK_ERR_NONE) {
        /* then waits for ACK */
        is_tx_done = FALSE;
        do {
          tx_retries++;
          /* polling for ACK until timeout is expired */
          mac_rxBufTimeout(&mac_tmrWfa, p_err);
          /* was no packet arrived during AckWaitDuration? */
          if (*p_err == NETSTK_ERR_FATAL) {
            TRACE_LOG_ERR("MAC_TX: WFA timeout seq=%02x, r=%d", p_data[2], tx_retries);
            /* was number of retries smaller than maximum retry? */
            if (tx_retries < tx_retriesMax) {
              /* then check if channel is free */
              mac_csma(p_err);
              /* was the channel free? */
              if (*p_err == NETSTK_ERR_NONE) {
                /* then retransmit the frame */
                #if (NETSTK_CFG_RF_RETX_EN == TRUE)
                pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_RETX, NULL, p_err);
                #else
                pmac_netstk->phy->send(p_data, len, p_err);
                #endif
                /* has frame failed to transmit? */
                if (*p_err != NETSTK_ERR_NONE) {
                  /* then terminate the transmission */
                  is_tx_done = TRUE;
                  TRACE_LOG_ERR("MAC_TX: TX failed, r=%d", tx_retries);
                }
              }
              /* was channel free or was the radio busy */
              else {
                is_tx_done = TRUE;
                TRACE_LOG_ERR("MAC_TX: CCA failed, r=%d, e=%d", tx_retries, *p_err);
              }
            }
            /* was number of retries equal or greater than the maximum retry? */
            else {
              /* then terminate the transmission process */
              is_tx_done = TRUE;
              *p_err = NETSTK_ERR_TX_NOACK;
              TRACE_LOG_ERR("MAC_TX: NO_ACK, r=%d", tx_retries);
            }
          }
          /* was a packet arrived during ACK-wait duration? */
          else {
            /* then TX result is set in mac_recv() and terminate the TX process */
            *p_err = mac_txErr;
            is_tx_done = TRUE;
          }
        } while (is_tx_done == FALSE);
      }
    }
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == FALSE) */
  }
  /* is ACK not required? */
  else {
    /* then terminate transmission process */
  }


  /* reset local variables */
  mac_isAckReq = 0;
  mac_txErr = NETSTK_ERR_NONE;
  TRACE_LOG_MAIN("MAC_TX: finished e=-%d", *p_err);
  LOG_INFO("MAC_TX: --> Done - TX Status %d (%d/%d retries).", *p_err, tx_retries, tx_retriesMax);

  /* was transmission callback function set? */
  if (mac_cbTxFnct) {
    /* then signal the upper layer of the result of transmission process */
    mac_cbTxFnct(pmac_cbTxArg, p_err);
  }
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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

  int hdrlen;
#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
  uint8_t exp_seq;
  uint8_t is_acked;
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */
  frame802154_t frame;

  /* set returned error code to default */
  *p_err = NETSTK_ERR_NONE;

  /* was packet length larger than size of packet buffer? */
  if (len > PACKETBUF_SIZE) {
    /* then discard the packet to avoid buffer overflow when using memcpy to
    * store the frame into the packet buffer
    */
    *p_err = NETSTK_ERR_INVALID_FRAME;
    TRACE_LOG_ERR("MAC_RX: invalid length");
    return;
  }

  /* Parsing but not reducing header as that will be then handled by DLLC */
  hdrlen = frame802154_parse(p_data, len, &frame);
  if (hdrlen == 0) {
    *p_err = NETSTK_ERR_INVALID_FRAME;
    TRACE_LOG_ERR("MAC_RX: bad format");
    return;
  }

  /* a valid frame has arrived */
  mac_hasData = 1;

  /* was MAC waiting for ACK? */
  if (mac_isAckReq) {
#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
    /* then frames other than ACK shall be discarded silently */

    /* check if this is expected ACK */
    exp_seq = (uint8_t) packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
    is_acked = ((frame.seq == exp_seq) &&
                (frame.fcf.frame_type == FRAME802154_ACKFRAME));
    if (is_acked) {
      /* valid ACK has arrived */
      mac_txErr = NETSTK_ERR_NONE;
    } else {
      /* unexpected packet arrives while waiting for ACK */
      mac_txErr = NETSTK_ERR_TX_COLLISION;
      TRACE_LOG_ERR("MAC_TX: collided");
    }
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */
  }
  else {
    switch (frame.fcf.frame_type) {
      case FRAME802154_DATAFRAME:
      case FRAME802154_CMDFRAME:
#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
        /* perform Auto-ACK */
        if ((frame.fcf.ack_required == 1) &&
            (frame.dest_pid == mac_phy_config.pan_id) &&
            (frame802154_broadcast(&frame) == 0) &&
            (linkaddr_cmp((linkaddr_t *) frame.dest_addr, &linkaddr_node_addr)) == 1) {
          mac_txAck(frame.seq, p_err);
        }
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */

        /* signal upper layer of the received packet */
        pmac_netstk->dllc->recv(p_data, len, p_err);
        break;

      case FRAME802154_ACKFRAME:
        /* silently discard unwanted ACK */
        *p_err = NETSTK_ERR_NONE;
        break;

      default:
        *p_err = NETSTK_ERR_INVALID_FRAME;
        break;
    }
  }
  LOG_INFO("MAC_RX: Received %d bytes.", len);
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
void mac_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    case NETSTK_CMD_TX_CBFNCT_SET:
      if (p_val == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      } else {
        mac_cbTxFnct = (nsTxCbFnct_t) p_val;
      }
      break;

    case NETSTK_CMD_TX_CBARG_SET:
      pmac_cbTxArg = p_val;
      break;

    default:
      pmac_netstk->phy->ioctrl(cmd, p_val, p_err);
      break;
  }
}

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
/**
 * @brief   ACK transmission
 *
 * @param   seq     Frame sequence number of the outgoing ACK
 */
static void mac_txAck(uint8_t seq, e_nsErr_t *p_err)
{
  uint8_t buf[9];
  uint8_t ack_len;
  uint8_t *p_ack;
  frame802154_t frame;

  /* clear buffer, 2 is maximum PHY header length */
  p_ack = &buf[2];
  memset(buf, 0, sizeof(buf));
  memset(&frame, 0, sizeof(frame));

  /* Build the FCF. */
  frame.fcf.frame_type = FRAME802154_ACKFRAME;

  /* following fields are already set to zero
  * - security enabled
  * - frame pending
  * - ACK-required
  * - PAN-ID compression
  */

  /* Insert IEEE 802.15.4 (2006) version bits. */
  frame.fcf.frame_version = FRAME802154_IEEE802154_2006;

  /* Increment and set the data sequence number. */
  frame.seq = seq;

  /* Complete the addressing fields. */
  frame.fcf.src_addr_mode = FRAME802154_NOADDR;
  frame.fcf.dest_addr_mode = FRAME802154_NOADDR;

  /* write the header */
  ack_len = frame802154_create(&frame, p_ack);

  /* Issue next lower layer to transmit ACK */
  pmac_netstk->phy->send(p_ack, ack_len, p_err);
  LOG_INFO("MAC_TX: ACK %d.", frame.seq);
}
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */


/**
 * @brief   This function performs CSMA-CA mechanism.
 *
 * @param   p_err   Pointer to a variable storing returned error code
 */
static void mac_csma(e_nsErr_t *p_err)
{
  uint32_t delay = 0;
  uint32_t max_random;
  uint8_t nb;
  uint8_t be;

  /* initialize CSMA variables */
  nb = 0;
  be = NETSTK_CFG_CSMA_MIN_BE;
  *p_err = NETSTK_ERR_NONE;

  /* perform CCA maximum MaxBackoff time */
  while (nb <= NETSTK_CFG_CSMA_MAX_BACKOFF) {
    /* delay for random (2^BE - 1) unit backoff periods */
    max_random = (1 << be) - 1;
    delay  = bsp_getrand(0, max_random);
    delay *= NETSTK_CFG_CSMA_UNIT_BACKOFF_US;
    bsp_delayUs(delay);

    /* perform CCA */
    pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_CCA_GET, 0, p_err);
    /* was channel free or was the radio busy? */
    if (*p_err == NETSTK_ERR_NONE) {
      /* channel free */
      break;
    }
    else if (*p_err == NETSTK_ERR_BUSY) {
      /* radio is likely busy receiving a packet and therefore should let it handle the received packet now */
      *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
      break;
    }
    /* was channel busy? */
    else {
      /* then increase number of backoff by one */
      nb++;
      /* be = MIN((be + 1), MaxBE) */
      be = ((be + 1) < NETSTK_CFG_CSMA_MAX_BE) ? (be + 1) : (NETSTK_CFG_CSMA_MAX_BE);
    }
  }
  LOG_INFO("MAC_TX: NB %d.", nb);
}

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
/**
 * @brief Polling for data until either the data is available or timeout is over
 * @param p_tmr
 * @param p_err NETSTK_ERR_NONE when data is available, otherwise NETSTK_ERR_MAC_ULE_NO_DATA
 *
 */
static void mac_rxBufTimeout(s_rt_tmr_t *p_tmr, e_nsErr_t *p_err)
{
  e_rt_tmr_state_t tmr_state;

  /* start polling timer */
  rt_tmr_stop(p_tmr);
  rt_tmr_start(p_tmr);
  mac_hasData = 0;
  do {
    /* check if RF is in reception process */
    pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, NULL, p_err);
    if (*p_err == NETSTK_ERR_BUSY) {
      /* wait until RF has completed reception process i.e., the radio is not
      * busy anymore
      */
      do {
        pmac_netstk->phy->ioctrl(NETSTK_CMD_RX_BUF_READ, NULL, p_err);
        pmac_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, NULL, p_err);
      } while (*p_err == NETSTK_ERR_BUSY);
      break;
    }

    /* get state of the timeout timer */
    tmr_state = rt_tmr_getState(p_tmr);
  } while (tmr_state == E_RT_TMR_STATE_RUNNING);

  /* stop polling timer */
  rt_tmr_stop(p_tmr);

  /* set return error code */
  if (mac_hasData == 0) {
    *p_err = NETSTK_ERR_FATAL;
  } else {
    *p_err = NETSTK_ERR_NONE;
  }
}
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
