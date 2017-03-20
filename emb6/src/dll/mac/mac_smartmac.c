/**
 * @file	  smartMAC.c
 * @date	  26.06.2016
 * @author 	Phuong Nguyen
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
#include "crc.h"
#include "phy_framer_802154.h"
#include "framer_802154.h"
#include "framer_smartmac.h"

#include "ctimer.h"
#include "rt_tmr.h"

/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define SMARTMAC_CFG_WAKEUP_INTERVAL_OFFSET_MAX     10u
#define SMARTMAC_CFG_WAKEUP_TBL_SIZE                5u
#define SMARTMAC_CFG_MIN_NUM_STROBE_TO_BE_SENT      1u

/*
********************************************************************************
*                               LOCAL TYPEDEF
********************************************************************************
*/
typedef enum
{
  E_SMARTMAC_STATE_NON_INIT     = 0x00,
  E_SMARTMAC_STATE_OFF          = 0x10,
  E_SMARTMAC_STATE_ON           = 0x20,
  E_SMARTMAC_STATE_SCAN         = 0x30,
  E_SMARTMAC_STATE_RX           = 0x40,
  E_SMARTMAC_STATE_RX_PENDING   = 0x41,
  E_SMARTMAC_STATE_RX_DELAY     = 0x42,
  E_SMARTMAC_STATE_TX           = 0x50,
  E_SMARTMAC_STATE_TX_DELAY     = 0x51,
  E_SMARTMAC_STATE_TX_BROADCAST = 0x52,
  E_SMARTMAC_STATE_TX_UNICAST   = 0x53,
  E_SMARTMAC_STATE_ERR          = 0x60,
}e_smartmacState;

typedef enum
{
  E_SMARTMAC_EVENT_RX_EXTEND,
  E_SMARTMAC_EVENT_RX_DELAY,
  E_SMARTMAC_EVENT_ASYNC_SCAN_EXIT,
} e_smartmacEvent;

struct s_wakeupTableEntry {
  uint32_t lastWakeupInterval;
  uint16_t destId;
  uint16_t numStrobeSent;
};

struct s_smartmac {
  s_ns_t           *p_netstk;
  void             *p_cbTxArg;
  nsTxCbFnct_t      cbTxFnct;
  uint32_t          rxDelay;
  e_smartmacState   state;
  uint8_t           event;

  /* smartmac timing parameters in milliseconds that are relied on underlying layer */
  uint32_t          sleepTimeout;
  uint8_t           strobeTxInterval;
  uint8_t           scanTimeout;
  uint8_t           lbtTimeout;
  uint8_t           rxDelayMin;
  uint8_t           rxTimeout;
  uint8_t           rxPendingTimeoutCount;
  uint8_t           maxUnicastCounter;
  uint8_t           maxBroadcastCounter;

  /* use more precise timer for wake-up timer */
  s_rt_tmr_t        tmrWakeup;
  struct ctimer     tmr1Scan;
  struct ctimer     tmr1RxPending;
  struct ctimer     tmr1RxDelay;
  struct ctimer     tmr1TxDelay;

#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
  struct s_wakeupTableEntry wakeupTable[SMARTMAC_CFG_WAKEUP_TBL_SIZE];
#endif
};


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void smartmac_init (void *p_netstk, e_nsErr_t *p_err);
static void smartmac_start(e_nsErr_t *p_err);
static void smartmac_stop(e_nsErr_t *p_err);
static void smartmac_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void smartmac_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void smartmac_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static void mac_txStrobe(struct s_smartmac *p_ctx, uint8_t counter, uint16_t dstShortAddr, e_nsErr_t *p_err);
static void mac_txBroadcast(struct s_smartmac *p_ctx, uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_txUnicast(struct s_smartmac *p_ctx, uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void mac_lbt(struct s_smartmac *p_ctx, e_nsErr_t *p_err);

static void mac_eventHandler(c_event_t c_event, p_data_t p_data);

/* timeout callback functions */
static void mac_tmrWakeupCb(void *p_arg);
static void mac_tmrScanCb(void *p_arg);
static void mac_tmrRxPendingCb(void *p_arg);
static void mac_tmrRxDelayCb(void *p_arg);

static uint32_t mac_calcRxDelay(struct s_smartmac *p_ctx, uint8_t counter);

#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
static void mac_updateWakeupTable(struct s_smartmac *p_ctx, uint16_t destId);
static uint32_t mac_calcTxDelay(struct s_smartmac *p_ctx, uint16_t destId);
#endif

/* state-event handling functions */
static void mac_off_entry(struct s_smartmac *p_ctx);
static void mac_off_sleepTimeout(struct s_smartmac *p_ctx);
static void mac_off_exit(struct s_smartmac *p_ctx);

static void mac_on_entry(struct s_smartmac *p_ctx);
static void mac_on_exit(struct s_smartmac *p_ctx);

static void mac_scan_entry(struct s_smartmac *p_ctx);
static void mac_scan_exit(struct s_smartmac *p_ctx);

static void mac_rxPending_entry(struct s_smartmac *p_ctx);
static void mac_rxPending_exit(struct s_smartmac *p_ctx);

static void mac_rxDelay_entry(struct s_smartmac *p_ctx);
static void mac_rxDelay_exit(struct s_smartmac *p_ctx);

#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
static void mac_txDelay_entry(struct s_smartmac *p_ctx);
static void mac_txDelay_exit(struct s_smartmac *p_ctx);
#endif

static void mac_txBroadcast_entry(struct s_smartmac *p_ctx);
static void mac_txBroadcast_exit(struct s_smartmac *p_ctx);

static void mac_txUnicast_entry(struct s_smartmac *p_ctx);
static void mac_txUnicast_exit(struct s_smartmac *p_ctx);

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static struct s_smartmac smartmac;
static uint8_t smartmacStrobe[15];

/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsMAC_t mac_driver_smartmac =
{
   "smartmac",
    smartmac_init,
    smartmac_start,
    smartmac_stop,
    smartmac_send,
    smartmac_recv,
    smartmac_ioctl,
};

extern uip_lladdr_t uip_lladdr;


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void smartmac_init (void *p_netstk, e_nsErr_t *p_err) {
  struct s_smartmac *p_ctx = &smartmac;
  packetbuf_attr_t macShortAddr;
  uint8_t strobeLen;

  /* store pointer to the network stack */
  p_ctx->p_netstk = (s_ns_t *)p_netstk;

  /* set MAC address */
  memcpy(&uip_lladdr.addr, &mac_phy_config.mac_address, 8);
  linkaddr_set_node_addr((linkaddr_t *) mac_phy_config.mac_address);

  macShortAddr = (mac_phy_config.mac_address[7]     ) |
                 (mac_phy_config.mac_address[6] << 8);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SHORT_ADDR, macShortAddr);

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

  /* compute and set macAckWaitDuration attribute */
  macAckWaitDuration = macUnitBackoffPeriod + phyTurnaroundTime + phySHRDuration;
#if (NETSTK_CFG_MR_FSK_PHY_EN == TRUE)
  macAckWaitDuration += 9 * phySymbolsPerOctet * phySymbolPeriod;
#else
  macAckWaitDuration += 6 * phySymbolsPerOctet * phySymbolPeriod;
#endif
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION, macAckWaitDuration);

  /* set smartmac attributes */
  strobeLen = framer_smartmac_getStrobeLen();
  p_ctx->strobeTxInterval = (phySHRDuration + (8 * strobeLen) * phySymbolPeriod + macAckWaitDuration) / 1000 + 1; /* actual TX time is about 1ms longer than expected */
  p_ctx->scanTimeout = p_ctx->strobeTxInterval + 5; /* >=16ms = 4 sniffs */
  p_ctx->lbtTimeout = p_ctx->scanTimeout;
#if (NETSTK_CFG_LOW_POWER_MODE_EN == TRUE)
  p_ctx->sleepTimeout = mac_phy_config.sleepTimeout;
#endif
  p_ctx->rxTimeout = 3 * p_ctx->strobeTxInterval;
  p_ctx->rxDelayMin = 3 * p_ctx->strobeTxInterval;
  p_ctx->maxBroadcastCounter = (p_ctx->sleepTimeout + 2 * p_ctx->scanTimeout ) / p_ctx->strobeTxInterval + 1;
  p_ctx->maxUnicastCounter = 2 * p_ctx->maxBroadcastCounter;
  memset(smartmacStrobe, 0, sizeof(smartmacStrobe));

  /* initialize local attributes */
  rt_tmr_create(&p_ctx->tmrWakeup, E_RT_TMR_TYPE_PERIODIC, p_ctx->sleepTimeout, mac_tmrWakeupCb, p_ctx);
  ctimer_set(&p_ctx->tmr1Scan, p_ctx->scanTimeout, mac_tmrScanCb, p_ctx);
  ctimer_set(&p_ctx->tmr1RxPending, p_ctx->rxTimeout, mac_tmrRxPendingCb, p_ctx);

  ctimer_stop(&p_ctx->tmr1Scan);
  ctimer_stop(&p_ctx->tmr1RxPending);

  /* register MAC event */
  evproc_regCallback(EVENT_TYPE_MAC_ULE, mac_eventHandler);

  /* initial transition */
  mac_off_entry(p_ctx);
}


static void smartmac_start(e_nsErr_t *p_err) {
  struct s_smartmac *p_ctx = &smartmac;

  /* start wake-up timer */
  rt_tmr_start(&p_ctx->tmrWakeup);
  *p_err = NETSTK_ERR_NONE;
}


static void smartmac_stop(e_nsErr_t *p_err) {

}


static void smartmac_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err) {
  struct s_smartmac *p_ctx = &smartmac;
  int isBroadcastTx;

  /* is MAC performing channel scan? */
  if (p_ctx->state == E_SMARTMAC_STATE_SCAN) {
    /* then terminate the channel scan and start TX process */
    mac_scan_exit(p_ctx);
  }
  else if (p_ctx->state == E_SMARTMAC_STATE_OFF) {
    /* state transition: OFF -> ON */
    mac_off_exit(p_ctx);
    mac_on_entry(p_ctx);
  }
  else {
    /* MAC is busy performing other tasks which cannot be overtaken */
    *p_err = NETSTK_ERR_BUSY;
    TRACE_LOG_ERR("smartMAC busy, %02x", p_ctx->state);

    /* was transmission callback function set? */
    if (p_ctx->cbTxFnct) {
      /* then signal the upper layer of the result of transmission process */
      p_ctx->cbTxFnct(p_ctx->p_cbTxArg, p_err);
    }
    return;
  }

  /* handle transmission */
  isBroadcastTx = packetbuf_holds_broadcast();
  if (*p_err == NETSTK_ERR_NONE) {
    if (isBroadcastTx) {
      mac_txBroadcast(p_ctx, p_data, len, p_err);
    }
    else {
      mac_txUnicast(p_ctx, p_data, len, p_err);
    }

    /* state transition: ON -> OFF */
    mac_on_exit(p_ctx);
    mac_off_entry(p_ctx);
  }
  else {
    mac_scan_entry(p_ctx);
  }

  /* was transmission callback function set? */
  if (p_ctx->cbTxFnct) {
    /* then signal the upper layer of the result of transmission process */
    p_ctx->cbTxFnct(p_ctx->p_cbTxArg, p_err);
  }
}


static void smartmac_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err) {
  struct s_smartmac *p_ctx = &smartmac;
  frame_smartmac_st frame;

  /* set return error code to default value */
  *p_err = NETSTK_ERR_NONE;

  /* parse the received frame */
  framer_smartmac_parse(p_data, len, &frame);

  /* is MAC in SCAN state? */
  if ((p_ctx->state == E_SMARTMAC_STATE_SCAN)) {
    /* is the received frame a strobe? */
    if (frame.type == SMARTMAC_FRAME_STROBE) {
      /* is the pending frame a broadcast frame? */
      if (frame.dest_addr == FRAME802154_BROADCASTADDR) {
        /* then calculate waiting time before the actual data arrives */
        p_ctx->rxDelay = mac_calcRxDelay(p_ctx, frame.counter);
        if (p_ctx->rxDelay > 0) {
          /* state transition to RX_DELAY is handled in mac_eventHandler()
           * As such radio has a chance to perform state transition after RX_FINI */
          p_ctx->event = E_SMARTMAC_EVENT_RX_DELAY;
          evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_MAC_ULE, &p_ctx->event);
        }
        else {
          /* is this last strobe? */
          if (frame.counter == 0) {
            /* then exits SCAN and enters RXPENDING */
            mac_scan_exit(p_ctx);
            mac_rxPending_entry(p_ctx);
          }
          else {
            /* the actual data frame is coming soon, then simply transition to
             * SCAN state. As such all incoming frame is treated in same manner */
            mac_scan_exit(p_ctx);
            mac_scan_entry(p_ctx);
          }
        }
      }
      else {
        /* otherwise the pending frame is a unicast frame, and therefore MAC
         * shall stay on for a certain amount of time to receive the frame */
        mac_scan_exit(p_ctx);
        mac_rxPending_entry(p_ctx);
      }
    }
    else {
      /* otherwise the received frame is not a strobe, then simply forwards to
       * upper layer before going back to OFF state */
      p_ctx->p_netstk->dllc->recv(p_data, len, p_err);

      /* after-reception process is handled in asynchronous manner */
      p_ctx->event = E_SMARTMAC_EVENT_RX_EXTEND;
      evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_MAC_ULE, &p_ctx->event);
    }
  }
  else if (p_ctx->state == E_SMARTMAC_STATE_RX_PENDING) {
    if (frame.type != SMARTMAC_FRAME_STROBE) {
      /* forwards to upper layer before going back to OFF state */
      p_ctx->p_netstk->dllc->recv(p_data, len, p_err);

      /* after-reception process is handled in asynchronous manner */
      p_ctx->event = E_SMARTMAC_EVENT_RX_EXTEND;
      evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_MAC_ULE, &p_ctx->event);
    }
    else {
      TRACE_LOG_ERR("<SM> RXed multiple strobes");
      mac_rxPending_exit(p_ctx);
      mac_rxPending_entry(p_ctx);
    }
  }
  else {
    /* unexpected event */
    TRACE_LOG_ERR("unexpected event in state=%02x", p_ctx->state);
    /* force MAC to OFF state */
    mac_off_entry(p_ctx);
    *p_err = NETSTK_ERR_FATAL;
  }
}


static void smartmac_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  struct s_smartmac *p_ctx = &smartmac;

  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    case NETSTK_CMD_TX_CBFNCT_SET:
      if (p_val == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      }
      else {
        p_ctx->cbTxFnct = (nsTxCbFnct_t) p_val;
      }
      break;

    case NETSTK_CMD_TX_CBARG_SET:
      p_ctx->p_cbTxArg = p_val;
      break;

    default:
      p_ctx->p_netstk->phy->ioctrl(cmd, p_val, p_err);
      break;
  }
}

static void mac_txStrobe(struct s_smartmac *p_ctx, uint8_t counter, uint16_t dstShortAddr, e_nsErr_t *p_err) {

  uint16_t len;
  uint8_t *p_hdr = &smartmacStrobe[PHY_HEADER_LEN];
  frame_smartmac_st frame;

  /* set frame attributes to zero */
  memset(&frame, 0, sizeof(frame));
  memset(&smartmacStrobe, 0, sizeof(smartmacStrobe));

  /* set frame fields */
  frame.type = SMARTMAC_FRAME_STROBE;
  frame.ack_required = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);
  frame.counter = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
  frame.pan_id = packetbuf_attr(PACKETBUF_ATTR_MAC_PAN_ID);
  frame.src_addr = packetbuf_attr(PACKETBUF_ATTR_MAC_SHORT_ADDR);
  frame.dest_addr = dstShortAddr;

  /* write strobe frame to buffer */
  len = framer_smartmac_create(&frame, p_hdr);

  /* issue PHY to deliver the strobe */
  p_ctx->p_netstk->phy->send(p_hdr, len, p_err);
}

static void mac_txBroadcast(struct s_smartmac *p_ctx, uint8_t *p_data, uint16_t len, e_nsErr_t *p_err) {
  uint8_t counter;
  uint8_t dataSeqNo;
  uint8_t dataAckReq;
  packetbuf_attr_t timeGap;
  packetbuf_attr_t dstShortAddr;

  /* perform Listen-Before-Talk */
  mac_lbt(p_ctx, p_err);
  if (*p_err == NETSTK_ERR_NONE) {
    /* transition to TXBROADCAST state from OFF state*/
    mac_txBroadcast_entry(p_ctx);

    /* transmit broadcast strobes */
    counter = p_ctx->maxBroadcastCounter;
    dstShortAddr = FRAME802154_BROADCASTADDR;
    timeGap = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION);

    /* store sequence number of the actual data packet to send */
    dataSeqNo = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
    dataAckReq = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);

    /* set value of packet buffer ACK required attribute according to transmission
     * of the unicast strobes */
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, FALSE);

    while (counter--) {
      /* set value of packet buffer sequence number attribute accordingly */
      packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, counter);
      mac_txStrobe(p_ctx, counter, dstShortAddr, p_err);

      /* was the strobe successfully transmitted? */
      if (*p_err != NETSTK_ERR_NONE) {
        /* then terminate the transmission process */
        TRACE_LOG_ERR("txStrobe failed -%d", *p_err);
        break;
      }
      else {
        /* otherwise wait for a certain amount of time after transmission */
        bsp_delayUs(timeGap);

        /* is the strobe the last one? */
        if (counter == 0) {
          /* restore data sequence number and acknowledgment requirement fields */
          packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, dataSeqNo);
          packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, dataAckReq);

          /* then transmit the actual data */
          p_ctx->p_netstk->phy->send(p_data, len, p_err);
        }
      }
    }

    /* by all means, values of data frame attributes should be restored */
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, dataSeqNo);
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, dataAckReq);

    /* transition to OFF state */
    mac_txBroadcast_exit(p_ctx);
  }
}

static void mac_txUnicast(struct s_smartmac *p_ctx, uint8_t *p_data, uint16_t len, e_nsErr_t *p_err) {

  uint8_t retx;
  uint8_t maxRetx;
  uint8_t isTxDone;
  uint8_t counter;
  uint8_t dataSeqNo;
  uint8_t dataAckReq;
  linkaddr_t dstAddr;
  packetbuf_attr_t dstShortAddr;

#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
  uint8_t isRxPending = FALSE;
  p_ctx->p_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, &isRxPending, p_err);
  if (isRxPending == TRUE) {
    /* a packet is being received and therefore TX should be terminated */
    *p_err = NETSTK_ERR_BUSY;
    return;
  }
#endif

  /* transition to TXUNICAST state from OFF state*/
  mac_txUnicast_entry(p_ctx);

  /* transmit unicast strobes */
  counter = p_ctx->maxUnicastCounter;
  isTxDone = FALSE;

  /* store sequence number of the actual data packet to send */
  dataSeqNo = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
  dataAckReq = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);
  /* obtain destination short address */
  linkaddr_copy(&dstAddr, packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
  dstShortAddr = (dstAddr.u8[7]     ) |
                 (dstAddr.u8[6] << 8);

#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
  /* estimate TX delay */
  uint32_t estimatedDelay = 0;
  uint32_t actualDelay;
  uint32_t delayOffset;
  uint32_t txReqInterval = bsp_getTick();
  estimatedDelay = mac_calcTxDelay(p_ctx, dstShortAddr);
#endif

  /* perform Listen-Before-Talk */
  mac_lbt(p_ctx, p_err);
  if (*p_err == NETSTK_ERR_NONE) {
    /* set value of packet buffer ACK required attribute according to transmission
     * of the unicast strobes */
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, TRUE);
    while ((isTxDone == FALSE) && (counter--)) {
      /* issue transmission request for unicast strobe */
      packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, counter);
      mac_txStrobe(p_ctx, counter, dstShortAddr, p_err);

      switch (*p_err) {
        case NETSTK_ERR_NONE:
#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
          actualDelay = (bsp_getTick() - txReqInterval) - p_ctx->strobeTxInterval;
          delayOffset = (actualDelay > estimatedDelay) ? (actualDelay - estimatedDelay) : (estimatedDelay - actualDelay);
          if (delayOffset > (p_ctx->sleepTimeout / 2)) {
            delayOffset = p_ctx->sleepTimeout - delayOffset;
          }

          /* update wake-up record table.
           * Only update the table if number of sent strobes are larger than a certain
           * value to ensure that the the wake-up strobe stream actually hit periodic channel scan
           * of the receiver instead of some extended listening following data packet reception */
          if ((p_ctx->maxUnicastCounter - counter) >= SMARTMAC_CFG_MIN_NUM_STROBE_TO_BE_SENT) {
            mac_updateWakeupTable(p_ctx, dstShortAddr);
          }
          else {
            if (delayOffset <= p_ctx->scanTimeout) {
              mac_updateWakeupTable(p_ctx, dstShortAddr);
            }
          }
          TRACE_LOG_MAIN("<TXDELAY> estimated=%d, actual=%d, numStrobeSent=%d", estimatedDelay, actualDelay, p_ctx->maxUnicastCounter - counter);
#endif

          /* the strobe was acknowledged. The MAC then commence transmission of
           * the actual data frame before declaring completion of transmission
           * process */

          /* restore data sequence number and acknowledgment requirement fields */
          packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, dataSeqNo);
          packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, dataAckReq);

          /* issue transmission request */
          maxRetx = 4; //packetbuf_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS);
          retx = 0;
          while (retx++ < maxRetx) {
            p_ctx->p_netstk->phy->send(p_data, len, p_err);
            if (*p_err != NETSTK_ERR_NONE) {
              TRACE_LOG_ERR("<TX> r=%d, a=%d, -%d", retx, dataAckReq, *p_err);
            }

            if (dataAckReq == FALSE) {
              break;
            }
            else {
              if (*p_err != NETSTK_ERR_TX_NOACK) {
                break;
              }
            }
          }
          isTxDone = TRUE;
          break;

        case NETSTK_ERR_TX_NOACK:
          /* the strobe was NOT acknowledged. The MAC shall continue transmission
           * of strobes as long as maximum number of transmission attempts are is
           * not reached */
          isTxDone = FALSE;
          break;

        case NETSTK_ERR_TX_TIMEOUT:
          /* the radio could not transmit the strobe in time. The MAC shall
           * terminate the transmission process */
          isTxDone = TRUE;
          break;

        case NETSTK_ERR_TX_COLLISION:
          /* the strobe was NOT transmitted as the channel was busy. The MAC shall
           * terminate the transmission process */
          isTxDone = TRUE;
          break;

        default:
          TRACE_LOG_ERR("txUnicast failed unexpectedly -%d", *p_err);
          isTxDone = TRUE;
          break;
      }
    }
  }

  /* by all means, values of data frame attributes should be restored */
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, dataSeqNo);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, dataAckReq);

  /* transition to OFF state */
  mac_txUnicast_exit(p_ctx);
}


static void mac_off_entry(struct s_smartmac *p_ctx) {
  e_nsErr_t err = NETSTK_ERR_NONE;

  p_ctx->p_netstk->phy->off(&err);
  if (err == NETSTK_ERR_NONE) {
    p_ctx->state = E_SMARTMAC_STATE_OFF;
  }
  else {
    mac_on_entry(p_ctx);
    mac_scan_entry(p_ctx);
  }
}

static void mac_off_sleepTimeout(struct s_smartmac *p_ctx) {
  mac_on_entry(p_ctx);
  mac_scan_entry(p_ctx);
}

static void mac_off_exit(struct s_smartmac *p_ctx) {

}

static void mac_on_entry(struct s_smartmac *p_ctx) {
  e_nsErr_t err;

  p_ctx->state = E_SMARTMAC_STATE_ON;
  p_ctx->p_netstk->phy->on(&err);
}

static void mac_on_exit(struct s_smartmac *p_ctx) {

}

static void mac_scan_entry(struct s_smartmac *p_ctx) {
  /* start scan timer */
  p_ctx->state = E_SMARTMAC_STATE_SCAN;
  ctimer_restart(&p_ctx->tmr1Scan);
}

static void mac_scan_exit(struct s_smartmac *p_ctx) {
  ctimer_stop(&p_ctx->tmr1Scan);
}


static void mac_rxPending_entry(struct s_smartmac *p_ctx) {
  p_ctx->state = E_SMARTMAC_STATE_RX_PENDING;
  ctimer_restart(&p_ctx->tmr1RxPending);
}

static void mac_rxPending_exit(struct s_smartmac *p_ctx) {
  p_ctx->rxPendingTimeoutCount = 0;
  ctimer_stop(&p_ctx->tmr1RxPending);
}

static void mac_rxDelay_entry(struct s_smartmac *p_ctx) {
  p_ctx->state = E_SMARTMAC_STATE_RX_DELAY;

  /* this timer should update its interval */
  ctimer_set(&p_ctx->tmr1RxDelay, p_ctx->rxDelay, mac_tmrRxDelayCb, p_ctx);
}

static void mac_rxDelay_exit(struct s_smartmac *p_ctx) {
  ctimer_stop(&p_ctx->tmr1RxDelay);
}

#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
static void mac_txDelay_entry(struct s_smartmac *p_ctx) {
  p_ctx->state = E_SMARTMAC_STATE_TX_DELAY;

  e_nsErr_t err;
  p_ctx->p_netstk->phy->off(&err);
}

static void mac_txDelay_exit(struct s_smartmac *p_ctx) {
  e_nsErr_t err;
  p_ctx->p_netstk->phy->on(&err);

  p_ctx->state = E_SMARTMAC_STATE_TX_UNICAST;
}
#endif

static void mac_txBroadcast_entry(struct s_smartmac *p_ctx) {
  p_ctx->state = E_SMARTMAC_STATE_TX_BROADCAST;
}

static void mac_txBroadcast_exit(struct s_smartmac *p_ctx) {

}

static void mac_txUnicast_entry(struct s_smartmac *p_ctx) {
  p_ctx->state = E_SMARTMAC_STATE_TX_UNICAST;
}

static void mac_txUnicast_exit(struct s_smartmac *p_ctx) {

}

static void mac_tmrWakeupCb(void *p_arg)
{
  struct s_smartmac *p_ctx = (struct s_smartmac *)p_arg;

  /* is smartMAC in OFF state? */
  if ((p_ctx->state & 0xF0) == E_SMARTMAC_STATE_OFF) {
    /* then accept sleepTimeout event */
    mac_off_sleepTimeout(p_ctx);
  }
  else {
    /* otherwise ignore the event */
  }
}

static void mac_tmrScanCb(void *p_arg) {
  struct s_smartmac *p_ctx = (struct s_smartmac *)p_arg;
  e_nsErr_t err = NETSTK_ERR_NONE;
  uint8_t isPendingRxFrame = FALSE;

  /* is smartMAC in SCAN state? */
  if (p_ctx->state == E_SMARTMAC_STATE_SCAN) {
    /* check if any frame is being received. If yes, restart the timer */
    p_ctx->p_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, &isPendingRxFrame, &err);
    if (isPendingRxFrame == TRUE) {
      ctimer_restart(&p_ctx->tmr1Scan);
    }
    else {
      p_ctx->event = E_SMARTMAC_EVENT_ASYNC_SCAN_EXIT;
      evproc_putEvent(E_EVPROC_HEAD, EVENT_TYPE_MAC_ULE, &p_ctx->event);
    }
  }
  else {
    /* otherwise ignore the event */
    TRACE_LOG_ERR("<SMARTMAC> unexpected event %02x, expected %02x", p_ctx->state, E_SMARTMAC_STATE_SCAN);
  }
}

static void mac_tmrRxPendingCb(void *p_arg) {
  struct s_smartmac *p_ctx = (struct s_smartmac *)p_arg;
  e_nsErr_t err = NETSTK_ERR_NONE;
  uint8_t isPendingRxFrame = FALSE;

  /* is smartMAC in RXPENDING state? */
  if (p_ctx->state == E_SMARTMAC_STATE_RX_PENDING) {
    /* check if any frame is being received. If yes, restart the timer */
    p_ctx->p_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, &isPendingRxFrame, &err);
    if (isPendingRxFrame == TRUE) {
      p_ctx->rxPendingTimeoutCount++;
      if (p_ctx->rxPendingTimeoutCount > 1) {
        TRACE_LOG_ERR("<RXPENDING> timeout max");
        mac_rxPending_exit(p_ctx);
        mac_off_entry(p_ctx);
      }
      else {
        ctimer_restart(&p_ctx->tmr1RxPending);
      }
    }
    else {
      TRACE_LOG_ERR("<MAC> RxPending timeout %d ms", p_ctx->rxTimeout);
      mac_rxPending_exit(p_ctx);
      mac_off_entry(p_ctx);
    }
  }
  else {
    /* otherwise ignore the event */
    TRACE_LOG_ERR("<SMARTMAC> unexpected event %02x, expected %02x", p_ctx->state, E_SMARTMAC_STATE_RX_PENDING);
  }
}

static void mac_tmrRxDelayCb(void *p_arg) {
  struct s_smartmac *p_ctx = (struct s_smartmac *)p_arg;

  /* is smartMAC in RXDELAY state? */
  if (p_ctx->state == E_SMARTMAC_STATE_RX_DELAY) {
    /* then accept rxDelayimeout event */
    mac_rxDelay_exit(p_ctx);
    mac_on_entry(p_ctx);
    mac_scan_entry(p_ctx);
  }
  else {
    /* otherwise ignore the event */
    TRACE_LOG_ERR("<SMARTMAC> unexpected event %02x, expected %02x", p_ctx->state, E_SMARTMAC_STATE_RX_DELAY);
  }
}

static uint32_t mac_calcRxDelay(struct s_smartmac *p_ctx, uint8_t counter) {
  uint32_t rx_delay = 0;

  if (counter) {
    rx_delay = counter * p_ctx->strobeTxInterval;
    if (rx_delay > p_ctx->rxDelayMin) {
      rx_delay -= p_ctx->rxDelayMin;
    }
    else {
      rx_delay = 0;
    }
  }
  return rx_delay;
}

#if (NETSTK_CFG_LOOSELY_SYNC_EN == TRUE)
static void mac_updateWakeupTable(struct s_smartmac *p_ctx, uint16_t destId)
{
  uint8_t ix;
  for (ix = 0; ix < SMARTMAC_CFG_WAKEUP_TBL_SIZE ; ix++) {
    if (p_ctx->wakeupTable[ix].destId == destId) {
      /* the table is usually updated complete strobe transmission, i.e., the strobe was acknowledged,
       * therefore the strobe transmission time should be omitted */
      p_ctx->wakeupTable[ix].lastWakeupInterval = bsp_getTick() - p_ctx->strobeTxInterval;
      break;
    }
  }
}

static uint32_t mac_calcTxDelay(struct s_smartmac *p_ctx, uint16_t destId)
{
  uint8_t isDone;
  uint32_t ix;
  uint32_t onQty;
  uint32_t onNext;
  uint32_t txDelay;
  uint32_t minDelay;
  uint32_t lastWakeup;
  uint32_t currTime;
  uint32_t estimatedWakeupInterval = 0;

  /* look for records with regard to the destination node */
  isDone = TRUE;
  txDelay = 0;
  lastWakeup = 0;
  for (ix = 0; ix < SMARTMAC_CFG_WAKEUP_TBL_SIZE ; ix++) {
    if (p_ctx->wakeupTable[ix].destId == destId) {
      lastWakeup = p_ctx->wakeupTable[ix].lastWakeupInterval;
      break;
    }
  }

  if (lastWakeup == 0) {
    /* the record has not found then allocate an entry for the destination node */
    for (ix = 0; ix < SMARTMAC_CFG_WAKEUP_TBL_SIZE ; ix++) {
      if (p_ctx->wakeupTable[ix].lastWakeupInterval == 0) {
        p_ctx->wakeupTable[ix].destId = destId;
        break;
      }
    }
  }
  else {
    /* preparation */
    ix = 1;
    txDelay = 0;
    isDone = FALSE;
    currTime = bsp_getTick();

    /* compute number of wake-up intervals since the last recorded interval */
    onQty = (currTime - lastWakeup) / p_ctx->sleepTimeout;

    /* minimum delay that allows immediate sleep before actual transmission attempt */
    minDelay = p_ctx->lbtTimeout + SMARTMAC_CFG_MIN_NUM_STROBE_TO_BE_SENT * p_ctx->strobeTxInterval;

    /* wake-up estimation algorithm */
    do {
      /* prefer to hit the closest estimated wake-up */
      onNext = lastWakeup + (onQty + ix) * p_ctx->sleepTimeout;
      estimatedWakeupInterval = (onNext - currTime);

      if (estimatedWakeupInterval < p_ctx->lbtTimeout) {
        /* time before estimated wake-up not enough to perform LBT then try to hit the following wake-up */
        ix++;
      }
      else {
        /* declare that the process has finished */
        isDone = TRUE;

        /* put radio to sleep when estimated time is sufficient */
        if (estimatedWakeupInterval > minDelay) {
          txDelay = estimatedWakeupInterval - minDelay;
          if (txDelay > 0) {
            mac_txDelay_entry(p_ctx);
            bsp_delay_us(txDelay * 1000);
            mac_txDelay_exit(p_ctx);
          }
        }
        else {
          /* starts LBT immediately */
        }
      }
    } while (isDone == FALSE);
  }
  return estimatedWakeupInterval;
}
#endif /* #if (SMARTMAC_CFG_LOOSELY_SYNC_EN == TRUE) */

static void mac_lbt(struct s_smartmac *p_ctx, e_nsErr_t *p_err) {
  uint32_t tickstart;
  uint8_t isPendingRxFrame;

  /* initialize immediate variables */
  *p_err = NETSTK_ERR_NONE;

  p_ctx->state = E_SMARTMAC_STATE_TX;

  isPendingRxFrame = FALSE;
  tickstart = bsp_getTick();

  /* wait for incoming packet until timeout */
  while ((bsp_getTick() - tickstart) < p_ctx->lbtTimeout) {
    p_ctx->p_netstk->phy->ioctrl(NETSTK_CMD_RF_IS_RX_BUSY, &isPendingRxFrame, p_err);
    if (isPendingRxFrame == TRUE) {
      *p_err = NETSTK_ERR_BUSY;
      break;
    }
  }
}

static void mac_eventHandler(c_event_t c_event, p_data_t p_data)
{
  struct s_smartmac *p_ctx = &smartmac;
  uint8_t data = p_ctx->event;
  uint32_t ucException = FALSE;

  switch (data) {
    case E_SMARTMAC_EVENT_RX_DELAY:
      /* the actual data frame is coming after a significant amount of time,
       * then transition to RXDELAY until the actual data arrives */
      mac_scan_exit(p_ctx);
      mac_on_exit(p_ctx);

      mac_off_entry(p_ctx);
      if (p_ctx->state == E_SMARTMAC_STATE_OFF) {
        mac_rxDelay_entry(p_ctx);
      }
      break;

    case E_SMARTMAC_EVENT_ASYNC_SCAN_EXIT:
      mac_scan_exit(p_ctx);
      mac_off_entry(p_ctx);
      break;

    case E_SMARTMAC_EVENT_RX_EXTEND:
      /* keep radio on for a certain amount of time after reception of good packet */
      switch (p_ctx->state) {
        case E_SMARTMAC_STATE_SCAN:
          mac_scan_exit(p_ctx);
          mac_scan_entry(p_ctx);
          break;

        case E_SMARTMAC_STATE_RX_PENDING:
          mac_rxPending_exit(p_ctx);
          mac_scan_entry(p_ctx);
          break;

        default:
          /* unexpected events */
          ucException = TRUE;
          break;
      }
      break;

    default:
      /* unexpected events */
      ucException = TRUE;
      break;
  }

  if (ucException == TRUE) {
    TRACE_LOG_ERR("<SMARTMAC> unexpected events %d", data);
  }
}
