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
 * @file    cc112x.c
 * @date    12.11.2015
 * @author  PN
 * @brief   Implementation of TI transceiver CC112X
 */


/*
 ********************************************************************************
 *                                   INCLUDES
 ********************************************************************************
 */
#include "emb6.h"
#include "board_conf.h"

#include "cc112x.h"
#include "cc112x_cfg.h"
#include "cc112x_spi.h"

#include "rt_tmr.h"
#include "packetbuf.h"
#include "evproc.h"
#include "phy_framer_802154.h"
#include "llframer.h"

#define  LOGGER_ENABLE        LOGGER_RADIO
#include "logger.h"

/* enable SW auto-acknowledgment feature by default */
#ifndef NETSTK_CFG_RF_SW_AUTOACK_EN
#define NETSTK_CFG_RF_SW_AUTOACK_EN         TRUE
#endif

#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
#include "crc.h"
#endif


/*
 ********************************************************************************
 *                                LOCAL DEFINES
 ********************************************************************************
 */

#ifndef CC112X_PART_NUMBER
#define CC112X_PART_NUMBER                  0x48u
#endif /* #ifndef CC112X_PART_NUMBER */

#ifndef CC112X_PART_VERSION
#define CC112X_PART_VERSION                 0x21u
#endif /* #ifndef CC112X_PART_VERSION */


/*
* cc112x has 128-byte TX FIFO and 128-byte RX-FIFO
* Threshold value is coded in opposite directions for the two FIFOs to give
* equal margin to the overflow and underflow conditions when the threshold
* is reached.
*
* Number of bytes in each FIFO is calculated as following:
*      FIFO_SIZE = 128
*      #Bytes_in_RX_FIFO = FIFO_THR + 1
*      #Bytes_in_TX_FIFO = FIFO_SIZE - #Byte_in_RX_FIFO = FIFO_SIZE - (FIFO_THR + 1)
*
* i.e. (1) FIFO_THR=0 means that there are 127 bytes in the TX FIFO and 1 byte
*          in the RX FIFO.
*
*      (2) FIFO_THR=127 means that there are 0 bytes in the TX FIFO and 128
*          bytes in the RX FIFO
*
*      (3) FIFO_THR=120 means that
*          #Bytes_in_RX_FIFO = FIFO_THR + 1 = 121
*          #Bytes_in_TX_FIFO = FIFO_SIZE - (FIFO_THR + 1) = 128 - (120 + 1) = 7
*          #Available_bytes_in_TX_FIFO = FIFO_THR + 1 = 121
*
*/
#define RF_MAX_FIFO_LEN                     (uint8_t)( 255U )
#define RF_CFG_FIFO_SIZE                    (uint8_t)( 128U )
#define RF_CFG_FIFO_THR                     (LLFRAME_MIN_NUM_RX_BYTES - 1)
#define RF_CFG_NUM_RXBYTES                  (RF_CFG_FIFO_THR + 1)
#define RF_CFG_NUM_TXBYTES                  (RF_CFG_FIFO_SIZE - RF_CFG_NUM_RXBYTES)
#define RF_CFG_NUM_FREE_TXBYTES             (RF_CFG_FIFO_THR + 1)

#define RF_CFG_TX_FIFO_THR                  (uint8_t)(  30U )

/*!< Maximum packet length */
#define RF_MAX_PACKET_LEN                   (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)

/*!< Chip states */
#define RF_CHIP_STATE_IDLE                  (uint8_t)( 0x00 )
#define RF_CHIP_STATE_RX                    (uint8_t)( 0x10 )
#define RF_CHIP_STATE_TX                    (uint8_t)( 0x20 )
#define RF_CHIP_STATE_FSTXON                (uint8_t)( 0x30 )
#define RF_CHIP_STATE_CALIBRATE             (uint8_t)( 0x40 )
#define RF_CHIP_STATE_SETTLING              (uint8_t)( 0x50 )
#define RF_CHIP_STATE_RX_FIFO_ERR           (uint8_t)( 0x60 )
#define RF_CHIP_STATE_TX_FIFO_ERR           (uint8_t)( 0x70 )

/*!< MARC status */
#define RF_MARC_STATUS_NO_FAILURE           (uint8_t)( 0x00 )
#define RF_MARC_STATUS_RX_TIMEOUT           (uint8_t)( 0x01 )
#define RF_MARC_STATUS_RX_TERM              (uint8_t)( 0x02 )
#define RF_MARC_STATUS_TX_OVERFLOW          (uint8_t)( 0x07 )
#define RF_MARC_STATUS_TX_UNDERFLOW         (uint8_t)( 0x08 )
#define RF_MARC_STATUS_RX_OVERFLOW          (uint8_t)( 0x09 )
#define RF_MARC_STATUS_RX_UNDERFLOW         (uint8_t)( 0x0A )
#define RF_MARC_STATUS_TX_ON_CCA_FAILED     (uint8_t)( 0x0B )
#define RF_MARC_STATUS_TX_FINI              (uint8_t)( 0x40 )
#define RF_MARC_STATUS_RX_FINI              (uint8_t)( 0x80 )

/*!< RSSI */
#define RF_RSSI0_CARRIER_SENSE_VALID        (uint8_t)( 0x02 )
#define RF_RSSI0_CARRIER_DETECTED           (uint8_t)( 0x04 )
#define RF_RSSI_VALID_MSK                   (0x01)
#define RF_RSSI_LOW_LEN                     ( 4 )
#define RF_RSSI_LOW_OFST                    ( 3 )
#define RF_RSSI_OFFSET                      ( 102 )
#define RF_RSSI_RES                         ( 0.0625 )

#ifndef MIN
#define MIN(a_, b_)           (((a_) > (b_)) ? (b_) : (a_))
#endif


#define RF_TRIGGER_EVENT()  \
  do {  \
    evproc_putEvent(E_EVPROC_HEAD, NETSTK_RF_EVENT, NULL);  \
  } while(0)

/*!< retrieve chip state from chip status value */
#define RF_GET_CHIP_STATE(_chip_status) \
  (uint8_t)((_chip_status) & 0x70)

#define RF_READ_CHIP_STATE()  \
  (cc112x_spiCmdStrobe(CC112X_SNOP) & 0x70)

/*!< write multiple registers */
#define RF_WR_REGS(regs_) \
  do {  \
    uint8_t len_; \
    len_ = sizeof((regs_)) / sizeof(s_regSettings_t);   \
    rf_configureRegs((regs_), len_);                \
  } while(0)

/*!< configure length of incoming packet
* the PKT_LEN register is set to mod(length, 256).
* See also CC112x User's guide 8.1.5
*/
#define RF_SET_PKT_LEN(len_)  \
  do {  \
    uint8_t wr_byte_; \
    wr_byte_ = (len_) % 256U; \
    cc112x_spiRegWrite(CC112X_PKT_LEN, &(wr_byte_), 1); \
  } while(0)

/*!< configure packet length mode of radio */
#define RF_SET_PKT_LEN_MODE(mode_)  \
  do {  \
    uint8_t wr_byte = mode_;   \
    cc112x_spiRegWrite(CC112X_PKT_CFG0, &wr_byte, 1);  \
  } while(0)

#define RF_SET_FIFO_THR(thr_) \
  do {  \
    uint8_t wr_byte = thr_;   \
    cc112x_spiRegWrite(CC112X_FIFO_CFG, &wr_byte, 1);  \
  } while (0)

/*!< check if RF is in one of reception states */
#define RF_IS_RX_BUSY() \
    ((rf_ctx.state >= RF_STATE_RX_SYNC)   &&  \
     (rf_ctx.state <= RF_STATE_RX_FINI))

/*!< Radio interrupt configuration macros */
#define RF_INT_PKT_BEGIN                E_TARGET_EXT_INT_0  //GPIO0
#define RF_INT_RXFIFO_THR               E_TARGET_EXT_INT_1  //GPIO2
#define RF_INT_PKT_END                  E_TARGET_EXT_INT_2  //GPIO3
#define RF_INT_EDGE_PKT_BEGIN           E_TARGET_INT_EDGE_RISING
#define RF_INT_EDGE_RXFIFO_THR          E_TARGET_INT_EDGE_RISING
#define RF_INT_EDGE_PKT_END             E_TARGET_INT_EDGE_FALLING


#define RF_INT_CONFIG()  \
  do {    \
    bsp_extIntRegister(RF_INT_PKT_BEGIN, RF_INT_EDGE_PKT_BEGIN, rf_pktRxTxBeginISR);      \
    bsp_extIntRegister(RF_INT_RXFIFO_THR, RF_INT_EDGE_RXFIFO_THR, rf_rxFifoThresholdISR); \
    bsp_extIntRegister(RF_INT_PKT_END, RF_INT_EDGE_PKT_END, rf_pktRxTxEndISR);            \
  } while(0)

#define RF_INT_ENABLED() \
  do {    \
      bsp_extIntClear(RF_INT_PKT_BEGIN);    \
      bsp_extIntClear(RF_INT_RXFIFO_THR);   \
      bsp_extIntClear(RF_INT_PKT_END);      \
      bsp_extIntEnable(RF_INT_PKT_BEGIN);   \
      bsp_extIntEnable(RF_INT_RXFIFO_THR);  \
      bsp_extIntEnable(RF_INT_PKT_END);     \
  } while(0)

#define RF_INT_DISABLED() \
  do {    \
      bsp_extIntDisable(RF_INT_PKT_BEGIN);  \
      bsp_extIntDisable(RF_INT_RXFIFO_THR); \
      bsp_extIntDisable(RF_INT_PKT_END);    \
  } while(0)


/*
 ********************************************************************************
 *                                LOCAL TYPEDEFS
 ********************************************************************************
 */
typedef enum e_rf_state {
  RF_STATE_IDLE           = 0x00,

  RF_STATE_RX_IDLE        = 0x10,
  RF_STATE_RX_SYNC        = 0x11,
  RF_STATE_RX_FINI        = 0x12,
  RF_STATE_RX_TXACK_SYNC  = 0x13,
  RF_STATE_RX_TERM        = 0x14,

  RF_STATE_TX             = 0x20,
  RF_STATE_TX_SYNC,
  RF_STATE_TX_FINI,
  RF_STATE_TX_RXACK_SYNC,
  RF_STATE_TX_TERM,

  RF_STATE_FSTXON         = 0x30,
  RF_STATE_CALIBRATE      = 0x40,
  RF_STATE_SETTLING       = 0x50,
  RF_STATE_RX_FIFO_ERR    = 0x60,
  RF_STATE_TX_FIFO_ERR    = 0x70,
  RF_STATE_SLEEP          = 0x80,
} e_rfState_t;


struct s_rf_ctx {
  s_ns_t *p_netstk;
  e_rfState_t state;

  e_nsRfOpMode cfgOpMode;
  uint8_t cfgFreqChanNum;
  uint8_t cfgWOREnabled;
  uint8_t isAckRequired;

  uint16_t rxBytesCounter;
  uint16_t rxNumRemBytes;
  uint32_t rxChksum;
  uint8_t rxIsAddrFiltered;

  /* RX state attributes */
  uint8_t rxReqAck;
  uint8_t rxBuf[RF_MAX_PACKET_LEN];
  uint8_t rxLastDataPtr;
  uint8_t rxLastChksumPtr;

  /* 2-byte appended status
  * - STATUS[0]:  7-0   RSSI
  * - STATUS[1]:  7     CRC_OK
  *               6-0   LQI
  */
  int8_t  rxRSSI;
  uint8_t rxLQI;
  llframe_attr_t rxFrame;

  /* TX state attributes */
  uint8_t *txDataPtr;
  uint16_t txDataLen;
  uint8_t txReqAck;
  uint8_t txStatus;
  e_nsErr_t txErr;
};

#define RF_TX_STATUS_NONE         0x00
#define RF_TX_STATUS_WFA          0x01
#define RF_TX_STATUS_DONE         0x02


/*
********************************************************************************
*                                LOCAL VARIABLES
********************************************************************************
*/
static struct s_rf_ctx rf_ctx;



/*
********************************************************************************
*                           LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void rf_init(void *p_netstk, e_nsErr_t *p_err);
static void rf_on(e_nsErr_t *p_err);
static void rf_off(e_nsErr_t *p_err);
static void rf_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void rf_recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err);
static void rf_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static void rf_pktRxTxBeginISR(void *p_arg);
static void rf_rxFifoThresholdISR(void *p_arg);
static void rf_pktRxTxEndISR(void *p_arg);

static void rf_readRxStatus(struct s_rf_ctx *p_ctx);
static void rf_readRxFifo(struct s_rf_ctx *p_ctx, uint8_t numBytes);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
static void rf_setPktLen(uint8_t mode, uint16_t len);
#endif

static void rf_listen(struct s_rf_ctx *p_ctx);

static void rf_sleep_entry(struct s_rf_ctx *p_ctx);
static void rf_sleep_exit(struct s_rf_ctx *p_ctx);

static void rf_on_entry(struct s_rf_ctx *p_ctx);
static void rf_on_exit(struct s_rf_ctx *p_ctx);

static void rf_rx_entry(struct s_rf_ctx *p_ctx);
static void rf_rx_sync(struct s_rf_ctx *p_ctx);
static void rf_rx_chksum(struct s_rf_ctx *p_ctx);
static void rf_rx_fini(struct s_rf_ctx *p_ctx);
static void rf_rx_term(struct s_rf_ctx *p_ctx);
static void rf_rx_exit(struct s_rf_ctx *p_ctx);

static void rf_tx_entry(struct s_rf_ctx *p_ctx);
static void rf_tx_sync(struct s_rf_ctx *p_ctx);
static void rf_tx_fini(struct s_rf_ctx *p_ctx);
static void rf_tx_rxAck(struct s_rf_ctx *p_ctx);
static void rf_tx_term(struct s_rf_ctx *p_ctx);
static void rf_tx_exit(struct s_rf_ctx *p_ctx);

static void rf_dispatcher(c_event_t c_event, p_data_t p_data);

static void rf_configureRegs(const s_regSettings_t *p_regs, uint8_t len);
static void rf_manualCalibration(void);
static void rf_calibrateRCOsc(void);
static void rf_cca(e_nsErr_t *p_err);

#if (NETSTK_CFG_RF_RETX_EN == TRUE)
static void cc112x_retx(e_nsErr_t *p_err);
#endif

static void rf_reset(void);
static void rf_chkPartnumber(e_nsErr_t *p_err);
static void rf_waitRdy(void);

static void rf_txPowerSet(int8_t power, e_nsErr_t *p_err);
static void rf_txPowerGet(int8_t *p_power, e_nsErr_t *p_err);
static void rf_chanNumSet(uint8_t chan_num, e_nsErr_t *p_err);
static void rf_opModeSet(e_nsRfOpMode mode, e_nsErr_t *p_err);
static void rf_readRSSI(int8_t *p_val, e_nsErr_t *p_err);


/*
********************************************************************************
*                           LOCAL FUNCTIONS DEFINITIONS
********************************************************************************
*/

/**
 * @brief initialize radio driver
 * @param p_netstk  point to variable holding netstack structure
 * @param p_err     point to variable storing return error code
 */
static void rf_init(void *p_netstk, e_nsErr_t *p_err)
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

  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* set radio context attributes to zero */
  memset(p_ctx, 0, sizeof(*p_ctx));

  /* store pointer to global netstack structure */
  p_ctx->p_netstk = (s_ns_t *)p_netstk;

  /* initialize SPI handle */
  cc112x_spiInit();

  /* reset the transceiver. Afterwards the chip will be in IDLE state */
  rf_reset();

  /* check part number */
  rf_chkPartnumber(p_err);
  if (*p_err != NETSTK_ERR_NONE) {
    emb6_errorHandler(p_err);
  }

  /* configure RF registers */
  RF_WR_REGS(cc112x_cfg_ieee802154g_default);
  RF_SET_FIFO_THR(RF_CFG_FIFO_THR);

  /* calibrate radio according to cc112x errata */
  rf_manualCalibration();

  /* calibrate RC oscillator */
  rf_calibrateRCOsc();

  /* configurations of radio interrupts */
  RF_INT_CONFIG();

  /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* then use infinite packet length mode by default */
  rf_setPktLen(CC112X_PKT_LEN_MODE_INFINITE, RF_MAX_FIFO_LEN);
#endif

  /* initialize local variables */
  evproc_regCallback(NETSTK_RF_EVENT, rf_dispatcher);

  /* configure operating mode and channel number */
  p_ctx->cfgFreqChanNum = 0;
  p_ctx->cfgOpMode = NETSTK_RF_OP_MODE_CSM;

  /* debug watchdog timer */
#if (NETSTK_CFG_RF_DEBUG_EN == TRUE)
  rt_tmr_create(&p_ctx->dbgTmr, E_RT_TMR_TYPE_PERIODIC, 10000, rf_dbgTmrCb, NULL);
#endif

  /* transition to SLEEP state */
  rf_sleep_entry(p_ctx);
}


/**
 * @brief turn radio driver on
 * @param p_err point to variable storing return error code
 */
static void rf_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* is the driver sleeping? */
  if ((p_ctx->state & 0xF0) == RF_STATE_SLEEP) {
    /* then exit SLEEP state */
    rf_sleep_exit(p_ctx);

    /* and enters ON state */
    rf_on_entry(p_ctx);
  }

  /* indicate successful operation */
  *p_err = NETSTK_ERR_NONE;
}

/**
 * @brief turn radio driver off
 * @param p_err point to variable storing return error code
 */
static void rf_off(e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* is the driver on? */
  if ((p_ctx->state & 0xF0) != RF_STATE_SLEEP) {
    /* then exits ON state */
    rf_on_exit(p_ctx);

    /* then turn it off */
    rf_sleep_entry(p_ctx);
  }

  /* set return error code */
  *p_err = NETSTK_ERR_NONE;
}

/**
 * @brief handle transmission request from upper layer
 * @param p_data  point to buffer holding frame to send
 * @param len     length of frame to send
 * @param p_err   point to variable storing return error code
 */
static void rf_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
  if ((p_data == NULL) || (len == 0) || (len > RF_MAX_PACKET_LEN)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  uint32_t tickstart;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  if (p_ctx->state != RF_STATE_RX_IDLE) {
    /* the radio is currently busy then terminate transmission request */
    *p_err = NETSTK_ERR_BUSY;
    return;
  }
  /* otherwise start transmission process */

  /* write frame to send into TX FIFO */
  uint16_t txNumRxBytes;
  uint8_t *txDataPtr;

  txNumRxBytes = len;
  txDataPtr = p_data;

  while (txNumRxBytes > 0) {
    if (txNumRxBytes > RF_CFG_TX_FIFO_THR) {
      cc112x_spiTxFifoWrite(txDataPtr, RF_CFG_TX_FIFO_THR);
      txNumRxBytes -= RF_CFG_TX_FIFO_THR;
      txDataPtr += RF_CFG_TX_FIFO_THR;
    }
    else {
      cc112x_spiTxFifoWrite(txDataPtr, txNumRxBytes);
      txNumRxBytes = 0;
      p_data += txNumRxBytes;
    }
  }

  /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* then store length of packet to send */
  p_ctx->txDataLen = len;
#endif /* NETSTK_CFG_IEEE_802154G_EN */

  /* issue the radio transmission command */
  cc112x_spiCmdStrobe(CC112X_STX);

  /* wait for complete transmission of the frame until timeout */
  tickstart = rt_tmr_getCurrenTick();
  while ((rt_tmr_getCurrenTick() - tickstart) < 50) { //50ms
    if ((p_ctx->txStatus == RF_TX_STATUS_DONE) ||
        (p_ctx->txStatus == RF_TX_STATUS_WFA)) {
      /* frame was transmitted successfully */
      break;
    }
  }
  if (p_ctx->txStatus == RF_TX_STATUS_DONE) {
    *p_err = p_ctx->txErr;

    if (p_ctx->txErr == NETSTK_ERR_TX_COLLISION) {
      TRACE_LOG_ERR("+++ RF: send err=-%d", p_ctx->txErr);
      /* FIXME exception handler */
      rf_tx_exit(p_ctx);
    }

    if (p_ctx->state != RF_STATE_RX_SYNC) {
      /* TXONCCA_FAILED */
      /* exit TX state */
      rf_tx_exit(p_ctx);
      /* enter RX state */
      rf_rx_entry(p_ctx);
    }
  }
#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
  else if (p_ctx->txStatus == RF_TX_STATUS_WFA) {
    /* wait for ACK for maximum macAckWaitDuration (2.6ms) */
    packetbuf_attr_t waitForAckTimeoutInTicks;
    waitForAckTimeoutInTicks = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION) / RT_TMR_CFG_TICK_FREQ_IN_HZ;

    /* make sure a time period of at least [macAckWaitDuration + 1] ms has passed */
    waitForAckTimeoutInTicks += 2;

    *p_err = NETSTK_ERR_TX_NOACK;
    tickstart = rt_tmr_getCurrenTick();
    while ((rt_tmr_getCurrenTick() - tickstart) < waitForAckTimeoutInTicks) {
      if (p_ctx->txStatus == RF_TX_STATUS_DONE) {
        /* either the frame was acknowledged or error occurred */
        *p_err = p_ctx->txErr;
        break;
      }
    }

    /* exit TX state */
    rf_tx_exit(p_ctx);
    /* enter RX state */
    rf_rx_entry(p_ctx);
  }
#endif /* NETSTK_CFG_RF_CC112X_AUTOACK_EN */
  else {
    *p_err = NETSTK_ERR_TX_TIMEOUT;

    /* exit TX state */
    rf_tx_exit(p_ctx);
    /* enter RX state */
    rf_rx_entry(p_ctx);
  }
}


/**
 * @brief handle incoming frame reception
 * @param p_buf point to buffer holding the received frame
 * @param len   length of the frame
 * @param p_err point to variable storing return error code
 */
static void rf_recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif
}


/**
 * @brief handle miscellaneous commands
 * @param cmd   command identifier
 * @param p_val opaque pointer referenced to variable holding/storing attribute value
 * @param p_err point to variable storing return error code
 */
static void rf_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    case NETSTK_CMD_RF_TXPOWER_SET:
      rf_txPowerSet(*((int8_t *) p_val), p_err);
      break;

    case NETSTK_CMD_RF_TXPOWER_GET:
      rf_txPowerGet(p_val, p_err);
      break;

    case NETSTK_CMD_RF_CCA_GET:
      rf_cca(p_err);
      break;

    case NETSTK_CMD_RF_RETX:
#if (NETSTK_CFG_RF_RETX_EN == TRUE)
      /* retransmit the last frame
      * - write TxFirst to the previous value
      */
      cc112x_retx(p_err);
#else
      *p_err = NETSTK_ERR_CMD_UNSUPPORTED;
#endif
      break;


    case NETSTK_CMD_RF_IS_RX_BUSY:
      if (RF_IS_RX_BUSY() == TRUE) {
        *p_err = NETSTK_ERR_BUSY;
      }
      break;

    case NETSTK_CMD_RF_CHAN_NUM_SET:
      rf_chanNumSet(*((uint8_t *) p_val), p_err);
      break;

    case NETSTK_CMD_RF_OP_MODE_SET:
      rf_opModeSet(*((e_nsRfOpMode *) p_val), p_err);
      break;

    case NETSTK_CMD_RF_WOR_EN:
      if (p_val) {
        rf_ctx.cfgWOREnabled = *((uint8_t *) p_val);
      } else {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      }
      break;

    case NETSTK_CMD_RX_BUF_READ:
      /*
       * Signal upper layer if a packet has arrived by the time this
       * command is issued.
       * Trigger event-process manually
       */
      rf_dispatcher(NETSTK_RF_EVENT, NULL);
      break;

    case NETSTK_CMD_RF_RSSI_GET:
      rf_readRSSI(p_val, p_err);
      break;

    case NETSTK_CMD_RF_RF_SWITCH_SET:
    case NETSTK_CMD_RF_ANT_DIV_SET:
    case NETSTK_CMD_RF_SENS_SET:
    case NETSTK_CMD_RF_SENS_GET:
    default:
      /* unsupported commands are treated in same way */
      *p_err = NETSTK_ERR_CMD_UNSUPPORTED;
      break;
  }
}


/*
********************************************************************************
*                       INTERRUPT SUBROUTINE HANDLERS
********************************************************************************
*/

/**
 * @brief interrupt subroutine to handle beginning of packet reception/transmission
 * @param p_arg point to variable holding callback argument
 */
static void rf_pktRxTxBeginISR(void *p_arg) {
  (void) &p_arg;

  uint8_t marc_status;
  uint8_t chip_state;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* entry */
  cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);
  chip_state = RF_READ_CHIP_STATE();
  TRACE_LOG_MAIN("B m=%02x, cs=%02x, ds=%02x", marc_status, chip_state, p_ctx->state);

  /* FIXME unexpected marc */
  if ((marc_status == RF_MARC_STATUS_RX_FINI) ||
      (marc_status == RF_MARC_STATUS_TX_FINI)) {
    marc_status = RF_MARC_STATUS_NO_FAILURE;
  }


  /* do */
  switch (marc_status) {
    case RF_MARC_STATUS_NO_FAILURE:
      if (p_ctx->state == RF_STATE_RX_FINI) {
        if (p_ctx->rxReqAck == TRUE) {
          /* SYNC words of ACK frame were transmitted */
          p_ctx->state = RF_STATE_RX_TXACK_SYNC;

          /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
          /* then switch to fixed packet length mode */
          rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->txDataLen);
#endif
        }
        else {
          /* unexpected event */
          TRACE_LOG_ERR("+++ RF: PKT_BEGIN unexpected interrupt state=%02x, cs=%02x", p_ctx->state, chip_state);
        }
      }
      else if (p_ctx->state == RF_STATE_RX_IDLE) {
        if (chip_state == RF_STATE_TX) {
          /* the radio just finished transmitting SYNC words */
          rf_tx_sync(p_ctx);
        }
        else {
          /* the radio just finished receiving SYNC words */
          rf_rx_sync(p_ctx);
        }
      }
      else if (p_ctx->state == RF_STATE_TX_FINI) {
        if (p_ctx->txReqAck == TRUE) {
          /* the incoming frame can be the ACK */
          p_ctx->state = RF_STATE_TX_RXACK_SYNC;
        }
        else {
          /* unexpected event */
          TRACE_LOG_ERR("+++ RF: PKT_BEGIN unexpected marc=%02x, state=%02x, cs=%02x", marc_status, p_ctx->state, chip_state);
        }
      }
      else {
        /* unexpected event */
        TRACE_LOG_ERR("+++ RF: PKT_BEGIN unexpected marc=%02x, state=%02x, cs=%02x", marc_status, p_ctx->state, chip_state);

        /* FIXME exception handler */
        if ((p_ctx->state == RF_STATE_RX_TXACK_SYNC) &&
            (chip_state == RF_STATE_RX_IDLE)) {
          /* the radio just finished receiving SYNC words */
          rf_rx_sync(p_ctx);
        }
      }
      break;

    case RF_MARC_STATUS_TX_ON_CCA_FAILED:
      //TRACE_LOG_ERR("+++ RF: PKT_BEGIN TxOnCCA failed state=%02x, cs=%02x", p_ctx->state, chip_state);
      p_ctx->txErr = NETSTK_ERR_TX_COLLISION;
      rf_tx_term(p_ctx);

      /* the radio just finished receiving SYNC words */
      rf_rx_sync(p_ctx);
      break;

    case RF_MARC_STATUS_TX_OVERFLOW:
    case RF_MARC_STATUS_TX_UNDERFLOW:
      TRACE_LOG_ERR("+++ RF: PKT_BEGIN TXFIFO_ERR state=%02x, cs=%02x", p_ctx->state, chip_state);
      rf_tx_term(p_ctx);
      rf_rx_exit(p_ctx);
      rf_rx_entry(p_ctx);
      break;

    case RF_MARC_STATUS_RX_OVERFLOW:
    case RF_MARC_STATUS_RX_UNDERFLOW:
    case RF_MARC_STATUS_RX_TIMEOUT:
    case RF_MARC_STATUS_RX_TERM:
      TRACE_LOG_ERR("+++ RF: PKT_BEGIN RX_ERR state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);
      rf_rx_term(p_ctx);
      break;

    default:
      TRACE_LOG_ERR("+++ RF: PKT_BEGIN unexpected marc=%02x, state=%02x, cs=%02x", marc_status, p_ctx->state, chip_state);
      break;
  }

  /* exit */
  bsp_extIntClear(RF_INT_PKT_BEGIN);
  TRACE_LOG_MAIN("+++ RF: PKT_BEGIN_EXIT  state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);
}

/**
 * @brief interrupt subroutine to handle RXFIFO threshold
 * @param p_arg point to variable holding callback argument
 */
static void rf_rxFifoThresholdISR(void *p_arg) {
  (void) &p_arg;

  uint8_t numRxBytes;
  uint8_t numChksumBytes;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* entry */
  cc112x_spiRegRead(CC112X_NUM_RXBYTES, &numRxBytes, 1);

  /* do */
  if (numRxBytes > RF_CFG_FIFO_THR) {
    /* read bytes from RX FIFO */
    rf_readRxFifo(p_ctx, numRxBytes);

    if (p_ctx->rxLastDataPtr == LLFRAME_MIN_NUM_RX_BYTES) {
      /* parse */
      p_ctx->rxNumRemBytes = llframe_parse(&p_ctx->rxFrame, p_ctx->rxBuf, p_ctx->rxLastDataPtr);
      if (p_ctx->rxNumRemBytes == 0) {
        /* invalid frame */
        TRACE_LOG_MAIN("dropped frame due to bad format");
        /* then terminate RX process */
        rf_rx_term(p_ctx);
      }
      p_ctx->rxBytesCounter = p_ctx->rxFrame.tot_len;

      /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
      /* then switch to fixed packet length mode */
      rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->rxBytesCounter);
#endif

#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
      /* does incoming frame require ACK? */
      if (p_ctx->rxFrame.is_ack_required == TRUE) {
        /* indicate incoming frame requiring ACK */
        p_ctx->rxReqAck = TRUE;

        /* write the ACK to send into TX FIFO */
        uint8_t ack[10];
        uint8_t ack_len;
        ack_len = llframe_createAck(&p_ctx->rxFrame, ack, sizeof(ack));
        cc112x_spiTxFifoWrite(ack, ack_len);

        /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
        /* then store length of ACK to send for later setting packet length modes */
        p_ctx->txDataLen = ack_len;
#endif /* NETSTK_CFG_IEEE_802154G_EN */
      }
#endif /* NETSTK_CFG_RF_SW_AUTOACK_EN */

      /* initialize checksum */
      p_ctx->rxChksum = llframe_crcInit(&p_ctx->rxFrame);
      p_ctx->rxLastChksumPtr = p_ctx->rxFrame.crc_offset;
      numChksumBytes = p_ctx->rxLastDataPtr - p_ctx->rxLastChksumPtr;
    }
    else {
      if (p_ctx->rxNumRemBytes >= numRxBytes) {
        p_ctx->rxNumRemBytes -= numRxBytes;

#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
        /* compute length of checksum data */
        if (p_ctx->rxNumRemBytes < p_ctx->rxFrame.crc_len) {
          numChksumBytes = numRxBytes - (p_ctx->rxFrame.crc_len - p_ctx->rxNumRemBytes);
        } else {
          numChksumBytes = numRxBytes;
        }

        /* is address not yet checked and number of received bytes sufficient for
        * address filtering? */
        if ((p_ctx->rxIsAddrFiltered == FALSE) &&
            (p_ctx->rxLastDataPtr > (LLFRAME_MIN_NUM_RX_BYTES + p_ctx->rxFrame.min_addr_len))) {
          /* then perform address filtering */
          p_ctx->rxIsAddrFiltered = llframe_addrFilter(&p_ctx->rxFrame, p_ctx->rxBuf, p_ctx->rxLastDataPtr);

          /* is address invalid? */
          if (p_ctx->rxIsAddrFiltered == FALSE) {
            TRACE_LOG_MAIN("dropped frame due to invalid address");

            /* then terminate RX process */
            rf_rx_term(p_ctx);
          }
        }
#endif /* NETSTK_CFG_RF_SW_AUTOACK_EN */
      }
      else {
        /* ignore unexpected ISRs */
        TRACE_LOG_ERR("unexpected ISR rxNumRemBytes=%d", p_ctx->rxNumRemBytes);
      }
    }

#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
    /* update checksum */
    p_ctx->rxChksum = crc_16_updateN(p_ctx->rxChksum, &p_ctx->rxBuf[p_ctx->rxLastChksumPtr], numChksumBytes);
    p_ctx->rxLastChksumPtr += numChksumBytes;
#endif /* NETSTK_CFG_RF_CC112X_AUTOACK_EN */
  }

  /* exit */
  bsp_extIntClear(RF_INT_RXFIFO_THR);
}


/**
 * @brief interrupt subroutine to handle ending of packet reception/transmission
 * @param p_arg point to variable holding callback argument
 */
static void rf_pktRxTxEndISR(void *p_arg) {
  (void) &p_arg;

  uint8_t marc_status;
  uint8_t chip_state;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* entry */
  cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);
  chip_state = RF_READ_CHIP_STATE();
  TRACE_LOG_MAIN("E m=%02x, cs=%02x, ds=%02x", marc_status, chip_state, p_ctx->state);

  switch (marc_status) {
    case RF_MARC_STATUS_RX_FINI:
      if (p_ctx->state == RF_STATE_RX_SYNC) {
        /* checksum verification */
        rf_rx_chksum(p_ctx);
      }
#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
      else if (p_ctx->state == RF_STATE_TX_RXACK_SYNC) {
        /* ACK verification */
        rf_tx_rxAck(p_ctx);
      }
#endif /* NETSTK_CFG_RF_SW_AUTOACK_EN */
      else {
        TRACE_LOG_ERR("+++ RF: PKT_END unexpected event state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);
      }

      /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
      /* then switch back to infinite packet length mode */
      rf_setPktLen(CC112X_PKT_LEN_MODE_INFINITE, RF_MAX_FIFO_LEN);
#endif
      break;

    case RF_MARC_STATUS_RX_OVERFLOW:
    case RF_MARC_STATUS_RX_UNDERFLOW:
    case RF_MARC_STATUS_RX_TIMEOUT:
    case RF_MARC_STATUS_RX_TERM:
      TRACE_LOG_ERR("+++ RF: PKT_END   RX_ERR state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);
      rf_rx_term(p_ctx);
      break;

    case RF_MARC_STATUS_NO_FAILURE:
      if (p_ctx->state == RF_STATE_RX_TERM) {
        /* RX termination */
      }
      else {
        if (p_ctx->state == RF_STATE_RX_SYNC) {
          rf_rx_term(p_ctx);
        }
        else {
          TRACE_LOG_ERR("+++ RF: PKT_END   unexpected event state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);

          if (p_ctx->state == RF_STATE_RX_TXACK_SYNC) {
            /* a responding ACK was successfully transmitted */
            rf_rx_exit(p_ctx);
            rf_rx_entry(p_ctx);
          }
        }
      }
      break;

    case RF_MARC_STATUS_TX_FINI:
      if (p_ctx->state == RF_STATE_TX_SYNC) {
        /* a frame was successfully transmitted */
        rf_tx_fini(p_ctx);
      }
      else if (p_ctx->state == RF_STATE_RX_TXACK_SYNC) {
        /* a responding ACK was successfully transmitted */
        rf_rx_exit(p_ctx);
        rf_rx_entry(p_ctx);
      }
      else {
        TRACE_LOG_ERR("+++ RF: PKT_END unexpected event state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);
        rf_rx_term(p_ctx);
      }
      break;

    case RF_MARC_STATUS_TX_OVERFLOW:
    case RF_MARC_STATUS_TX_UNDERFLOW:
      TRACE_LOG_ERR("+++ RF: PKT_END TXFIFO_ERR state=%02x, cs=%02x", p_ctx->state, chip_state);
      rf_tx_term(p_ctx);
      rf_rx_exit(p_ctx);
      rf_rx_entry(p_ctx);
      break;

    default:
      TRACE_LOG_ERR("+++ RF: PKT_END unexpected state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);
      rf_rx_term(p_ctx);
      break;
  }

  /* exit */
  bsp_extIntClear(RF_INT_PKT_END);
  TRACE_LOG_MAIN("+++ RF: PKT_END_EXIT    state=%02x, cs=%02x, marc=%02x", p_ctx->state, chip_state, marc_status);
}


/*
********************************************************************************
*                           STATE-EVENT HANDLING
********************************************************************************
*/

/**
 * @brief handle entry function of SLEEP state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_sleep_entry(struct s_rf_ctx *p_ctx) {
  /* finally put the radio to POWERDOWN state */
  p_ctx->state = RF_STATE_SLEEP;
  cc112x_spiCmdStrobe(CC112X_SPWD);
}


/**
 * @brief handle exit function of SLEEP state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_sleep_exit(struct s_rf_ctx *p_ctx) {
  p_ctx->state = RF_STATE_IDLE;
  cc112x_spiCmdStrobe(CC112X_SIDLE);
  while (RF_READ_CHIP_STATE() != RF_STATE_IDLE) {
    /* wait until the chip goes to IDLE state */
  }
}


/**
 * @brief handle entry function of ON state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_on_entry(struct s_rf_ctx *p_ctx) {
  /* enable radio interrupts handling */
  RF_INT_ENABLED();

  /* transition to RX_IDLE state */
  rf_rx_entry(p_ctx);
}


/**
 * @brief handle exit function of ON state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_on_exit(struct s_rf_ctx *p_ctx) {
  /* disable radio interrupts handling */
  RF_INT_DISABLED();

  /* put the radio to IDLE state and flush FIFOs */
  cc112x_spiCmdStrobe(CC112X_SIDLE);
  cc112x_spiCmdStrobe(CC112X_SFRX);
  cc112x_spiCmdStrobe(CC112X_SFTX);
}


/**
 * @brief handle entry function of RX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_rx_entry(struct s_rf_ctx *p_ctx) {
  TRACE_LOG_MAIN("+++ RF: RX_ENTRY");
  /* initialize RX attributes */
  p_ctx->rxReqAck = FALSE;
  p_ctx->rxNumRemBytes = 0;
  p_ctx->rxLastDataPtr = 0;
  p_ctx->rxLastChksumPtr = 0;
  p_ctx->rxIsAddrFiltered = FALSE;
  memset(&p_ctx->rxFrame, 0, sizeof(p_ctx->rxFrame));

  /* state transition */
  p_ctx->state = RF_STATE_RX_IDLE;
  rf_listen(p_ctx);
}


/**
 * @brief handle event of SYNC words received in RX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_rx_sync(struct s_rf_ctx *p_ctx) {
  p_ctx->state = RF_STATE_RX_SYNC;
  LED_RX_ON();
}


/**
 * @brief handle event
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_rx_chksum(struct s_rf_ctx *p_ctx) {
  uint8_t isChecksumOK;
  uint8_t numChksumBytes;

  /* read remaining bytes from RX FIFO */
  rf_readRxFifo(p_ctx, p_ctx->rxNumRemBytes);

#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
  /* update checksum */
  if (p_ctx->rxLastDataPtr > (p_ctx->rxFrame.crc_len + p_ctx->rxLastChksumPtr)) {
    numChksumBytes = p_ctx->rxLastDataPtr - (p_ctx->rxLastChksumPtr + p_ctx->rxFrame.crc_len);
    p_ctx->rxChksum = crc_16_updateN(p_ctx->rxChksum, &p_ctx->rxBuf[p_ctx->rxLastChksumPtr], numChksumBytes);
    p_ctx->rxLastChksumPtr += numChksumBytes;
  }

  isChecksumOK = llframe_crcFilter(&p_ctx->rxFrame,
                                    p_ctx->rxChksum,
                                   &p_ctx->rxBuf[p_ctx->rxLastChksumPtr],
                                    p_ctx->rxFrame.crc_len);
#else /* NETSTK_CFG_RF_SW_AUTOACK_EN */
  /* checksum verification is handled by upper layer */
  isChecksumOK = TRUE;
#endif /* NETSTK_CFG_RF_SW_AUTOACK_EN */

  /* is checksum valid? */
  if (isChecksumOK == TRUE) {
#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
    if (p_ctx->rxReqAck == TRUE) {
      /* commence ACK transmission */
      cc112x_spiCmdStrobe(CC112X_STX);
    }
#endif /* NETSTK_CFG_RF_CC112X_AUTOACK_EN */

    /* read status bytes */
    rf_readRxStatus(p_ctx);

    /* signal upper layer */
    p_ctx->state = RF_STATE_RX_FINI;
    RF_TRIGGER_EVENT();
  }
  else {
    /* discard the received frame */
    TRACE_LOG_ERR("+++ RF: RX_CHKSUM invalid checksum");
    rf_rx_term(p_ctx);
  }
}


/**
 * @brief handle event of a good packet being completely received in RX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_rx_fini(struct s_rf_ctx *p_ctx) {
  /* if ACK is not required by the received frame, then exit and re-enter RX state */
  if (p_ctx->rxReqAck == FALSE) {
    rf_rx_exit(p_ctx);
    rf_rx_entry(p_ctx);
  }

  /* set the error code to default */
  e_nsErr_t err = NETSTK_ERR_NONE;
  TRACE_LOG_MAIN("+++ RF: RX_FINI seq=%02x, %d bytes", p_ctx->rxBuf[PHY_HEADER_LEN + 2], p_ctx->rxBytesCounter);
  //trace_printHex("+++ RF: RX_FINI", p_ctx->rxBuf, p_ctx->rxBytesCounter);

  /* then signal upper layer */
  p_ctx->p_netstk->phy->recv(p_ctx->rxBuf, p_ctx->rxBytesCounter, &err);
  if ((err != NETSTK_ERR_NONE) &&
      (err != NETSTK_ERR_INVALID_ADDRESS)) {
    /* the frame is discarded by upper layers */
    TRACE_LOG_ERR("+++ RF: RX_FINI discarded e=-%d", err);
    trace_printHex("", p_ctx->rxBuf, p_ctx->rxBytesCounter);
  }
}


/**
 * @brief handle event of reception termination in RX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_rx_term(struct s_rf_ctx *p_ctx) {
  TRACE_LOG_MAIN("+++ RF: RX_TERM");

  /* set sub-state */
  p_ctx->state = RF_STATE_RX_TERM;

  /* handle reception termination events in RF critical section
  * - put the radio to IDLE state
  * - flush RX FIFO
  * - flush TX FIFO
  */
  RF_INT_DISABLED();
  cc112x_spiCmdStrobe(CC112X_SIDLE);
  cc112x_spiCmdStrobe(CC112X_SFRX);
  cc112x_spiCmdStrobe(CC112X_SFTX);
  RF_INT_ENABLED();

  /* exit RX state */
  rf_rx_exit(p_ctx);

  /* enter RX state */
  rf_rx_entry(p_ctx);
}


/**
 * @brief handle exit function of RX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_rx_exit(struct s_rf_ctx *p_ctx) {
  TRACE_LOG_MAIN("+++ RF: RX_EXIT");
  LED_RX_OFF();

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* use infinite packet length mode by default */
  RF_SET_PKT_LEN_MODE(CC112X_PKT_LEN_MODE_INFINITE);
#endif /* NETSTK_CFG_IEEE_802154G_EN */
}


/**
 * @brief handle entry function in TX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_tx_entry(struct s_rf_ctx *p_ctx) {
  LED_TX_ON();
  p_ctx->txErr = NETSTK_ERR_NONE;
  p_ctx->txStatus = RF_TX_STATUS_NONE;
  p_ctx->txReqAck = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);
  TRACE_LOG_MAIN("+++ RF: TX_ENTRY");
}


/**
 * @brief handle event of SYNC words transmitted in TX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_tx_sync(struct s_rf_ctx *p_ctx) {
  p_ctx->state = RF_STATE_TX_SYNC;
  rf_tx_entry(p_ctx);

  /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* then switch to fixed packet length mode */
  rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->txDataLen);
#endif
}


/**
 * @brief handle event of a frame successfully transmitted in TX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_tx_fini(struct s_rf_ctx *p_ctx) {
  p_ctx->state = RF_STATE_TX_FINI;
  p_ctx->txErr = NETSTK_ERR_NONE;

  if (p_ctx->txReqAck == FALSE) {
    /* indicate the TX process has finished */
    p_ctx->txStatus = RF_TX_STATUS_DONE;
  }
#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
  else {
    /* indicate the radio is waiting for ACK as response to the transmitted
    * frame */
    p_ctx->txStatus = RF_TX_STATUS_WFA;

    /* put the radio into idle listening */
    rf_listen(p_ctx);
  }
#endif /* NETSTK_CFG_RF_CC112X_AUTOACK_EN */

  /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* then switch back to infinite packet length mode */
  rf_setPktLen(CC112X_PKT_LEN_MODE_INFINITE, RF_MAX_FIFO_LEN);
#endif

  TRACE_LOG_MAIN("+++ RF: TX_FINI txStatus=%02x", p_ctx->txStatus);
}


#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
/**
 * @brief handle event of ACK reception in TX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_tx_rxAck(struct s_rf_ctx *p_ctx) {
  uint8_t isChecksumOK;
  uint8_t numChksumBytes;
  packetbuf_attr_t expSeqNo;

  /* read remaining bytes from RX FIFO */
  rf_readRxFifo(p_ctx, p_ctx->rxNumRemBytes);

  /* update checksum */
  if (p_ctx->rxLastDataPtr > (p_ctx->rxFrame.crc_len + p_ctx->rxLastChksumPtr)) {
    numChksumBytes = p_ctx->rxLastDataPtr - (p_ctx->rxLastChksumPtr + p_ctx->rxFrame.crc_len);
    p_ctx->rxChksum = crc_16_updateN(p_ctx->rxChksum, &p_ctx->rxBuf[p_ctx->rxLastChksumPtr], numChksumBytes);
    p_ctx->rxLastChksumPtr += numChksumBytes;
  }

  isChecksumOK = llframe_crcFilter(&p_ctx->rxFrame,
                                    p_ctx->rxChksum,
                                   &p_ctx->rxBuf[p_ctx->rxLastChksumPtr],
                                    p_ctx->rxFrame.crc_len);

  /* is checksum valid? */
  if (isChecksumOK == TRUE) {
    /* read status bytes */
    rf_readRxStatus(p_ctx);

    /* FIXME remove "spaghetti code" */
    expSeqNo = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
    if ((p_ctx->rxBuf[PHY_HEADER_LEN] == 0x02) &&
        (p_ctx->rxBuf[PHY_HEADER_LEN + 2] == expSeqNo)) {
      p_ctx->txErr = NETSTK_ERR_NONE;
    }
    else {
      p_ctx->txErr = NETSTK_ERR_TX_COLLISION;
    }
  }
  else {
    /* discard the received frame */
    p_ctx->txErr = NETSTK_ERR_TX_COLLISION;
  }

  /* indicate TX process has finished */
  p_ctx->txStatus = RF_TX_STATUS_DONE;
  TRACE_LOG_MAIN("+++ RF: RX_ACK seq=%02x, err=-%d", p_ctx->rxBuf[PHY_HEADER_LEN + 2], p_ctx->txErr);
}
#endif


/**
 * @brief handle event of transmission termination in TX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_tx_term(struct s_rf_ctx *p_ctx) {
  p_ctx->txStatus = RF_TX_STATUS_DONE;
  cc112x_spiCmdStrobe(CC112X_SFTX);

  /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* then switch back to infinite packet length mode */
  rf_setPktLen(CC112X_PKT_LEN_MODE_INFINITE, RF_MAX_FIFO_LEN);
#endif
}


/**
 * @brief handle exit function of TX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_tx_exit(struct s_rf_ctx *p_ctx) {
  LED_TX_OFF();
  p_ctx->txErr = NETSTK_ERR_NONE;
  p_ctx->txStatus = RF_TX_STATUS_NONE;
  p_ctx->txReqAck = FALSE;

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* use infinite packet length mode by default */
  RF_SET_PKT_LEN_MODE(CC112X_PKT_LEN_MODE_INFINITE);
#endif /* NETSTK_CFG_IEEE_802154G_EN */

  TRACE_LOG_MAIN("+++ RF: TX_EXIT");
}


/**
 * @brief radio state dispatcher function
 * @param c_event
 * @param p_data
 */
static void rf_dispatcher(c_event_t c_event, p_data_t p_data) {
  (void)p_data;

  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* handle nested events */
  if (c_event == NETSTK_RF_EVENT) {
    switch (p_ctx->state) {
      case RF_STATE_RX_SYNC:
        rf_rx_sync(p_ctx);
        break;

      case RF_STATE_RX_FINI:
        rf_rx_fini(p_ctx);
        break;

      default:
        /* unexpected event */
        TRACE_LOG_ERR("+++ RF: Dispatcher unexpected, lastState=%02x", p_ctx->dbgLastTriggedState);
        break;
    }
  }
}



/*
 ********************************************************************************
 *                               MISCELLANEOUS
 ********************************************************************************
 */

/**
 * @brief put the radio into idle listening
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_listen(struct s_rf_ctx *p_ctx) {
  if (p_ctx->cfgWOREnabled == TRUE) {
    /* eWOR mode */
    cc112x_spiCmdStrobe(CC112X_SWOR);
  } else {
    /* RX mode */
    cc112x_spiCmdStrobe(CC112X_SRX);
  }
}

/**
 * @brief configure multiple radio registers
 * @param p_regs  point to variable holding register-value
 * @param len     number of registers to configure
 */
static void rf_configureRegs(const s_regSettings_t *p_regs, uint8_t len) {
  uint8_t ix;
  uint8_t data;

  for (ix = 0; ix < len; ix++) {
    data = p_regs[ix].data;
    cc112x_spiRegWrite(p_regs[ix].addr, &data, 1);
  }
}

/**
 * @brief read appended status of the most recently received frame
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_readRxStatus(struct s_rf_ctx *p_ctx) {
  uint8_t rxStatus[2] = {0};

  cc112x_spiRxFifoRead(rxStatus, 2);
  p_ctx->rxRSSI = (int8_t)(rxStatus[0]) - RF_RSSI_OFFSET;
  p_ctx->rxLQI = rxStatus[1] & 0x7F;
}

/**
 * @brief read bytes from RX FIFO
 * @param p_ctx     point to variable holding radio context structure
 * @param numBytes  number of bytes to read
 * @return
 */
static void rf_readRxFifo(struct s_rf_ctx *p_ctx, uint8_t numBytes) {
  /* read a given number of bytes from RX FIFO and append to RX buffer */
  cc112x_spiRxFifoRead(&p_ctx->rxBuf[p_ctx->rxLastDataPtr], numBytes);
  p_ctx->rxLastDataPtr += numBytes;
}


#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
/**
 * @brief set packet length mode and packet size
 * @param mode
 * @param len
 */
static void rf_setPktLen(uint8_t mode, uint16_t len) {

  uint8_t writeByte;

  /* set packet length */
  writeByte = len % (RF_MAX_FIFO_LEN + 1);
  RF_SET_PKT_LEN(writeByte);

  /* set packet length mode */
  RF_SET_PKT_LEN_MODE(mode);
}
#endif /* NETSTK_CFG_IEEE_802154G_EN */


#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void rf_manualCalibration(void) {
  uint8_t original_fs_cal2;
  uint8_t calResults_for_vcdac_start_high[3];
  uint8_t calResults_for_vcdac_start_mid[3];
  uint8_t marcstate;
  uint8_t writeByte;

  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112x_spiRegWrite(CC112X_FS_VCO2, &writeByte, 1);

  // 2) Start with high VCDAC (original VCDAC_START + 2):
  cc112x_spiRegRead(CC112X_FS_CAL2, &original_fs_cal2, 1);
  writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
  cc112x_spiRegWrite(CC112X_FS_CAL2, &writeByte, 1);

  // 3) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  cc112x_spiCmdStrobe(CC112X_SCAL);

  do {
    cc112x_spiRegRead(CC112X_MARCSTATE, &marcstate, 1);
  } while (marcstate != 0x41);

  // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
  //    high VCDAC_START value
  cc112x_spiRegRead(CC112X_FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
  cc112x_spiRegRead(CC112X_FS_VCO4, &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
  cc112x_spiRegRead(CC112X_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

  // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112x_spiRegWrite(CC112X_FS_VCO2, &writeByte, 1);

  // 6) Continue with mid VCDAC (original VCDAC_START):
  writeByte = original_fs_cal2;
  cc112x_spiRegWrite(CC112X_FS_CAL2, &writeByte, 1);

  // 7) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  cc112x_spiCmdStrobe(CC112X_SCAL);

  do {
    cc112x_spiRegRead(CC112X_MARCSTATE, &marcstate, 1);
  } while (marcstate != 0x41);

  // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
  //    with mid VCDAC_START value
  cc112x_spiRegRead(CC112X_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
  cc112x_spiRegRead(CC112X_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
  cc112x_spiRegRead(CC112X_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

  // 9) Write back highest FS_VCO2 and corresponding FS_VCO
  //    and FS_CHP result
  if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
    writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
    cc112x_spiRegWrite(CC112X_FS_VCO2, &writeByte, 1);

    writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
    cc112x_spiRegWrite(CC112X_FS_VCO4, &writeByte, 1);

    writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
    cc112x_spiRegWrite(CC112X_FS_CHP, &writeByte, 1);

  } else {
    writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
    cc112x_spiRegWrite(CC112X_FS_VCO2, &writeByte, 1);

    writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
    cc112x_spiRegWrite(CC112X_FS_VCO4, &writeByte, 1);

    writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
    cc112x_spiRegWrite(CC112X_FS_CHP, &writeByte, 1);
  }
}


static void rf_calibrateRCOsc(void) {
  uint8_t temp;

  /* Read current register value */
  cc112x_spiRegRead(CC112X_WOR_CFG0, &temp, 1);

  /* Mask register bit fields and write new values */
  temp = (temp & 0xF9) | (0x02 << 1);

  /* Write new register value */
  cc112x_spiRegWrite(CC112X_WOR_CFG0, &temp, 1);

  /* Strobe IDLE to calibrate the RCOSC */
  cc112x_spiCmdStrobe(CC112X_SIDLE);

  /* Disable RC calibration */
  temp = (temp & 0xF9) | (0x00 << 1);
  cc112x_spiRegWrite(CC112X_WOR_CFG0, &temp, 1);
}


static void rf_cca(e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  uint8_t rssi_status;

  if (rf_ctx.state != RF_STATE_RX_IDLE) {
    *p_err = NETSTK_ERR_BUSY;
  } else {
    /* poll for carrier sense valid until timeout */
    do {
      cc112x_spiRegRead(CC112X_RSSI0, &rssi_status, 1);

      /* keep polling until either
       * - carrier sense is valid
       * - a packet is arrived
       */
    } while (((rssi_status & RF_RSSI0_CARRIER_SENSE_VALID ) == 0)
        && (rf_ctx.state == RF_STATE_RX_IDLE));

    /* read CCA value */
    /* has a packet arrived? */
    if (rf_ctx.state != RF_STATE_RX_IDLE) {
      /* then set packet reception a higher priority than CCA */
      *p_err = NETSTK_ERR_BUSY;
    }
    /* is radio still in RX state? */
    else {
      /* was carrier detected? */
      if (rssi_status & RF_RSSI0_CARRIER_DETECTED) {
        /* then declare channel busy */
        *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
      }
      else {
        /* declare channel free */
        *p_err = NETSTK_ERR_NONE;
      }
    }
  }
}


#if (NETSTK_CFG_RF_RETX_EN == TRUE)
static void cc112x_retx(e_nsErr_t *p_err)
{
  if ((rf_ctx.state != RF_STATE_RX_IDLE) &&
      (rf_ctx.state != RF_STATE_IDLE)) {
    *p_err = NETSTK_ERR_BUSY;
  } else {
    /* entry:
    * - go to the IDLE state
    * - disable RX interrupt
    */
    cc112x_gotoIdle();
    rf_waitRdy();
    RF_INT_DISABLE();

    /* do:
    * - go to the TX_BUSY state
    * - rewrite TX first pointer
    * - enable TX interrupts
    * - issue STX strobe
    */
    rf_ctx.state = RF_STATE_TX_BUSY;
    cc112x_spiRegWrite(CC112X_TXFIRST, &rf_txFirstPtr, 1);


    LED_TX_ON();
    RF_INT_TX_ENABLED();
    cc112x_spiCmdStrobe(CC112X_STX);

    /* wait for packet to be sent */
    uint16_t iteration = 0xffff;
    while ((rf_ctx.state == RF_STATE_TX_BUSY) && (iteration > 0)) {
      iteration--;
    }
    if (rf_ctx.state == RF_STATE_TX_FINI) {
      /* TX finished successfully */
      *p_err = NETSTK_ERR_NONE;
    } else {
      /* TX error handling */
      *p_err = NETSTK_ERR_TX_TIMEOUT;
    }

    /* exit:
    * - go to state RX
    */
    LED_TX_OFF();
    cc112x_gotoRX();
  }
}
#endif


static void rf_reset(void) {
  /* by issuing a manual reset, all internal registers are set to their default
  * values and the radio will go to the IDLE state
  */
  cc112x_spiCmdStrobe(CC112X_SRES);

  /* wait for the crystal oscillator to stabilize */
  rf_waitRdy();
}

static void rf_chkPartnumber(e_nsErr_t *p_err) {
  uint8_t part_number;
  uint8_t part_version;

  /* set returned error to default */
  *p_err = NETSTK_ERR_NONE;

  /* get part number */
  cc112x_spiRegRead(CC112X_PARTNUMBER, &part_number, 1);
  if (part_number != CC112X_PART_NUMBER) {
    *p_err = NETSTK_ERR_INIT;
    return;
  }

  /* get part version */
  cc112x_spiRegRead(CC112X_PARTVERSION, &part_version, 1);
  if (part_version != CC112X_PART_VERSION) {
    *p_err = NETSTK_ERR_INIT;
    return;
  }
}


static void rf_waitRdy(void) {
  rf_status_t chip_status;
  do {
    chip_status = cc112x_spiCmdStrobe(CC112X_SNOP);
  } while (chip_status & CC112X_STATE_CHIP_RDYn);
}


static void rf_txPowerSet(int8_t power, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  uint8_t pa_power_ramp;

  /*
   * Output power = (PA_POWER_RAMP+1)/2 - 18 [dBm]
   * PA_POWER_RAMP = PA_CFG1[5:0] ~> PA_POWER_RAMP_MASK = 0x3F
   */
  pa_power_ramp = ((power + 18) * 2 - 1) & 0x3Fu;
  cc112x_spiRegWrite(CC112X_PA_CFG1, &pa_power_ramp, 1);
  *p_err = NETSTK_ERR_NONE;
}


static void rf_txPowerGet(int8_t *p_power, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if (p_power == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  uint8_t pa_power_ramp;

  /*
   * Output power = (PA_POWER_RAMP+1)/2 - 18 [dBm]
   */
  cc112x_spiRegRead(CC112X_PA_CFG1, &pa_power_ramp, 1);
  pa_power_ramp &= 0x3F;
  *p_power = ((pa_power_ramp + 1) / 2) - 18;
  *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Select operating channel number
 * @param   chan_num    Channel number to select
 * @param   p_err       Pointer to variable holding returned error code
 */
static void rf_chanNumSet(uint8_t chan_num, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  uint32_t freq_reg = 0;
  uint32_t freq_center = 0;
  uint32_t freq_delta = 0;
  uint8_t write_byte = 0;

  /* set returned error value to default */
  *p_err = NETSTK_ERR_NONE;

  /* configure operating channel frequency based on operation mode i.e. CSM,
   * mode #1, mode #2, or mode #3 */
  switch (rf_ctx.cfgOpMode) {
    case NETSTK_RF_OP_MODE_CSM:
    if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_CSM) {
      freq_center = CC112X_CSM_CHAN_CENTER_FREQ;
      freq_delta = CC112X_CSM_DELTA_FREQ;
      rf_ctx.cfgFreqChanNum = chan_num;
    } else {
      *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    }
    break;

#if (NETSTK_CFG_PHY_OP_MODE_1_EN == TRUE)
    case NETSTK_RF_OP_MODE_1:
      if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE1) {
        freq_center = CC112X_OPMODE1_CHAN_CENTER_FREQ;
        freq_delta = CC112X_OPMODE1_DELTA_FREQ;
        rf_ctx.cfgFreqChanNum = chan_num;
      } else {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      }
      break;
#endif /* NETSTK_CFG_PHY_OP_MODE_1_EN */

#if (NETSTK_CFG_PHY_OP_MODE_2_EN == TRUE)
      case NETSTK_RF_OP_MODE_2:
      if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE2) {
        freq_center = CC112X_OPMODE2_CHAN_CENTER_FREQ;
        freq_delta = CC112X_OPMODE2_DELTA_FREQ;
        rf_ctx.cfgFreqChanNum = chan_num;
      } else {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      }
      break;
#endif /* NETSTK_CFG_PHY_OP_MODE_2_EN */

#if (NETSTK_CFG_PHY_OP_MODE_3_EN == TRUE)
      case NETSTK_RF_OP_MODE_3:
      if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE3) {
        freq_center = CC112X_OPMODE3_CHAN_CENTER_FREQ;
        freq_delta = CC112X_OPMODE3_DELTA_FREQ;
        rf_ctx.cfgFreqChanNum = chan_num;
      } else {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      }
      break;
#endif /* NETSTK_CFG_PHY_OP_MODE_3_EN */

    default:
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    break;
  }

  /*
   * configure frequency registers only when arguments are valid
   */
  if (*p_err == NETSTK_ERR_NONE) {
    freq_reg = freq_center + rf_ctx.cfgFreqChanNum * freq_delta;

    write_byte = (freq_reg & 0x00FF0000) >> 16;
    cc112x_spiRegWrite(CC112X_FREQ2, &write_byte, 1);

    write_byte = (freq_reg & 0x0000FF00) >> 8;
    cc112x_spiRegWrite(CC112X_FREQ1, &write_byte, 1);

    write_byte = (freq_reg & 0x000000FF);
    cc112x_spiRegWrite(CC112X_FREQ0, &write_byte, 1);
  }

  /* frequency calibration */
  rf_manualCalibration();
}

/**
 * @brief   Select operating mode
 * @param   mode        Operating mode to select
 * @param   p_err       Pointer to variable holding returned error code
 */
static void rf_opModeSet(e_nsRfOpMode mode, e_nsErr_t *p_err) {
  if (rf_ctx.cfgOpMode < NETSTK_RF_OP_MODE_MAX) {
    rf_ctx.cfgOpMode = mode;
    *p_err = NETSTK_ERR_NONE;
  } else {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
  }
}


/**
 * @brief   Read Receiver Signal Strength Indicator
 * @param p_val
 * @param p_err
 */
static void rf_readRSSI(int8_t *p_val, e_nsErr_t *p_err) {
  /* get real RSSI from most recently received frame */
  *p_val = rf_ctx.rxRSSI;
}


/*
 ********************************************************************************
 *                               DRIVER DEFINITION
 ********************************************************************************
 */
const s_nsRF_t rf_driver_ticc112x = {
    "CC112X",
    rf_init,
    rf_on,
    rf_off,
    rf_send,
    rf_recv,
    rf_ioctl,
};


/*
 ********************************************************************************
 *                                   END OF FILE
 ********************************************************************************
 */


