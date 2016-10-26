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
#include "ctimer.h"
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

/*!< Calibration */
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

#ifndef MIN
#define MIN(a_, b_)           (((a_) > (b_)) ? (b_) : (a_))
#endif

#define RF_READ_CHIP_STATE()  \
  (cc112x_spiCmdStrobe(CC112X_SNOP) & 0x70)

/*!< write multiple registers */
#define RF_WR_REGS(regs_) \
  do {  \
    uint8_t len_; \
    len_ = sizeof((regs_)) / sizeof(s_regSettings_t);   \
    rf_configureRegs((regs_), len_);                    \
  } while(0)

/*!< configure packet length mode of radio */
#define RF_SET_PKT_LEN_MODE(mode_)  \
  do {  \
    uint8_t wr_byte = mode_;   \
    cc112x_spiRegWrite(CC112X_PKT_CFG0, &wr_byte, 1);  \
  } while(0)

/*!< configure FIFO threshold */
#define RF_SET_FIFO_THR(thr_) \
  do {  \
    uint8_t wr_byte = thr_;   \
    cc112x_spiRegWrite(CC112X_FIFO_CFG, &wr_byte, 1);  \
  } while (0)

/*!< check if RF is in one of reception states */
#define RF_IS_RX_BUSY() \
    ((rf_ctx.state >= RF_STATE_RX_SYNC)   &&  \
     (rf_ctx.state <= RF_STATE_RX_TXACK_FINI))

/*!< radio state mask */
#define RF_STATE_MASK                   ( 0xF0u )

/*!< Radio interrupt configuration macros */
#define RF_INT_PKT_BEGIN                E_TARGET_EXT_INT_0  //GPIO0
#define RF_INT_RXFIFO_THR               E_TARGET_EXT_INT_1  //GPIO2
#define RF_INT_TXFIFO_THR               E_TARGET_EXT_INT_1  //GPIO2
#define RF_INT_PQT_REACHED              E_TARGET_EXT_INT_1  //GPIO2
#define RF_INT_PKT_END                  E_TARGET_EXT_INT_2  //GPIO3

#define RF_INT_EDGE_PKT_BEGIN           E_TARGET_INT_EDGE_RISING
#define RF_INT_EDGE_RXFIFO_THR          E_TARGET_INT_EDGE_RISING
#define RF_INT_EDGE_TXFIFO_THR          E_TARGET_INT_EDGE_FALLING
#define RF_INT_EDGE_PQT_REACHED         E_TARGET_INT_EDGE_RISING
#define RF_INT_EDGE_PKT_END             E_TARGET_INT_EDGE_FALLING

#define RF_IOCFG_PKT_BEGIN              0u
#define RF_IOCFG_RXFIFO_THR             1u
#define RF_IOCFG_PKT_END                2u

#define RF_IOCFG_TXFIFO_THR             3u
#define RF_IOCFG_PQT_REACHED            4u

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
  RF_STATE_RX_TXACK_FINI  = 0x14,
  RF_STATE_RX_TERM        = 0x15,

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
  uint8_t regVerify;
  uint8_t isAckRequired;
  uint8_t pktLenMode;

  uint16_t rxBytesCounter;
  uint16_t rxNumRemBytes;
  uint32_t rxChksum;
  uint8_t rxIsAddrFiltered;

  /* RX state attributes */
  uint8_t rxReqAck;
  uint8_t rxBuf[RF_MAX_PACKET_LEN];
  uint16_t rxLastDataPtr;
  uint16_t rxLastChksumPtr;

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
  uint16_t txNumRemBytes;
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
static void rf_txFifoThresholdISR(void *p_arg);
static void rf_pktRxTxEndISR(void *p_arg);

static void rf_readRxStatus(struct s_rf_ctx *p_ctx);
static uint8_t rf_readRxFifo(struct s_rf_ctx *p_ctx, uint8_t numBytes);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
static void rf_setPktLen(uint8_t mode, uint16_t len);
#endif

static uint8_t rf_gotoIdle(struct s_rf_ctx *p_ctx);
static uint8_t rf_gotoRx(struct s_rf_ctx *p_ctx);
static uint8_t rf_gotoWor(struct s_rf_ctx *p_ctx);

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
#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
static void rf_rx_txAckSync(struct s_rf_ctx *p_ctx);
static void rf_rx_txAckFini(struct s_rf_ctx *p_ctx);
#endif

static void rf_tx_entry(struct s_rf_ctx *p_ctx);
static void rf_tx_sync(struct s_rf_ctx *p_ctx);
static void rf_tx_fini(struct s_rf_ctx *p_ctx);
static void rf_tx_term(struct s_rf_ctx *p_ctx);
static void rf_tx_exit(struct s_rf_ctx *p_ctx);
#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
static void rf_tx_rxAckSync(struct s_rf_ctx *p_ctx);
static void rf_tx_rxAckFini(struct s_rf_ctx *p_ctx);
#endif

static void rf_eventHandler(c_event_t c_event, p_data_t p_data);
static void rf_exceptionHandler(struct s_rf_ctx *p_ctx, uint8_t marcStatus, e_rfState_t chipState);

static void rf_configureRegs(const s_regSettings_t *p_regs, uint8_t len);
static void rf_manualCalibration(void);
static void rf_calibrateRCOsc(void);
static void rf_cca(struct s_rf_ctx *p_ctx, e_nsErr_t *p_err);

#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
static uint8_t rf_filterAddr(struct s_rf_ctx *p_ctx);
#endif

static void rf_chkReset(struct s_rf_ctx *p_ctx);
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
#if (EMB6_TEST_CFG_WOR_EN == TRUE)
  RF_WR_REGS(cc112x_cfg_worSmartRFTesting);
#else
  RF_WR_REGS(cc112x_cfg_ieee802154g_default);
#endif

  RF_SET_FIFO_THR(RF_CFG_FIFO_THR);

  /* calibrate radio according to cc112x errata */
  rf_manualCalibration();

  /* calibrate RC oscillator */
  rf_calibrateRCOsc();

  /* initialize local variables */
  evproc_regCallback(NETSTK_RF_EVENT, rf_eventHandler);

  /* configure operating mode and channel number */
  p_ctx->cfgFreqChanNum = 0;
  p_ctx->cfgOpMode = NETSTK_RF_OP_MODE_CSM;

  /* transition to SLEEP state */
  rf_sleep_entry(p_ctx);
}


/**
 * @brief turn radio driver on
 * @param p_err   point to variable storing return error code
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
  if ((p_ctx->state & RF_STATE_MASK) == RF_STATE_SLEEP) {
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
 * @param p_err   point to variable storing return error code
 */
static void rf_off(e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* set default return error code */
  *p_err = NETSTK_ERR_NONE;

  /* is the driver on? */
  if ((p_ctx->state & RF_STATE_MASK) != RF_STATE_SLEEP) {
    if (RF_IS_RX_BUSY() == TRUE) {
      /* a frame is being received */
      *p_err = NETSTK_ERR_BUSY;
    }
    else {
      /* then exits ON state */
      rf_on_exit(p_ctx);

      /* then turn it off */
      rf_sleep_entry(p_ctx);
    }
  }
}


/**
 * @brief handle transmission request from upper layer
 * @param p_data    point to buffer holding frame to send
 * @param len       length of frame to send
 * @param p_err     point to variable storing return error code
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
  uint32_t txTimeout;
  uint8_t enterRx;
  uint8_t exitTx;
  uint8_t numWrBytes;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  if (RF_IS_RX_BUSY() == TRUE) {
    /* radio is in middle of packet reception process */
    *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
    return;
  }

  /* otherwise start transmission process */

  /* put radio back to RX state if eWOR is enabled */
  if (p_ctx->cfgWOREnabled == TRUE) {
    if (rf_gotoIdle(p_ctx) == FALSE) {
      /* radio is in middle of packet reception process */
      *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
      return;
    }
  }
  p_ctx->txDataPtr = p_data;
  p_ctx->txDataLen = len;
  p_ctx->txNumRemBytes = len;
  p_ctx->txStatus = RF_TX_STATUS_NONE;
  enterRx = TRUE;
  exitTx = TRUE;
  txTimeout = 0;

  /* simply issue the radio transmission command right away */
  cc112x_spiCmdStrobe(CC112X_STX);

  /* initial one-time delay dedicated for TXONCCA process */
  bsp_delay_us(200);

  /* wait for complete transmission of the frame until timeout */
  tickstart = bsp_getTick();
  do {
    /* was the frame transmitted successfully? */
    if ((p_ctx->txStatus == RF_TX_STATUS_DONE) ||
        (p_ctx->txStatus == RF_TX_STATUS_WFA)) {
      break;
    }

    /* was a frame being received? */
    if (RF_IS_RX_BUSY() == TRUE) {
      p_ctx->txErr = NETSTK_ERR_TX_COLLISION;
      rf_tx_term(p_ctx);
      break;
    }

    /* write remaining bytes to send to TXFIFO */
    if (p_ctx->txNumRemBytes > 0) {
      /* write as much as RF_CFG_TX_FIFO_THR bytes */
      if (p_ctx->txNumRemBytes > RF_CFG_TX_FIFO_THR) {
        numWrBytes = RF_CFG_TX_FIFO_THR;
      }
      else {
        numWrBytes = p_ctx->txNumRemBytes;
      }
      rf_writeTxFifo(p_ctx, numWrBytes, p_err);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
      /* switch to fixed packet length mode if number of remaining bytes is less than FIFO SIZE
       * the actual number of remaining bytes to be sent is calculated as follow:
       * totNumRemBytes = numBytesInTxFifo + numRemBytes
       * numBytesInTxFifo = RF_CFG_NUM_TXBYTES
       */
      if( ( p_ctx->pktLenMode == CC112X_PKT_LEN_MODE_INFINITE ) &&
          ( p_ctx->txNumRemBytes < ( RF_MAX_FIFO_LEN + 1 - RF_CFG_NUM_TXBYTES ) ) ) {
        rf_setPktLen( CC112X_PKT_LEN_MODE_FIXED, p_ctx->txDataLen );
      }
#endif /* NETSTK_CFG_IEEE_802154G_EN */

      /* TODO calculate CRC if enabled */

      /* small delay to reduce overhead due to SPI communication */
      bsp_delay_us(500);
    }

    /* update TX timeout */
    txTimeout = bsp_getTick() - tickstart;

  } while (txTimeout < 40); // 40ms

  if (p_ctx->txStatus == RF_TX_STATUS_DONE) {
    if (p_ctx->txErr == NETSTK_ERR_FATAL) {
      /* exception was thrown and radio was already put to RX_IDLE state or
       * channel was detected busy and radio was already put to RX_SYNC state */
      enterRx = FALSE;
    }
  }
#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
  else if (p_ctx->txStatus == RF_TX_STATUS_WFA) {
    /* reset TX status to default before waiting for ACK */
    p_ctx->txErr = NETSTK_ERR_TX_NOACK;

    /* wait for ACK for at most ackWaitDuration */
    packetbuf_attr_t waitForAckTimeout;
    waitForAckTimeout = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION);
    bsp_delay_us(waitForAckTimeout);
  }
#endif /* NETSTK_CFG_RF_CC112X_AUTOACK_EN */
  else {
    /* packet failed to be transmit within timeout */
    p_ctx->txErr = NETSTK_ERR_TX_TIMEOUT;
    TRACE_LOG_ERR("<TX> timeout");

    /* check if transceiver was reset */
    rf_chkReset(p_ctx);
  }

  /* set return error code */
  *p_err = p_ctx->txErr;

  /* is chip already set to idle listening */
  if (p_ctx->txErr == NETSTK_ERR_TX_NOACK) {
    p_ctx->state = RF_STATE_RX_IDLE;
    enterRx = FALSE;
  }

  /* exit TX state */
  if (exitTx == TRUE) {
    rf_tx_exit(p_ctx);
  }

  /* need to enter RX state? */
  if (enterRx == TRUE) {
    rf_rx_entry(p_ctx);
  }
}


/**
 * @brief handle incoming frame reception
 * @param p_buf   point to buffer holding the received frame
 * @param len     length of the frame
 * @param p_err   point to variable storing return error code
 */
static void rf_recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* set default error code */
  *p_err = NETSTK_ERR_NONE;

  /* signal upper layer */
  p_ctx->p_netstk->phy->recv(p_ctx->rxBuf, p_ctx->rxBytesCounter, p_err);

  /* is the frame discarded by upper layers? */
  if (*p_err != NETSTK_ERR_NONE) {
    TRACE_LOG_ERR("<RXFINI> discarded e=-%d", *p_err);
    trace_printHex("", p_ctx->rxBuf, p_ctx->rxBytesCounter);
  }

  /* exit and re-enter RX state */
  rf_rx_exit(p_ctx);
  rf_rx_entry(p_ctx);
}

/**
 * @brief handle miscellaneous commands
 * @param cmd     command identifier
 * @param p_val   opaque pointer referenced to variable holding/storing attribute value
 * @param p_err   point to variable storing return error code
 */
static void rf_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  uint8_t rfendCfg0;
  uint8_t preambleCfg1;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    case NETSTK_CMD_RF_TXPOWER_SET:
      rf_txPowerSet(*((int8_t *) p_val), p_err);
      break;

    case NETSTK_CMD_RF_TXPOWER_GET:
      rf_txPowerGet(p_val, p_err);
      break;

    case NETSTK_CMD_RF_CCA_GET:
      rf_cca(p_ctx, p_err);
      break;

    case NETSTK_CMD_RF_IS_RX_BUSY:
      if (p_val == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      }
      else {
        *((uint8_t *) p_val) = RF_IS_RX_BUSY() || p_ctx->rxBytesCounter;
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
        /* backup current state */
        e_rfState_t prevState = p_ctx->state;
        if (prevState == RF_STATE_SLEEP) {
          rf_sleep_exit(p_ctx);
        }
        else {
          rf_on_exit(p_ctx);
        }

        rf_ctx.cfgWOREnabled = *((uint8_t *) p_val);

        /* read value of registers to modify */
        cc112x_spiRegRead(CC112X_RFEND_CFG0, &rfendCfg0, 1);
        cc112x_spiRegRead(CC112X_PREAMBLE_CFG1, &preambleCfg1, 1);

        if (rf_ctx.cfgWOREnabled == TRUE) {
          /* enable RX termination based on CS */
          rfendCfg0 |= 0x09;
          cc112x_spiRegWrite(CC112X_RFEND_CFG0, &rfendCfg0, 1);

          /* set preamble length to 24 bytes */
          preambleCfg1 = 0x31;
          cc112x_spiRegWrite(CC112X_PREAMBLE_CFG1, &preambleCfg1, 1);
        }
        else {
          /* disable RX termination based on CS */
          rfendCfg0 &= ~0x09;
          cc112x_spiRegWrite(CC112X_RFEND_CFG0, &rfendCfg0, 1);

          /* set preamble length to 4 bytes */
          preambleCfg1 = 0x19;
          cc112x_spiRegWrite(CC112X_PREAMBLE_CFG1, &preambleCfg1, 1);
        }

        /* restore previous state */
        if (prevState == RF_STATE_SLEEP) {
          rf_sleep_entry(p_ctx);
        }
        else {
          rf_on_entry(p_ctx);
        }
      }
      else {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
      }
      break;

    case NETSTK_CMD_RX_BUF_READ:
      /*
       * Signal upper layer if a packet has arrived by the time this
       * command is issued.
       * Trigger event-process manually
       */
      rf_eventHandler(NETSTK_RF_EVENT, NULL);
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
 * @param p_arg   point to variable holding callback argument
 */
static void rf_pktRxTxBeginISR(void *p_arg) {
  (void) &p_arg;

  uint8_t marc_status;
  uint8_t chip_state;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* entry */
  bsp_extIntClear(RF_INT_PKT_BEGIN);
  cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);
  chip_state = RF_READ_CHIP_STATE();
  TRACE_LOG_MAIN("<B>: ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);

  /* FIXME unexpected marc */
  if ((marc_status == RF_MARC_STATUS_RX_FINI) ||
      (marc_status == RF_MARC_STATUS_TX_FINI)) {
    marc_status = RF_MARC_STATUS_NO_FAILURE;
  }

  /* do */
  switch (marc_status) {
    case RF_MARC_STATUS_TX_OVERFLOW :
    case RF_MARC_STATUS_TX_UNDERFLOW :
      TRACE_LOG_ERR("<B> TX_ERR ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
      rf_tx_term(p_ctx);
      rf_rx_exit(p_ctx);
      rf_rx_entry(p_ctx);
      break;

    case RF_MARC_STATUS_RX_TERM :
      if (p_ctx->cfgWOREnabled == TRUE) {
        if (chip_state == RF_STATE_TX) {
          /* the radio just finished transmitting SYNC words */
          rf_tx_sync(p_ctx);
          break;
        }
      }
      /* no break */
    case RF_MARC_STATUS_RX_OVERFLOW :
    case RF_MARC_STATUS_RX_UNDERFLOW :
    case RF_MARC_STATUS_RX_TIMEOUT :
      TRACE_LOG_ERR("<B> RX_ERR ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
      rf_rx_term(p_ctx);
      break;

    default: {
      /* 'good' marc_status */
      switch (p_ctx->state) {
        case RF_STATE_RX_IDLE:
          if (marc_status == RF_MARC_STATUS_NO_FAILURE) {
            if (chip_state == RF_STATE_TX) {
              /* the radio just finished transmitting SYNC words */
              rf_tx_sync(p_ctx);
            } else {
              /* the radio just finished receiving SYNC words */
              rf_rx_sync(p_ctx);
            }
          } else if (marc_status == RF_MARC_STATUS_TX_ON_CCA_FAILED) {
            /* the radio just finished receiving SYNC words */
            rf_rx_sync(p_ctx);
          } else {
            TRACE_LOG_ERR("<B> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
            rf_exceptionHandler(p_ctx, marc_status, chip_state);
          }
          break;

        case RF_STATE_RX_FINI:
          if ((marc_status == RF_MARC_STATUS_NO_FAILURE) && (p_ctx->rxReqAck == TRUE)) {
            /* SYNC words of ACK frame were transmitted */
            rf_rx_txAckSync(p_ctx);
          } else {
            TRACE_LOG_ERR("<B> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
            rf_exceptionHandler(p_ctx, marc_status, chip_state);
          }
          break;

        case RF_STATE_TX_FINI:
          if ((marc_status == RF_MARC_STATUS_NO_FAILURE) && (p_ctx->txReqAck == TRUE)) {
            /* the incoming frame can be the ACK */
            rf_tx_rxAckSync(p_ctx);
          } else {
            TRACE_LOG_ERR("<B> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
            rf_exceptionHandler(p_ctx, marc_status, chip_state);
          }
          break;

        default:
          TRACE_LOG_ERR("<B> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
          rf_exceptionHandler(p_ctx, marc_status, chip_state);
          break;
      } /* end switch */
      break;
    }
  }

  /* exit */
  TRACE_LOG_MAIN("</B>: ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
}


/**
 * @brief interrupt subroutine to handle RXFIFO threshold
 * @param p_arg   point to variable holding callback argument
 */
static void rf_rxFifoThresholdISR(void *p_arg) {

  (void) &p_arg;

  uint8_t isRxOk = TRUE;
  uint8_t numRxBytes = 0;
  uint8_t numChksumBytes = 0;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* entry */
  bsp_extIntClear(RF_INT_RXFIFO_THR);
  if ((p_ctx->state != RF_STATE_RX_SYNC) &&
      (p_ctx->state != RF_STATE_TX_RXACK_SYNC)) {
    return;
  }

  /* read number of bytes in RXFIFO */
  cc112x_spiRegRead(CC112X_NUM_RXBYTES, &numRxBytes, 1);
  if (numRxBytes < RF_CFG_FIFO_THR) {
    /* number of received bytes is not sufficient */
    TRACE_LOG_MAIN("<RXFIFO_THR> insufficient %d %d %02x", numRxBytes, RF_CFG_FIFO_THR, p_ctx->state);
    return;
  }

  if ((p_ctx->rxNumRemBytes == 0) ||
      (p_ctx->rxNumRemBytes > (RF_CFG_FIFO_THR + 1))) {

    /* read bytes from RXFIFO */
    if (rf_readRxFifo(p_ctx, numRxBytes) == 0) {
      isRxOk = FALSE;
    }
    else {
      isRxOk = TRUE;

      if (p_ctx->rxLastDataPtr > LLFRAME_MIN_NUM_RX_BYTES) {
        /* compute number of checksum bytes */
        p_ctx->rxNumRemBytes -= numRxBytes;
        if (p_ctx->rxNumRemBytes < p_ctx->rxFrame.crc_len) {
          numChksumBytes = numRxBytes - (p_ctx->rxFrame.crc_len - p_ctx->rxNumRemBytes);
        }
        else {
          numChksumBytes = numRxBytes;
        }

  #if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
        /* is address not yet checked and number of received bytes sufficient for
        * address filtering? */
        if ((p_ctx->rxIsAddrFiltered == FALSE) &&
            (p_ctx->rxLastDataPtr >= (LLFRAME_MIN_NUM_RX_BYTES + p_ctx->rxFrame.min_addr_len))) {
          isRxOk = rf_filterAddr(p_ctx);
        }
  #endif /* NETSTK_CFG_RF_SW_AUTOACK_EN */
      }
      else {
        /* first chunk of bytes is sufficient to obtain incoming frame information */
        p_ctx->rxNumRemBytes = llframe_parse(&p_ctx->rxFrame, p_ctx->rxBuf, p_ctx->rxLastDataPtr);
        if (p_ctx->rxNumRemBytes == 0) {
          isRxOk = FALSE;
        }
        else {
          /* set total number of received bytes */
          p_ctx->rxBytesCounter = p_ctx->rxFrame.tot_len;

  #if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
          /* switch to fixed packet length mode if total of remaining bytes is less than RF_MAX_FIFO_LEN */
          if ((p_ctx->pktLenMode == CC112X_PKT_LEN_MODE_INFINITE) &&
              (p_ctx->rxBytesCounter < (RF_MAX_FIFO_LEN + 1))) {
            rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->rxBytesCounter);
          }
  #endif /* NETSTK_CFG_IEEE_802154G_EN */

          /* initialize checksum */
          p_ctx->rxChksum = llframe_crcInit(&p_ctx->rxFrame);
          p_ctx->rxLastChksumPtr = p_ctx->rxFrame.crc_offset;

          /* compute number of checksum bytes */
          numChksumBytes = p_ctx->rxLastDataPtr - p_ctx->rxLastChksumPtr;

  #if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
          /* filter unwanted ACKs */
          if ((p_ctx->rxBuf[PHY_HEADER_LEN ] == 0x02) && (p_ctx->txReqAck == FALSE)) {
            isRxOk = FALSE;
          }
  #endif /* NETSTK_CFG_RF_SW_AUTOACK_EN */
        }
      }
    }
  }

  /* is RX process OK so far? */
  if (isRxOk == FALSE) {
    rf_rx_term(p_ctx);
  }
  else {
    /* otherwise update checksum if there is no error during RX process */
    p_ctx->rxChksum = llframe_crcUpdate(&p_ctx->rxFrame, &p_ctx->rxBuf[p_ctx->rxLastChksumPtr], numChksumBytes, p_ctx->rxChksum);
    p_ctx->rxLastChksumPtr += numChksumBytes;

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
    /* switch to fixed packet length mode if number of remaining bytes is less than FIFO_SIZE */
    if ((p_ctx->pktLenMode == CC112X_PKT_LEN_MODE_INFINITE) &&
        (p_ctx->rxNumRemBytes < (RF_MAX_FIFO_LEN + 1))) {
      rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->rxBytesCounter);
    }

    /* disable RXFIFO_THR interrupt once number of remaining bytes is less than FIFO_THR */
    if (p_ctx->rxNumRemBytes <= RF_CFG_FIFO_THR) {
      bsp_extIntDisable(RF_INT_RXFIFO_THR);
    }
#endif /* NETSTK_CFG_IEEE_802154G_EN */
  }
}


/**
 * @brief interrupt subroutine to handle TXFIFO threshold
 * @param p_arg   point to variable holding callback argument
 */
static void rf_txFifoThresholdISR(void *p_arg) {

  e_nsErr_t err;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  uint8_t numWrBytes;
  if (p_ctx->txNumRemBytes > RF_CFG_FIFO_THR) {
    numWrBytes = RF_CFG_FIFO_THR;
  }
  else {
    numWrBytes = p_ctx->txNumRemBytes;
  }
  rf_writeTxFifo(p_ctx, numWrBytes, &err);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* switch to fixed packet length mode if number of remaining bytes is less than FIFO SIZE
   * the actual number of remaining bytes to be sent is calculated as follow:
   * totNumRemBytes = numBytesInTxFifo + numRemBytes
   * numBytesInTxFifo = RF_CFG_NUM_TXBYTES
   */
  if ((p_ctx->pktLenMode == CC112X_PKT_LEN_MODE_INFINITE) &&
      (p_ctx->txNumRemBytes < (RF_MAX_FIFO_LEN + 1 - RF_CFG_NUM_TXBYTES))) {
    rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->txDataLen);
  }
#endif /* NETSTK_CFG_IEEE_802154G_EN */
}


/**
 * @brief interrupt subroutine to handle ending of packet reception/transmission
 * @param p_arg   point to variable holding callback argument
 */
static void rf_pktRxTxEndISR(void *p_arg) {
  (void) &p_arg;

  uint8_t marc_status;
  uint8_t chip_state;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* entry */
  bsp_extIntClear(RF_INT_PKT_END);
  if ((p_ctx->cfgWOREnabled == TRUE) &&
      ((p_ctx->state == RF_STATE_RX_IDLE) ||
       (p_ctx->state == RF_STATE_RX_FINI) ||
       (p_ctx->state == RF_STATE_RX_TXACK_FINI) ||
       (p_ctx->state == RF_STATE_TX_FINI))) {
    /* this interrupt was triggered by periodic WOR timer then ignore */
    return;
  }

  /* otherwise read MARC_STATUS to see cause of the interrupt */
  cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);
  chip_state = RF_READ_CHIP_STATE();
  TRACE_LOG_MAIN("<E>: ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);

  switch (marc_status) {
    case RF_MARC_STATUS_TX_OVERFLOW :
    case RF_MARC_STATUS_TX_UNDERFLOW :
      TRACE_LOG_ERR("<E> TX_ERR ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
      rf_tx_term(p_ctx);
      rf_rx_exit(p_ctx);
      rf_rx_entry(p_ctx);
      break;

    case RF_MARC_STATUS_RX_OVERFLOW :
    case RF_MARC_STATUS_RX_UNDERFLOW :
    case RF_MARC_STATUS_RX_TIMEOUT :
    case RF_MARC_STATUS_RX_TERM :
      TRACE_LOG_ERR("<E> RX_ERR ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
      rf_rx_term(p_ctx);
      break;

    default: {
      /* 'good' marc_status */
      switch (p_ctx->state) {
        case RF_STATE_RX_SYNC:
          if ((marc_status == RF_MARC_STATUS_RX_FINI) && (p_ctx->rxBytesCounter)) {
            rf_rx_chksum(p_ctx);
          } else {
            TRACE_LOG_ERR("<E> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
            rf_exceptionHandler(p_ctx, marc_status, chip_state);
          }
          break;

        case RF_STATE_RX_TXACK_SYNC:
          if (marc_status == RF_MARC_STATUS_TX_FINI) {
            /* a responding ACK was successfully transmitted then signal upper layer */
            rf_rx_txAckFini(p_ctx);
          } else {
            TRACE_LOG_ERR("<E> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
            rf_exceptionHandler(p_ctx, marc_status, chip_state);
          }
          break;

        case RF_STATE_TX_SYNC:
          if (marc_status == RF_MARC_STATUS_TX_FINI) {
            /* a frame was successfully transmitted */
            rf_tx_fini(p_ctx);
          } else {
            TRACE_LOG_ERR("<E> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
            rf_exceptionHandler(p_ctx, marc_status, chip_state);
          }
          break;

        case RF_STATE_TX_RXACK_SYNC:
          if (marc_status == RF_MARC_STATUS_RX_FINI) {
            /* verify acknowledgment */
            rf_tx_rxAckFini(p_ctx);
          } else {
            TRACE_LOG_ERR("<E> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
            rf_exceptionHandler(p_ctx, marc_status, chip_state);
          }
          break;

        default:
          TRACE_LOG_ERR("<E> exception ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
          rf_exceptionHandler(p_ctx, marc_status, chip_state);
          break;
      } /* end switch */
      break;
    }
  }

  /* exit */
  TRACE_LOG_MAIN("</E>: ds=%02x, ms=%02x, cs=%02x", p_ctx->state, marc_status, chip_state);
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
  /* force radio to enter IDLE state */
  rf_gotoIdle(p_ctx);

  /* configure RX GPIOx */
  RF_WR_REGS(cc112x_cfg_iocfgOff);

  /* finally put the radio to POWERDOWN state */
  p_ctx->state = RF_STATE_SLEEP;
  cc112x_spiCmdStrobe(CC112X_SPWD);
}


/**
 * @brief handle exit function of SLEEP state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_sleep_exit(struct s_rf_ctx *p_ctx) {

  /* put radio to IDLE state */
  p_ctx->state = RF_STATE_IDLE;
  rf_gotoIdle(p_ctx);
}


/**
 * @brief handle entry function of ON state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_on_entry(struct s_rf_ctx *p_ctx) {

  /* configure common RX GPIOx */
  rf_intConfig(RF_IOCFG_PKT_BEGIN, RF_INT_PKT_BEGIN, RF_INT_EDGE_PKT_BEGIN, rf_pktRxTxBeginISR);
  rf_intConfig(RF_IOCFG_PKT_END, RF_INT_PKT_END, RF_INT_EDGE_PKT_END, rf_pktRxTxEndISR);

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

  /* put radio to IDLE state */
  p_ctx->state = RF_STATE_IDLE;
  rf_gotoIdle(p_ctx);
}


/**
 * @brief handle entry function of RX state
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_rx_entry(struct s_rf_ctx *p_ctx) {

  /* initialize RX attributes */
  p_ctx->rxReqAck = FALSE;
  p_ctx->rxNumRemBytes = 0;
  p_ctx->rxLastDataPtr = 0;
  p_ctx->rxLastChksumPtr = 0;
  p_ctx->rxBytesCounter = 0;
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

  /* configure RF interrupt */
  rf_intConfig(RF_IOCFG_RXFIFO_THR, RF_INT_RXFIFO_THR, RF_INT_EDGE_RXFIFO_THR, rf_rxFifoThresholdISR);
}


/**
 * @brief calculate checksum of received frames
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_rx_chksum(struct s_rf_ctx *p_ctx) {

  uint8_t isChecksumOK;
  uint8_t numChksumBytes;

  /* read remaining bytes from RX FIFO */
  if (p_ctx->rxNumRemBytes) {
    rf_readRxFifo(p_ctx, p_ctx->rxNumRemBytes);
    p_ctx->rxNumRemBytes = 0;
  }

#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
  /* update checksum */
  if (p_ctx->rxLastDataPtr > (p_ctx->rxFrame.crc_len + p_ctx->rxLastChksumPtr)) {
    numChksumBytes = p_ctx->rxLastDataPtr - (p_ctx->rxLastChksumPtr + p_ctx->rxFrame.crc_len);
    p_ctx->rxChksum = llframe_crcUpdate(&p_ctx->rxFrame, &p_ctx->rxBuf[p_ctx->rxLastChksumPtr], numChksumBytes, p_ctx->rxChksum);
    p_ctx->rxLastChksumPtr += numChksumBytes;
  }
  p_ctx->rxChksum = llframe_crcFinal(&p_ctx->rxFrame, p_ctx->rxChksum);

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

    p_ctx->state = RF_STATE_RX_FINI;
    if (p_ctx->rxReqAck == FALSE) {
      /* signal upper layer */
      evproc_putEvent(E_EVPROC_HEAD, NETSTK_RF_EVENT, p_ctx);
    }
  }
  else {
    /* discard the received frame */
    TRACE_LOG_ERR("<CRC> invalid %d/%d", p_ctx->rxLastDataPtr, p_ctx->rxBytesCounter);
    rf_rx_term(p_ctx);
  }
}


/**
 * @brief handle event of a good packet being completely received in RX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_rx_fini(struct s_rf_ctx *p_ctx) {

  /* signal upper layer */
  e_nsErr_t err;
  rf_recv(p_ctx->rxBuf, p_ctx->rxBytesCounter, &err);
}


#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
/**
 * @brief handle event of ACK SYNC transmission in RX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_rx_txAckSync(struct s_rf_ctx *p_ctx) {

  p_ctx->state = RF_STATE_RX_TXACK_SYNC;

  /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* then switch to fixed packet length mode */
  rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->txDataLen);
#endif /* NETSTK_CFG_IEEE_802154G_EN */
}


/**
 * @brief handle event of ACK transmission in RX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_rx_txAckFini(struct s_rf_ctx *p_ctx) {

  p_ctx->state = RF_STATE_RX_TXACK_FINI;
  evproc_putEvent(E_EVPROC_HEAD, NETSTK_RF_EVENT, p_ctx);
}
#endif


/**
 * @brief handle event of reception termination in RX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_rx_term(struct s_rf_ctx *p_ctx) {

  /* set sub-state */
  p_ctx->state = RF_STATE_RX_TERM;

  /* exit RX state */
  rf_rx_exit(p_ctx);

  /* enter RX state */
  rf_rx_entry(p_ctx);
}


/**
 * @brief handle exit function of RX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_rx_exit(struct s_rf_ctx *p_ctx) {

  LED_RX_OFF();

  /* configure RF interrupt */
  RF_WR_REGS(&cc112x_cfg_iocfgOff[RF_IOCFG_RXFIFO_THR]);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* use infinite packet length mode by default */
  RF_SET_PKT_LEN_MODE(CC112X_PKT_LEN_MODE_INFINITE);
#endif /* NETSTK_CFG_IEEE_802154G_EN */
}


/**
 * @brief handle entry function in TX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_tx_entry(struct s_rf_ctx *p_ctx) {
  LED_TX_ON();
  p_ctx->txErr = NETSTK_ERR_NONE;
  p_ctx->txStatus = RF_TX_STATUS_NONE;
  p_ctx->txReqAck = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);
}


/**
 * @brief handle event of SYNC words transmitted in TX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_tx_sync(struct s_rf_ctx *p_ctx) {

  p_ctx->state = RF_STATE_TX_SYNC;
  rf_tx_entry(p_ctx);

  /* configure RF interrupt */
  //rf_intConfig(RF_IOCFG_TXFIFO_THR, RF_INT_TXFIFO_THR, RF_INT_EDGE_TXFIFO_THR, rf_txFifoThresholdISR);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  /* switch to fixed packet length mode if number of remaining bytes is less than FIFO SIZE
   * the actual number of remaining bytes to be sent is calculated as follow:
   * totNumRemBytes = numBytesInTxFifo + numRemBytes
   * numBytesInTxFifo = RF_CFG_NUM_TXBYTES
   */
  if ((p_ctx->pktLenMode == CC112X_PKT_LEN_MODE_INFINITE) &&
      (p_ctx->txDataLen < (RF_MAX_FIFO_LEN + 1))) {
    rf_setPktLen(CC112X_PKT_LEN_MODE_FIXED, p_ctx->txDataLen);
  }
#endif /* NETSTK_CFG_IEEE_802154G_EN */
}


/**
 * @brief handle event of a frame successfully transmitted in TX state
 * @param p_ctx   point to variable holding radio context structure
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
}


#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
/**
 * @brief handle event of ACK SYNC reception in TX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_tx_rxAckSync(struct s_rf_ctx *p_ctx) {

  p_ctx->state = RF_STATE_TX_RXACK_SYNC;

  /* configure RF interrupt */
  rf_intConfig(RF_IOCFG_RXFIFO_THR, RF_INT_RXFIFO_THR, RF_INT_EDGE_RXFIFO_THR, rf_rxFifoThresholdISR);
}


/**
 * @brief handle event of ACK reception in TX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_tx_rxAckFini(struct s_rf_ctx *p_ctx) {

  uint8_t isChecksumOK;
  uint8_t numChksumBytes;
  packetbuf_attr_t expSeqNo;

  /* read remaining bytes from RX FIFO */
  if (p_ctx->rxNumRemBytes) {
    rf_readRxFifo(p_ctx, p_ctx->rxNumRemBytes);
    p_ctx->rxNumRemBytes = 0;
  }

  /* update checksum */
  if (p_ctx->rxLastDataPtr > (p_ctx->rxFrame.crc_len + p_ctx->rxLastChksumPtr)) {
    numChksumBytes = p_ctx->rxLastDataPtr - (p_ctx->rxLastChksumPtr + p_ctx->rxFrame.crc_len);
    p_ctx->rxChksum = llframe_crcUpdate(&p_ctx->rxFrame, &p_ctx->rxBuf[p_ctx->rxLastChksumPtr], numChksumBytes, p_ctx->rxChksum);
    p_ctx->rxLastChksumPtr += numChksumBytes;
  }
  p_ctx->rxChksum = llframe_crcFinal(&p_ctx->rxFrame, p_ctx->rxChksum);

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
      p_ctx->txErr = NETSTK_ERR_TX_NOACK;

      /* bad ACK handling */
      rf_tx_term(p_ctx);
      rf_rx_entry(p_ctx);
    }
  }
  else {
    /* discard the received frame */
    p_ctx->txErr = NETSTK_ERR_TX_NOACK;

    /* bad ACK handling */
    rf_tx_term(p_ctx);
    rf_rx_entry(p_ctx);
  }

  /* indicate TX process has finished */
  p_ctx->txStatus = RF_TX_STATUS_DONE;
}
#endif


/**
 * @brief handle event of transmission termination in TX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_tx_term(struct s_rf_ctx *p_ctx) {
  uint8_t txLast;

  p_ctx->txStatus = RF_TX_STATUS_DONE;

  /* force radio to flush TXFIFO by manipulating TXFIFO pointers */
  cc112x_spiRegRead(CC112X_TXLAST, &txLast, 1);
  cc112x_spiRegWrite(CC112X_TXFIRST, &txLast, 1);
}


/**
 * @brief handle exit function of TX state
 * @param p_ctx   point to variable holding radio context structure
 */
static void rf_tx_exit(struct s_rf_ctx *p_ctx) {
  LED_TX_OFF();

  /* no need to change RF settings as ACK is missing because the radio is already put to RX_IDLE */
  if (p_ctx->txErr != NETSTK_ERR_TX_NOACK) {
    /* configure RF interrupt */
    RF_WR_REGS(&cc112x_cfg_iocfgOff[RF_IOCFG_RXFIFO_THR]);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
    /* use infinite packet length mode by default */
    RF_SET_PKT_LEN_MODE(CC112X_PKT_LEN_MODE_INFINITE);
#endif /* NETSTK_CFG_IEEE_802154G_EN */
  }

  p_ctx->txErr = NETSTK_ERR_NONE;
  p_ctx->txStatus = RF_TX_STATUS_NONE;
  p_ctx->txReqAck = FALSE;
}


/**
 * @brief radio state dispatcher function
 * @param c_event   event to handle
 * @param p_data    point to callback data
 */
static void rf_eventHandler(c_event_t c_event, p_data_t p_data) {
  (void)p_data;

  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* handle nested events */
  if (c_event == NETSTK_RF_EVENT) {
    if ((p_ctx->state == RF_STATE_RX_FINI) ||
        (p_ctx->state == RF_STATE_RX_TXACK_FINI)) {
      rf_rx_fini(p_ctx);
    }
  }
}


/**
 * @note  radio driver exception handler
 * @param p_ctx       point to variable holding radio context structure
 * @param marcStatus  value of MARC status at which exception was thrown
 * @param chipState   state of radio chip at which exception was thrown
 */
static void rf_exceptionHandler(struct s_rf_ctx *p_ctx, uint8_t marcStatus, e_rfState_t chipState) {

  /* terminate TX process */
  p_ctx->txErr = NETSTK_ERR_FATAL;
  rf_tx_term(p_ctx);

  /* terminate RX process */
  rf_rx_term(p_ctx);
}


/*
 ********************************************************************************
 *                               MISCELLANEOUS
 ********************************************************************************
 */

/**
 * @brief put the radio into IDLE state
 * @param p_ctx   point to variable holding radio context structure
 */
static uint8_t rf_gotoIdle(struct s_rf_ctx *p_ctx) {

  uint8_t isIdleOk = TRUE;

  /* wait until radio enters IDLE state */
  cc112x_spiCmdStrobe(CC112X_SIDLE);
  while (RF_READ_CHIP_STATE() != RF_STATE_IDLE) {
    /* do nothing */
    if (RF_IS_RX_BUSY() == TRUE) {
      trace_printf("<IDLE> RX while IDLE");
      isIdleOk = FALSE;
      break;
    }
  };

  if (isIdleOk == TRUE) {
    /* flushing TXFIFO and RXFIFO */
    cc112x_spiCmdStrobe(CC112X_SFTX);
    cc112x_spiCmdStrobe(CC112X_SFRX);
  }

  return isIdleOk;
}


/**
 * @brief put the radio into idle listening state
 * @param p_ctx   point to variable holding radio context structure
 */
static uint8_t rf_gotoRx(struct s_rf_ctx *p_ctx) {

  uint8_t isIdleOk;

  /* did the transceiver enter Idle state successfully? */
  isIdleOk = rf_gotoIdle(p_ctx);
  if (isIdleOk == TRUE) {
    /* disable RX termination in RX mode */
    if (p_ctx->cfgWOREnabled == TRUE) {
      uint8_t rfendCfg0;
      /* read value of registers to modify */
      cc112x_spiRegRead(CC112X_RFEND_CFG0, &rfendCfg0, 1);
      /* disable RX termination based on CS */
      rfendCfg0 &= ~0x09;
      cc112x_spiRegWrite(CC112X_RFEND_CFG0, &rfendCfg0, 1);
    }

    /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
    /* then switch back to infinite packet length mode */
    rf_setPktLen(CC112X_PKT_LEN_MODE_INFINITE, RF_MAX_FIFO_LEN);
#endif

    /* configure RF interrupt */
    rf_intConfig(RF_IOCFG_RXFIFO_THR, RF_INT_RXFIFO_THR, RF_INT_EDGE_RXFIFO_THR, rf_rxFifoThresholdISR);

    /* wait until radio enters RX state */
    cc112x_spiCmdStrobe(CC112X_SRX);
    while (RF_READ_CHIP_STATE() != RF_STATE_RX_IDLE) {
      /* do nothing */
    }
  }
  return isIdleOk;
}


/**
 * @brief put the radio into WOR state
 * @param p_ctx   point to variable holding radio context structure
 */
static uint8_t rf_gotoWor(struct s_rf_ctx *p_ctx) {

  uint8_t isIdleOk;

  p_ctx->numSniffs = 0;
  p_ctx->numPqtReached = 0;

  /* did the transceiver enter Idle state successfully? */
  isIdleOk = rf_gotoIdle(p_ctx);
  if (isIdleOk == TRUE) {
    /* Setting radio registers must be done in IDLE state when WOR is enabled */

    /* enable RX termination in WOR mode */
    if (p_ctx->cfgWOREnabled == TRUE) {
      uint8_t rfendCfg0;
      /* read value of registers to modify */
      cc112x_spiRegRead(CC112X_RFEND_CFG0, &rfendCfg0, 1);
      /* enable RX termination based on CS */
      rfendCfg0 |= 0x09;
      cc112x_spiRegWrite(CC112X_RFEND_CFG0, &rfendCfg0, 1);
    }

    /* is IEEE Std. 802.15.4g supported? */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
    /* then switch back to infinite packet length mode */
    rf_setPktLen(CC112X_PKT_LEN_MODE_INFINITE, RF_MAX_FIFO_LEN);
#endif

    /* configure RF interrupt */
    rf_intConfig(RF_IOCFG_RXFIFO_THR, RF_INT_RXFIFO_THR, RF_INT_EDGE_RXFIFO_THR, rf_rxFifoThresholdISR);

    /* WOR mode */
    cc112x_spiCmdStrobe(CC112X_SWOR);
  }
  return isIdleOk;
}


/**
 * @brief put the radio into idle listening
 * @param p_ctx point to variable holding radio context structure
 */
static void rf_listen(struct s_rf_ctx *p_ctx) {

  if (p_ctx->cfgWOREnabled == TRUE) {
    /* eWOR mode */
    rf_gotoWor(p_ctx);
  } else {
    /* RX mode */
    rf_gotoRx(p_ctx);
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
static uint8_t rf_readRxFifo(struct s_rf_ctx *p_ctx, uint8_t numBytes) {

  /* check buffer overflow */
  if ((p_ctx->rxLastDataPtr + numBytes) > RF_MAX_PACKET_LEN) {
    return 0;
  }

  /* read a given number of bytes from RX FIFO and append to RX buffer */
  cc112x_spiRxFifoRead(&p_ctx->rxBuf[p_ctx->rxLastDataPtr], numBytes);
  p_ctx->rxLastDataPtr += numBytes;
  return numBytes;
}


/**
 * @brief   Write a given number of bytes to TXFIFO
 * @param p_ctx         point to variable holding radio context structure
 * @param totNumBytes   total number of bytes to write
 * @param p_err         point to variable storing returned error code
 */
static void rf_writeTxFifo(struct s_rf_ctx *p_ctx, uint8_t totNumBytes, e_nsErr_t *p_err) {

  uint8_t numWrBytes;

  /* set return error code */
  *p_err = NETSTK_ERR_NONE;

  /* write small chunk of data at a time to reduce long continuous critical section due to SPI operations */
  while (totNumBytes > 0) {
    if (totNumBytes > RF_CFG_TX_FIFO_THR) {
      numWrBytes = RF_CFG_TX_FIFO_THR;
    }
    else {
      numWrBytes = totNumBytes;
    }

    cc112x_spiTxFifoWrite(p_ctx->txDataPtr, numWrBytes);
    p_ctx->txNumRemBytes -= numWrBytes;
    p_ctx->txDataPtr += numWrBytes;
    totNumBytes -= numWrBytes;
  }
}


/**
 * @brief   Configure radio interrupt
 * @param iocfg     radio GPIO pin
 * @param e_extInt  related MCU interrupt
 * @param e_edge    interrupt edge to detect
 * @param cbfnct    callback function
 */
static void rf_intConfig(uint8_t iocfg, en_targetExtInt_t e_extInt, en_targetIntEdge_t e_edge,
    pfn_intCallb_t cbfnct) {

  RF_WR_REGS(&cc112x_cfg_iocfgOn[iocfg]);
  bsp_extIntRegister(e_extInt, e_edge, cbfnct);
  bsp_extIntClear(e_extInt);
  bsp_extIntEnable(e_extInt);
}


#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
/**
 * @brief set packet length mode and packet size
 * @param mode    packet length mode to set
 * @param len     packet length
 */
static void rf_setPktLen(uint8_t mode, uint16_t len) {

  uint8_t pktLen;
  struct s_rf_ctx *p_ctx = &rf_ctx;

  /* set packet length */
  pktLen = len % (RF_MAX_FIFO_LEN + 1);
  cc112x_spiRegWrite(CC112X_PKT_LEN, &pktLen, 1);

  /* set packet length mode */
  RF_SET_PKT_LEN_MODE(mode);
  p_ctx->pktLenMode = mode;
}
#endif


#if (NETSTK_CFG_RF_SW_AUTOACK_EN == TRUE)
/**
 * @brief   Perform address filtering
 * @param p_ctx     point to variable holding radio context structure
 * @return  TRUE if passed, otherwise FALSE
 */
static uint8_t rf_filterAddr(struct s_rf_ctx *p_ctx) {

  uint8_t ret = TRUE;
  uint8_t ack_len = 0;
  uint8_t ack[10] = { 0 };

  /* filter address */
  p_ctx->rxIsAddrFiltered = llframe_addrFilter(&p_ctx->rxFrame, p_ctx->rxBuf, p_ctx->rxLastDataPtr);
  if (p_ctx->rxIsAddrFiltered == FALSE) {
    ret = FALSE;
  }
  else {
    /* does incoming frame require ACK? */
    p_ctx->rxReqAck = p_ctx->rxFrame.is_ack_required;
    if (p_ctx->rxReqAck == TRUE) {
      /* write the ACK to send into TX FIFO */
      ack_len = llframe_createAck(&p_ctx->rxFrame, ack, sizeof(ack));
      cc112x_spiTxFifoWrite(ack, ack_len);

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
      /* store length of ACK to send for later setting packet length modes */
      p_ctx->txDataLen = ack_len;
#endif /* NETSTK_CFG_IEEE_802154G_EN */
    }
  }
  return ret;
}
#endif /* NETSTK_CFG_IEEE_802154G_EN */


/**
 * @brief   Manual calibrate radio
 */
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


/**
 * @brief   Calibrate RC oscillator
 */
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


/**
 * @brief   Perform Clear Channel Assessment
 * @param p_err   point to variable storing returned error code
 */
static void rf_cca(struct s_rf_ctx *p_ctx, e_nsErr_t *p_err) {

#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  uint8_t rssi_status;
  uint32_t tickStart;
  uint32_t ccaTimeout;

  if (RF_IS_RX_BUSY() == TRUE) {
    *p_err = NETSTK_ERR_BUSY;
    return;
  }

  /* put radio back to RX state if eWOR is enabled */
  if (p_ctx->cfgWOREnabled == TRUE) {
    rf_gotoRx(p_ctx);
    if (RF_IS_RX_BUSY() == TRUE) {
      /* radio is in middle of packet reception process */
      *p_err = NETSTK_ERR_BUSY;
      return;
    }
  }

  /* poll for valid carrier sense until timeout */
  tickStart = bsp_getTick();

  /* since atomic time unit is 1ms, ensure CCA timeout is at least 1ms */
  do {
    /* verify if either carrier sense is valid? */
    cc112x_spiRegRead( CC112X_RSSI0, &rssi_status, 1);
    if ((rssi_status & RF_RSSI0_CARRIER_SENSE_VALID ) != 0) {
      break;
    }

    /* is a packet being received? */
    if (RF_IS_RX_BUSY() == TRUE) {
      *p_err = NETSTK_ERR_BUSY;
      return;
    }

    /* update CCA timeout */
    ccaTimeout = bsp_getTick() - tickStart;
  } while (ccaTimeout < 2);

  /* was CCA timeout expired? If so carrier sense is invalid */
  if ((rssi_status & RF_RSSI0_CARRIER_SENSE_VALID ) == 0) {
    /* then a runtime error was detected. The driver is still in RX_IDLE but the radio
     * did not give a valid carrier sense in time. In this case simply force the radio
     * to RX_IDLE and declare channel access failure */
    rf_rx_term(p_ctx);
    *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
  }
  else {
    /* was carrier detected? */
    if (rssi_status & RF_RSSI0_CARRIER_DETECTED) {
      /* then declare channel busy */
      *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
    }
    else {
      /* declare channel free */
      *p_err = NETSTK_ERR_NONE;

      /* then go back to eWOR when enabled */
      if (p_ctx->cfgWOREnabled == TRUE) {
        rf_gotoWor(p_ctx);
      }
    }
  }
}


static void rf_chkReset(struct s_rf_ctx *p_ctx) {

  e_nsErr_t err;
  uint8_t read_byte;

  /* check one of the registers */
  cc112x_spiRegRead(CC112X_FREQ2, &read_byte, 1);

  if (read_byte != p_ctx->regVerify) {
    /* re-configure radio transceiver without re-initializing driver configuration attributes */
    RF_INT_DISABLED();

    /* initialize SPI handle */
    cc112x_spiInit();

    /* reset the transceiver. Afterwards the chip will be in IDLE state */
    rf_reset();

    /* configure RF registers */
    RF_WR_REGS(cc112x_cfg_ieee802154g_default);
    RF_SET_FIFO_THR(RF_CFG_FIFO_THR);

    /* calibrate radio according to cc112x errata */
    rf_manualCalibration();

    /* calibrate RC oscillator */
    rf_calibrateRCOsc();

    /* configurations of radio interrupts */
    RF_INT_CONFIG();

    /* re-configure operation frequency */
    rf_opModeSet(p_ctx->cfgOpMode, &err);
    rf_chanNumSet(p_ctx->cfgFreqChanNum, &err);
    TRACE_LOG_ERR("radio transceiver was reset, opMode=%d, chanNum=%d", p_ctx->cfgOpMode, p_ctx->cfgFreqChanNum);
  }
}


/**
 * @brief   Reset radio transceiver
 */
static void rf_reset(void) {

  /* by issuing a manual reset, all internal registers are set to their default
  * values and the radio will go to the IDLE state
  */
  cc112x_spiCmdStrobe(CC112X_SRES);

  /* wait for the crystal oscillator to stabilize */
  rf_waitRdy();
}


/**
 * @brief   Check part number and part version of the radio chip
 * @param p_err   point to variable storing returned error code
 */
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


/**
 * @brief   wait until radio chip is ready
 */
static void rf_waitRdy(void) {

  rf_status_t chip_status;
  do {
    chip_status = cc112x_spiCmdStrobe(CC112X_SNOP);
  } while (chip_status & CC112X_STATE_CHIP_RDYn);
}


/**
 * @brief   set transceiver transmission power
 * @param power   transmission power to set
 * @param p_err   point to variable storing returned error code
 */
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
  cc112x_spiRegWrite(CC112X_PA_CFG2, &pa_power_ramp, 1);
  *p_err = NETSTK_ERR_NONE;
}

/**
 * @brief   obtain transceiver transmission power
 * @param p_power point to variable storing transmission power
 * @param p_err   point to variable storing returned error code
 */
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
  cc112x_spiRegRead(CC112X_PA_CFG2, &pa_power_ramp, 1);
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
  e_rfState_t prevState;
  struct s_rf_ctx *p_ctx = &rf_ctx;

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
    /* backup current state */
    prevState = p_ctx->state;
    if (prevState == RF_STATE_SLEEP) {
      rf_sleep_exit(p_ctx);
    }
    else {
      rf_on_exit(p_ctx);
    }

    /* set operation frequency */
    freq_reg = freq_center + rf_ctx.cfgFreqChanNum * freq_delta;

    write_byte = (freq_reg & 0x00FF0000) >> 16;
    cc112x_spiRegWrite(CC112X_FREQ2, &write_byte, 1);
    p_ctx->regVerify = write_byte;

    write_byte = (freq_reg & 0x0000FF00) >> 8;
    cc112x_spiRegWrite(CC112X_FREQ1, &write_byte, 1);

    write_byte = (freq_reg & 0x000000FF);
    cc112x_spiRegWrite(CC112X_FREQ0, &write_byte, 1);

    /* frequency calibration */
    rf_manualCalibration();

    /* restore previous state */
    if (prevState == RF_STATE_SLEEP) {
      rf_sleep_entry(p_ctx);
    }
    else {
      rf_on_entry(p_ctx);
    }
  }
}

/**
 * @brief   Select operating mode
 * @param   mode        Operating mode to select
 * @param   p_err       Pointer to variable holding returned error code
 */
static void rf_opModeSet(e_nsRfOpMode mode, e_nsErr_t *p_err) {
  uint8_t write_byte;

  if (rf_ctx.cfgOpMode < NETSTK_RF_OP_MODE_MAX) {
    rf_ctx.cfgOpMode = mode;

    /* change symbol rate and RX filter bandwidth accordingly */
    if ((mode == NETSTK_RF_OP_MODE_CSM) ||
        (mode == NETSTK_RF_OP_MODE_1)) {
      /* symbol rate of 50kbps: 99 99 99 */
      write_byte = 0x99;
      cc112x_spiRegWrite(CC112X_SYMBOL_RATE2, &write_byte, 1);
      cc112x_spiRegWrite(CC112X_SYMBOL_RATE1, &write_byte, 1);
      cc112x_spiRegWrite(CC112X_SYMBOL_RATE0, &write_byte, 1);

      /* change RX filter bandwidth 100 kHz: 02 */
      write_byte = 0x02;
      cc112x_spiRegWrite(CC112X_CHAN_BW, &write_byte, 1);
    }
    else {
      /* MR-FSK mode #2 or #3 */
      /* symbol rate of 100kbps: A9 99 9A  */
      write_byte = 0xA9;
      cc112x_spiRegWrite(CC112X_SYMBOL_RATE2, &write_byte, 1);
      write_byte = 0x99;
      cc112x_spiRegWrite(CC112X_SYMBOL_RATE1, &write_byte, 1);
      write_byte = 0x9A;
      cc112x_spiRegWrite(CC112X_SYMBOL_RATE0, &write_byte, 1);

      /* change RX filter bandwidth 200 kHz: 01 */
      write_byte = 0x01;
      cc112x_spiRegWrite(CC112X_CHAN_BW, &write_byte, 1);
    }
    *p_err = NETSTK_ERR_NONE;
  }
  else {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
  }
}


/**
 * @brief   Read Receiver Signal Strength Indicator
 * @param p_val   point to variable storing RSSI value to read
 * @param p_err   point to variable storing returned error code
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


