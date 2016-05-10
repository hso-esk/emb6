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

#define  LOGGER_ENABLE        LOGGER_RADIO
#include "logger.h"

/*
 ********************************************************************************
 *                                LOCAL TYPEDEFS
 ********************************************************************************
 */
typedef enum e_rf_state {
  RF_STATE_NON_INIT,
  RF_STATE_INIT,
  RF_STATE_SLEEP,
  RF_STATE_ERR,
  RF_STATE_IDLE,

  /* RX Submachine states */
  RF_STATE_RX,
  RF_STATE_RX_SYNC,
  RF_STATE_RX_PORTION_MIDDLE,
  RF_STATE_RX_PORTION_LAST,
  RF_STATE_RX_FINI,

  /* TX Submachine states */
  RF_STATE_TX_STARTED,
  RF_STATE_TX_BUSY,
  RF_STATE_TX_FINI,
  RF_STATE_TX_PORTION_MIDDLE,
  RF_STATE_TX_PORTION_LAST,

  /* CCA Submachine states */
  RF_STATE_CCA_BUSY,
  RF_STATE_CCA_FINI,

} e_rfState_t;

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

#define RF_CFG_MAX_PACKET_LENGTH            (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)

#define RF_IS_IN_TX(_chip_status)           ((_chip_status) & 0x20)
#define RF_IS_IN_RX(_chip_status)           ((_chip_status) & 0x10)

#define RF_CCA_MODE_NONE                    (uint8_t)( 0x00 )
#define RF_CCA_MODE_RSSI_BELOW_THR          (uint8_t)( 0x24 )

#define RF_CHIP_STATE_IDLE                  (uint8_t)( 0x00 )
#define RF_CHIP_STATE_RX                    (uint8_t)( 0x10 )
#define RF_CHIP_STATE_TX                    (uint8_t)( 0x20 )
#define RF_CHIP_STATE_FSTXON                (uint8_t)( 0x30 )
#define RF_CHIP_STATE_CALIBRATE             (uint8_t)( 0x40 )
#define RF_CHIP_STATE_SETTLING              (uint8_t)( 0x50 )
#define RF_CHIP_STATE_RX_FIFO_ERR           (uint8_t)( 0x60 )
#define RF_CHIP_STATE_TX_FIFO_ERR           (uint8_t)( 0x70 )

#define RF_MARC_STATUS_NO_FAILURE           (uint8_t)( 0x00 )
#define RF_MARC_STATUS_TX_FINI              (uint8_t)( 0x40 )
#define RF_MARC_STATUS_RX_FINI              (uint8_t)( 0x80 )

#define RF_MARC_STATUS_TX_OVERFLOW          (uint8_t)( 0x07 )
#define RF_MARC_STATUS_TX_UNDERFLOW         (uint8_t)( 0x08 )
#define RF_MARC_STATUS_RX_OVERFLOW          (uint8_t)( 0x09 )
#define RF_MARC_STATUS_RX_UNDERFLOW         (uint8_t)( 0x0A )

#define RF_RSSI0_CARRIER_SENSE_VALID        (uint8_t)( 0x02 )
#define RF_RSSI0_CARRIER_DETECTED           (uint8_t)( 0x04 )

#define RF_RSSI_VALID_MSK                   (0x01)
#define RF_RSSI_LOW_LEN                     ( 4 )
#define RF_RSSI_LOW_OFST                    ( 3 )
#define RF_RSSI_OFFSET                      ( 102 )
#define RF_RSSI_RES                         ( 0.0625 )


#define RF_GET_CHIP_STATE(_chip_status)     (uint8_t)((_chip_status) & 0x70)

/*!< Disable all RF interrupts */
#define RF_INT_DISABLED()  \
    do {    \
        bsp_extIntDisable(RF_INT_CFG_RX_SYNC);  \
        bsp_extIntDisable(RF_INT_CFG_RX_FINI);  \
        bsp_extIntDisable(RF_INT_CFG_TX_FINI);  \
    } while (0)

#define RF_INT_TX_ENABLED() \
    do {    \
        bsp_extIntClear(RF_INT_CFG_TX_FINI);    \
        bsp_extIntEnable(RF_INT_CFG_TX_FINI);   \
    } while(0)

#define RF_INT_RX_ENABLED() \
    do {    \
        bsp_extIntClear(RF_INT_CFG_RX_SYNC);    \
        bsp_extIntClear(RF_INT_CFG_RX_FINI);    \
        bsp_extIntEnable(RF_INT_CFG_RX_SYNC);   \
        bsp_extIntEnable(RF_INT_CFG_RX_FINI);   \
    } while(0)

#define RF_CHIP_STATUS()  \
  do {  \
    uint8_t status_; \
    status_ = cc112x_spiCmdStrobe(CC112X_SNOP);     \
    TRACE_LOG_MAIN("RF_STATUS: cs=%02x", status_);  \
  } while(0)

#define RF_WR_REGS(regs_) \
  do {  \
    uint8_t len_; \
    len_ = sizeof((regs_)) / sizeof(s_regSettings_t);   \
    cc112x_configureRegs((regs_), len_);                \
  } while(0)

/*!< check if RF is in one of reception states */
#define RF_IS_RX_BUSY() \
    ((rf_state == RF_STATE_RX_SYNC)             ||  \
     (rf_state == RF_STATE_RX_FINI))

#define RF_INT_CFG_RX_SYNC                  E_TARGET_EXT_INT_0
#define RF_INT_CFG_RX_FINI                  E_TARGET_EXT_INT_1
#define RF_INT_CFG_TX_FINI                  E_TARGET_EXT_INT_2

#define RF_INT_CFG_EDGE_RX_SYNC             E_TARGET_INT_EDGE_RISING
#define RF_INT_CFG_EDGE_RX_FINI             E_TARGET_INT_EDGE_FALLING
#define RF_INT_CFG_EDGE_TX_FINI             E_TARGET_INT_EDGE_FALLING


/*
 ********************************************************************************
 *                                LOCAL VARIABLES
 ********************************************************************************
 */
static s_ns_t *prf_netstk;
static uint8_t rf_rxBuf[RF_CFG_MAX_PACKET_LENGTH ];
static uint8_t rf_rxBufLen;
static e_rfState_t volatile rf_state = RF_STATE_NON_INIT;
static uint8_t rf_worEn;
static uint8_t rf_chanNum;
static e_nsRfOpMode rf_opMode;

static uint8_t rf_txFirstPtr;


/*
 ********************************************************************************
 *                           LOCAL FUNCTIONS DECLARATION
 ********************************************************************************
 */
static void cc112x_Init(void *p_netstk, e_nsErr_t *p_err);
static void cc112x_On(e_nsErr_t *p_err);
static void cc112x_Off(e_nsErr_t *p_err);
static void cc112x_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void cc112x_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err);
static void cc112x_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static void cc112x_isrRxSyncReceived(void *p_arg);
static void cc112x_isrRxPacketReceived(void *p_arg);
static void cc112x_isrTxPacketSent(void *p_arg);

static void cc112x_eventHandler(c_event_t c_event, p_data_t p_data);
static void cc112x_rxFifoErrHandler(void);

static void cc112x_configureRegs(const s_regSettings_t *p_regs, uint8_t len);
static void cc112x_manualCalibration(void);
static void cc112x_calibrateRCOsc(void);
static void cc112x_cca(e_nsErr_t *p_err);
static void cc112x_retx(e_nsErr_t *p_err);

static void cc112x_reset(void);
static void cc112x_chkPartnumber(e_nsErr_t *p_err);
static void cc112x_waitRdy(void);
static void cc112x_gotoSleep(void);
static void cc112x_gotoRX(void);
static void cc112x_gotoIdle(void);
static void cc112x_txPowerSet(int8_t power, e_nsErr_t *p_err);
static void cc112x_txPowerGet(int8_t *p_power, e_nsErr_t *p_err);
static void cc112x_chanNumSet(uint8_t chan_num, e_nsErr_t *p_err);
static void cc112x_opModeSet(e_nsRfOpMode mode, e_nsErr_t *p_err);
static void cc112x_readRSSI(int16_t *p_val, e_nsErr_t *p_err);
static void cc112x_selfTest(e_nsErr_t *p_err);

/*
********************************************************************************
*                           LOCAL FUNCTIONS DEFINITIONS
********************************************************************************
*/
static void cc112x_Init(void *p_netstk, e_nsErr_t *p_err)
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


  /* indicates radio is in state initialization */
  rf_state = RF_STATE_INIT;

  /* store pointer to global netstack structure */
  prf_netstk = (s_ns_t *)p_netstk;

  /* initialize SPI handle */
  cc112x_spiInit();

  /* reset the transceiver. Afterwards the chip will be in IDLE state */
  cc112x_reset();

  /* check part number */
  cc112x_chkPartnumber(p_err);
  if (*p_err != NETSTK_ERR_NONE) {
    emb6_errorHandler(p_err);
  }

  /* configure RF registers */
  RF_WR_REGS(cc112x_cfg_ieee802154g_default);

  /* calibrate radio according to cc112x errata */
  cc112x_manualCalibration();

  /* calibrate RC oscillator */
  cc112x_calibrateRCOsc();

  /* configure interrupts */
  bsp_extIntRegister(RF_INT_CFG_TX_FINI, RF_INT_CFG_EDGE_TX_FINI, cc112x_isrTxPacketSent);
  bsp_extIntRegister(RF_INT_CFG_RX_SYNC, RF_INT_CFG_EDGE_RX_SYNC, cc112x_isrRxSyncReceived);
  bsp_extIntRegister(RF_INT_CFG_RX_FINI, RF_INT_CFG_EDGE_RX_FINI, cc112x_isrRxPacketReceived);

  /* initialize local variables */
  evproc_regCallback(NETSTK_RF_EVENT, cc112x_eventHandler);
  memset(rf_rxBuf, 0, sizeof(rf_rxBuf));
  rf_rxBufLen = 0;
  rf_worEn = TRUE; /* enable WOR mode by default */

  /* configure operating mode and channel number */
  rf_chanNum = 0;
  rf_opMode = NETSTK_RF_OP_MODE_CSM;

  /* put radio to IDLE state */
  cc112x_gotoIdle();
}

static void cc112x_On(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  /* put the radio to the IDLE state after waking up */
  if (rf_state == RF_STATE_SLEEP) {
    cc112x_gotoIdle();
  }

  /* go to state RX */
  cc112x_gotoRX();

  /* indicate successful operation */
  *p_err = NETSTK_ERR_NONE;
}

static void cc112x_Off(e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  /* force the radio to go to the IDLE state */
  cc112x_gotoIdle();

  /* then put it to SLEEP state */
  cc112x_gotoSleep();

  /* indicate successful operation */
  *p_err = NETSTK_ERR_NONE;
}

static void cc112x_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((p_data == NULL) || (len == 0) || (len > RF_CFG_MAX_PACKET_LENGTH)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

#if (EMB6_CFG_CONTINUOUS_RX == TRUE)
  *p_err = NETSTK_ERR_BUSY;
  return;
#endif

  if ((rf_state != RF_STATE_RX) &&
      (rf_state != RF_STATE_IDLE)) {
    *p_err = NETSTK_ERR_BUSY;
  } else {
    uint8_t num_txbytes, chip_status;

    /* entry:
    * - go to the IDLE state
    * - disable RX interrupt
    */
    RF_INT_DISABLED();
    cc112x_gotoIdle();
    cc112x_waitRdy();

    /* do:
    * - go to the TX_BUSY state
    * - write packet to TX FIFO
    * - verify number of bytes in TX FIFO
    * - enable TX interrupts
    * - issue STX strobe
    */
    rf_state = RF_STATE_TX_BUSY;
    cc112x_spiTxFifoWrite(p_data, len);
    chip_status = cc112x_spiRegRead(CC112X_NUM_TXBYTES, &num_txbytes, 1);
    if (num_txbytes != len) {
      TRACE_LOG_ERR("RF_TX: failed to write to TX FIFO %d / %d / %02x", num_txbytes, len, chip_status);
    }

    /* store TX first pointer for retransmission when required */
    cc112x_spiRegRead(CC112X_TXFIRST, &rf_txFirstPtr, 1);

    LED_TX_ON();
    RF_INT_TX_ENABLED();
    cc112x_spiCmdStrobe(CC112X_STX);

    /* wait for packet to be sent */
    uint16_t iteration = 0xffff;
    while ((rf_state == RF_STATE_TX_BUSY) && (iteration > 0)) {
      iteration--;
    }
    if (rf_state == RF_STATE_TX_FINI) {
      /* TX finished successfully */
      *p_err = NETSTK_ERR_NONE;
    } else {
      /* TX error handling */
      *p_err = NETSTK_ERR_TX_TIMEOUT;
    }

    /* exit:
    * - force the radio to flush TX FIFO
    * - go to state RX
    */
    LED_TX_OFF();
    cc112x_gotoIdle();
    cc112x_gotoRX();
  }
}


static void cc112x_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

}

static void cc112x_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    case NETSTK_CMD_RF_TXPOWER_SET:
      cc112x_txPowerSet(*((int8_t *) p_val), p_err);
      break;

    case NETSTK_CMD_RF_TXPOWER_GET:
      cc112x_txPowerGet(p_val, p_err);
      break;

    case NETSTK_CMD_RF_CCA_GET:
      cc112x_cca(p_err);
      break;

    case NETSTK_CMD_RF_RETX:
      /* retransmit the last frame
      * - write TxFirst to the previous value
      */
      cc112x_retx(p_err);
      break;

    case NETSTK_CMD_RF_IS_RX_BUSY:
      if (RF_IS_RX_BUSY() == TRUE) {
        *p_err = NETSTK_ERR_BUSY;
      }
      break;


    case NETSTK_CMD_RF_CHAN_NUM_SET:
      cc112x_chanNumSet(*((uint8_t *) p_val), p_err);
      break;

    case NETSTK_CMD_RF_OP_MODE_SET:
      cc112x_opModeSet(*((e_nsRfOpMode *) p_val), p_err);
      break;

    case NETSTK_CMD_RF_WOR_EN:
      if (p_val) {
        rf_worEn = *((uint8_t *) p_val);
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
      cc112x_eventHandler(NETSTK_RF_EVENT, NULL);
      break;

    case NETSTK_CMD_RF_RSSI_GET:
      cc112x_readRSSI(p_val, p_err);
      break;

    case NETSTK_CMD_RF_SELFTEST:
      cc112x_selfTest(p_err);
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
*                           STATE TRANSITION HANDLERS
********************************************************************************
*/
static void cc112x_gotoSleep(void) {
  /* disable RF external interrupts */
  RF_INT_DISABLED();

  /* enter state Sleep */
  rf_state = RF_STATE_SLEEP;
  cc112x_spiCmdStrobe(CC112X_SPWD);
}


static void cc112x_gotoRX(void) {
  if (rf_state == RF_STATE_IDLE) {
    /* disable TX interrupt, enable RX interrupts */
    RF_INT_DISABLED();
    RF_INT_RX_ENABLED();

    /* when eWOR strobe is sent on SPI, the radio will go to the SLEEP state when
    * CSn is released.
    */
#if (NETSTK_CFG_WOR_EN == TRUE)
    rf_state = RF_STATE_RX;
    cc112x_spiCmdStrobe(CC112X_SWOR);
#else
    uint8_t chip_status;
    cc112x_spiCmdStrobe(CC112X_SRX);
    if (rf_state == RF_STATE_IDLE) {
      /* wait until PLL settling is complete */
      do {
        chip_status = cc112x_spiCmdStrobe(CC112X_SNOP);
      } while (RF_GET_CHIP_STATE(chip_status) == RF_CHIP_STATE_SETTLING);
    }

    chip_status = cc112x_spiCmdStrobe(CC112X_SNOP);
    assert_equ("GotoRx: ", RF_CHIP_STATE_RX, RF_GET_CHIP_STATE(chip_status));
    rf_state = RF_STATE_RX;
#endif
  }
}


static void cc112x_gotoIdle(void) {
  uint8_t chip_status;

  /* the SIDLE strobe command can always be used to force the radio to go to
  * the IDLE state
  */
  if (rf_state != RF_STATE_IDLE) {
    rf_state = RF_STATE_IDLE;
    chip_status = cc112x_spiCmdStrobe(CC112X_SIDLE);
    /* wait until the radio is in IDLE state */
    while (RF_GET_CHIP_STATE(chip_status) != RF_CHIP_STATE_IDLE) {
      chip_status = cc112x_spiCmdStrobe(CC112X_SIDLE);
    }
  }
}


static void cc112x_reset(void) {
  /* by issuing a manual reset, all internal registers are set to their default
  * values and the radio will go to the IDLE state
  */
  cc112x_spiCmdStrobe(CC112X_SRES);

  /* wait for the crystal oscillator to stabilize */
  cc112x_waitRdy();
}

static void cc112x_chkPartnumber(e_nsErr_t *p_err) {
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


static void cc112x_waitRdy(void) {
  rf_status_t chip_status;
  do {
    chip_status = cc112x_spiCmdStrobe(CC112X_SNOP);
  } while (chip_status & CC112X_STATE_CHIP_RDYn);
}

/*
********************************************************************************
*                       INTERRUPT SUBROUTINE HANDLERS
********************************************************************************
*/
static void cc112x_isrRxSyncReceived(void *p_arg) {
  uint8_t marc_status;
  uint8_t chip_status;

  /* avoid compiler warning of unused parameters */
  (void) &p_arg;
  TRACE_LOG_ENTER_ISR();

  /* clear ISR flag */
  bsp_extIntClear(RF_INT_CFG_RX_SYNC);

  /* achieve MARC_STATUS to determine what caused the interrupt */
  chip_status = cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);
  switch (marc_status) {
    case RF_MARC_STATUS_RX_OVERFLOW:
    case RF_MARC_STATUS_RX_UNDERFLOW:
      TRACE_LOG_ERR("RF_RX_SYNC: FIFO ERR marc=%02x, cs=%02x, state=%d", marc_status, chip_status, rf_state);
      cc112x_rxFifoErrHandler();
      break;

    case RF_MARC_STATUS_NO_FAILURE:
      if (rf_state == RF_STATE_RX) {
        /* go to state RX SYCN */
        rf_state = RF_STATE_RX_SYNC;
        LED_RX_ON();
      }
      break;

    case RF_MARC_STATUS_RX_FINI:
      /* reception has finished successfully and the radio will go to the
      * state indicated by RFEND_CFG1.RXOFF_MODE setting (default: IDLE)
      */
      rf_state = RF_STATE_RX_FINI;

      /* read RX FIFO  */
      cc112x_spiRegRead(CC112X_NUM_RXBYTES, &rf_rxBufLen, 1);
      cc112x_spiRxFifoRead(rf_rxBuf, rf_rxBufLen);
      LED_RX_OFF();
      TRACE_LOG_MAIN("RF_RX: RX FIFO len=%d; seq=%02x;", rf_rxBufLen, rf_rxBuf[3]);

      /* signal complete reception interrupt */
      evproc_putEvent(E_EVPROC_HEAD, NETSTK_RF_EVENT, NULL);
      break;

    default:
      TRACE_LOG_ERR("RF_RX_SYNC: unexpected marc=%02x, cs=%02x, state=%d", marc_status, chip_status, rf_state);
      break;
  }
  TRACE_LOG_INT("RF_RX_SYNC: marc=%02x, cs=%02x, state=%d", marc_status, chip_status, rf_state);
  TRACE_LOG_EXIT_ISR();
}


static void cc112x_isrRxPacketReceived(void *p_arg) {
  uint8_t marc_status;
  uint8_t chip_status;

  /* avoid compiler warning of unused parameters */
  (void) &p_arg;
  TRACE_LOG_ENTER_ISR();

  /* clear ISR flag */
  bsp_extIntClear(RF_INT_CFG_RX_FINI);

  /* achieve MARC_STATUS to determine what caused the interrupt */
  chip_status = cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);
  switch (marc_status) {
    case RF_MARC_STATUS_RX_FINI:
    //case RF_MARC_STATUS_NO_FAILURE:
      if (rf_state == RF_STATE_RX_SYNC) {
        /* reception has finished successfully and the radio will go to the
        * state indicated by RFEND_CFG1.RXOFF_MODE setting (default: IDLE)
        */
        rf_state = RF_STATE_RX_FINI;

        /* read RX FIFO  */
        cc112x_spiRegRead(CC112X_NUM_RXBYTES, &rf_rxBufLen, 1);
        cc112x_spiRxFifoRead(rf_rxBuf, rf_rxBufLen);
        LED_RX_OFF();
        TRACE_LOG_MAIN("RF_RX: RX FIFO len=%d; seq=%02x;", rf_rxBufLen, rf_rxBuf[3]);

        /* signal complete reception interrupt */
        evproc_putEvent(E_EVPROC_HEAD, NETSTK_RF_EVENT, NULL);
      }
      break;

    case RF_MARC_STATUS_RX_OVERFLOW:
    case RF_MARC_STATUS_RX_UNDERFLOW:
      TRACE_LOG_ERR("RF_RX_FINI: FIFO ERR marc=%02x, cs=%02x, state=%d", marc_status, chip_status, rf_state);
      cc112x_rxFifoErrHandler();
      break;

    default: {
      if (rf_state == RF_STATE_RX_SYNC) {
        /* bad packet is received, then try to read data from RX FIFO as for
        * good packet
        */
        rf_state = RF_STATE_RX_FINI;

        /* read RX FIFO  */
        cc112x_spiRegRead(CC112X_NUM_RXBYTES, &rf_rxBufLen, 1);
        cc112x_spiRxFifoRead(rf_rxBuf, rf_rxBufLen);
        LED_RX_OFF();
        TRACE_LOG_MAIN("RF_RX: RX FIFO len=%d; seq=%02x;", rf_rxBufLen, rf_rxBuf[3]);

        /* signal complete reception interrupt */
        evproc_putEvent(E_EVPROC_HEAD, NETSTK_RF_EVENT, NULL);
      }
      /* unexpected event? */
      else {
        TRACE_LOG_ERR("RF_RX_FINI: unexpected marc=%02x, cs=%02x, state=%d", marc_status, chip_status, rf_state);
      }
      break;
    }
  }
  TRACE_LOG_INT("RF_RX_FINI: marc=%02x; cs=%02x; state=%d", marc_status, chip_status, rf_state);
  TRACE_LOG_EXIT_ISR();
}


static void cc112x_isrTxPacketSent(void *p_arg) {
  uint8_t marc_status;
  uint8_t chip_status;

  /* avoid compiler warning of unused parameters */
  (void) &p_arg;
  TRACE_LOG_ENTER_ISR();

  /* clear ISR flag */
  bsp_extIntClear(RF_INT_CFG_TX_FINI);

  /* achieve MARC_STATUS to determine what caused the interrupt */
  chip_status = cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);
  if ((marc_status == RF_MARC_STATUS_TX_OVERFLOW) ||
      (marc_status == RF_MARC_STATUS_TX_UNDERFLOW)) {
    TRACE_LOG_ERR("RF_TX_FINI: FIFO ERR marc=%02x, cs=%02x, state=%d", marc_status, chip_status, rf_state);
    /* TX FIFO error, the chip will remain in TX mode
    * - flush TX FIFO
    * - force the radio to go to the IDLE state
    */
    cc112x_spiCmdStrobe(CC112X_SFTX);
    cc112x_gotoIdle();
  } else if (marc_status == RF_MARC_STATUS_TX_FINI) {
    /* the packet was successfully transmitted, the chip will go to the state
     * indicated by the RFEND_CFG0.TXOFF_MODE setting (default: IDLE) */
    rf_state = RF_STATE_TX_FINI;
  } else {
    TRACE_LOG_ERR("RF_TX_FINI: unexpected marc=%02x; cs=%02x; state=%d", marc_status, chip_status, rf_state);
  }
  TRACE_LOG_INT("RF_TX: ISR_TX_FINI marc=%02x; cs=%02x; state=%d", marc_status, chip_status, rf_state);
  TRACE_LOG_EXIT_ISR();
}


static void cc112x_eventHandler(c_event_t c_event, p_data_t p_data) {
  TRACE_LOG_MAIN("RF_RX: signal state=%d", rf_state);

  if (c_event == NETSTK_RF_EVENT) {
    /* set the error code to default */
    e_nsErr_t err = NETSTK_ERR_NONE;

    /* finalize reception process */
    if (rf_state == RF_STATE_RX_FINI) {
      /* put the radio in the IDLE state */
      cc112x_gotoIdle();
      LED_RX_OFF();

      /* do:
      * - signal the next higher layer of the received frame
      */
#if (EMB6_CFG_CONTINUOUS_RX == TRUE)
      (void)&err;
      /* always go to the RX state after complete reception when playing the
      * role of 'sniffer'
      */
      cc112x_gotoRX();
      /* print received frame on console */
      trace_printHex("", rf_rxBuf, rf_rxBufLen);
#else
      prf_netstk->phy->recv(rf_rxBuf, rf_rxBufLen, &err);
      if ((err != NETSTK_ERR_NONE) &&
          (err != NETSTK_ERR_INVALID_ADDRESS)) {
        /* the frame is discarded by upper layers */
        TRACE_LOG_ERR("RF_RX: discarded e=-%d", err);
        trace_printHex("RF_RX: discarded ", rf_rxBuf, rf_rxBufLen);
      }
      /* exit:
      * - clear RX buffer
      * - enter idle listening state if the RF is not there yet
      */
      rf_rxBufLen = 0;
      cc112x_gotoRX();
#endif
    }
  }
}



/*
 ********************************************************************************
 *                               MISCELLANEOUS
 ********************************************************************************
 */
static void cc112x_configureRegs(const s_regSettings_t *p_regs, uint8_t len) {
  uint8_t ix;
  uint8_t data;

  for (ix = 0; ix < len; ix++) {
    data = p_regs[ix].data;
    cc112x_spiRegWrite(p_regs[ix].addr, &data, 1);
  }
}


#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void cc112x_manualCalibration(void) {
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


static void cc112x_calibrateRCOsc(void) {
  uint8_t temp;

  /* Read current register value */
  cc112x_spiRegRead(CC112X_WOR_CFG0, &temp, 1);

  /* Mask register bit fields and write new values */
  temp = (temp & 0xF9) | (0x02 << 1);

  /* Write new register value */
  cc112x_spiRegWrite(CC112X_WOR_CFG0, &temp, 1);

  /* Strobe IDLE to calibrate the RCOSC */
  cc112x_spiCmdStrobe(CC112X_SIDLE);
  //cc112x_waitRdy();

  /* Disable RC calibration */
  temp = (temp & 0xF9) | (0x00 << 1);
  cc112x_spiRegWrite(CC112X_WOR_CFG0, &temp, 1);
}


static void cc112x_cca(e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  uint8_t rssi_status;

  /* set the returned error code to default */
  *p_err = NETSTK_ERR_NONE;

  if ((rf_state != RF_STATE_RX) &&
      (rf_state != RF_STATE_IDLE)) {
    *p_err = NETSTK_ERR_BUSY;
  } else {
    TRACE_LOG_MAIN("RF: CCA");

    /* entry:
    * - go to the IDLE state
    * - disable RF interrupts
    */
    cc112x_gotoIdle();
    cc112x_waitRdy();
    RF_INT_DISABLED();

    /* do:
    * - set RF state to CCA_BUSY
    * - change the radio state to RX
    * - poll for carrier sense valid
    * - change the radio state to IDLE
    * - read carrier sense
    */
    rf_state = RF_STATE_CCA_BUSY;

    /* force the radio to go to the RX state */
    uint8_t chip_status;
    do {
      chip_status = cc112x_spiCmdStrobe(CC112X_SRX);
      if (RF_GET_CHIP_STATE(chip_status) ==  0x60) {
        /* RX FIFO error then flush RX FIFO */
        cc112x_spiCmdStrobe(CC112X_SFRX);
      }
    } while (RF_GET_CHIP_STATE(chip_status) != RF_CHIP_STATE_RX );

    /* poll for carrier sense valid */
    do {
      cc112x_spiRegRead(CC112X_RSSI0, &rssi_status, 1);
    } while ((rssi_status & RF_RSSI0_CARRIER_SENSE_VALID) == 0);

    /* force the radio to go to the IDLE state and flush RX FIFO to avoid
    * corrupted data received during CCA
    */
    cc112x_gotoIdle();
    cc112x_spiCmdStrobe(CC112X_SFRX);

    if (rssi_status & RF_RSSI0_CARRIER_DETECTED) {
      *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
    }

    /* exit:
    * - go to the eWOR state
    */
    cc112x_gotoRX();
  }
}

static void cc112x_retx(e_nsErr_t *p_err)
{
  if ((rf_state != RF_STATE_RX) &&
      (rf_state != RF_STATE_IDLE)) {
    *p_err = NETSTK_ERR_BUSY;
  } else {
    /* entry:
    * - go to the IDLE state
    * - disable RX interrupt
    */
    cc112x_gotoIdle();
    cc112x_waitRdy();
    RF_INT_DISABLED();

    /* do:
    * - go to the TX_BUSY state
    * - rewrite TX first pointer
    * - enable TX interrupts
    * - issue STX strobe
    */
    rf_state = RF_STATE_TX_BUSY;
    cc112x_spiRegWrite(CC112X_TXFIRST, &rf_txFirstPtr, 1);

    LED_TX_ON();
    RF_INT_TX_ENABLED();
    cc112x_spiCmdStrobe(CC112X_STX);

    /* wait for packet to be sent */
    uint16_t iteration = 0xffff;
    while ((rf_state == RF_STATE_TX_BUSY) && (iteration > 0)) {
      iteration--;
    }
    if (rf_state == RF_STATE_TX_FINI) {
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



static void cc112x_rxFifoErrHandler(void) {
  /* readout any useful data, then flush RX FIFO */
  cc112x_spiRegRead(CC112X_NUM_RXBYTES, &rf_rxBufLen, 1);
  cc112x_spiRxFifoRead(rf_rxBuf, rf_rxBufLen);
  rf_rxBufLen = 0;  // TODO signal upper layer of received bytes

  /* force the radio to go to the IDLE state and flush RX FIFO */
  cc112x_spiCmdStrobe(CC112X_SFRX);

  /* put the radio to RX state */
  cc112x_gotoIdle();
  cc112x_gotoRX();
}


static void cc112x_txPowerSet(int8_t power, e_nsErr_t *p_err) {
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


static void cc112x_txPowerGet(int8_t *p_power, e_nsErr_t *p_err) {
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
static void cc112x_chanNumSet(uint8_t chan_num, e_nsErr_t *p_err) {
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

#if NETSTK_CFG_IEEE_802154G_EN
  uint32_t freq_reg = 0;
  uint32_t freq_center = 0;
  uint32_t freq_delta = 0;
  uint8_t write_byte = 0;

  /* set returned error value to default */
  *p_err = NETSTK_ERR_NONE;

  /* configure operating channel frequency based on operation mode i.e. CSM,
   * mode #1, mode #2, or mode #3 */
  switch (rf_opMode) {
    case NETSTK_RF_OP_MODE_CSM:
    if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_CSM) {
      freq_center = CC112X_CSM_CHAN_CENTER_FREQ;
      freq_delta = CC112X_CSM_DELTA_FREQ;
      rf_chanNum = chan_num;
    } else {
      *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    }
    break;

#if (NETSTK_CFG_PHY_OP_MODE_1_EN == TRUE)
    case NETSTK_RF_OP_MODE_1:
    if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE1) {
      freq_center = CC112X_OPMODE1_CHAN_CENTER_FREQ;
      freq_delta = CC112X_OPMODE1_DELTA_FREQ;
      rf_chanNum = chan_num;
    } else {
      *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    }
    break;
#endif

#if (NETSTK_CFG_PHY_OP_MODE_2_EN == TRUE)
    case NETSTK_RF_OP_MODE_2:
    if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE2) {
      freq_center = CC112X_OPMODE2_CHAN_CENTER_FREQ;
      freq_delta = CC112X_OPMODE2_DELTA_FREQ;
      rf_chanNum = chan_num;
    } else {
      *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    }
    break;
#endif

#if (NETSTK_CFG_PHY_OP_MODE_3_EN == TRUE)
    case NETSTK_RF_OP_MODE_3:
    if (chan_num < NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE3) {
      freq_center = CC112X_OPMODE3_CHAN_CENTER_FREQ;
      freq_delta = CC112X_OPMODE3_DELTA_FREQ;
      rf_chanNum = chan_num;
    } else {
      *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    }
    break;
#endif

    default:
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    break;
  }

  /*
   * configure frequency registers only when arguments are valid
   */
  if (*p_err == NETSTK_ERR_NONE) {
    freq_reg = freq_center + rf_chanNum * freq_delta;

    write_byte = (freq_reg & 0x00FF0000) >> 16;
    cc112x_spiRegWrite(CC112X_FREQ2, &write_byte, 1);

    write_byte = (freq_reg & 0x0000FF00) >> 8;
    cc112x_spiRegWrite(CC112X_FREQ1, &write_byte, 1);

    write_byte = (freq_reg & 0x000000FF);
    cc112x_spiRegWrite(CC112X_FREQ0, &write_byte, 1);
  }
#endif /* #if NETSTK_CFG_IEEE_802154G_EN */
}

/**
 * @brief   Select operating mode
 * @param   mode        Operating mode to select
 * @param   p_err       Pointer to variable holding returned error code
 */
static void cc112x_opModeSet(e_nsRfOpMode mode, e_nsErr_t *p_err) {
  if (rf_opMode < NETSTK_RF_OP_MODE_MAX) {
    rf_opMode = mode;
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
static void cc112x_readRSSI(int16_t *p_val, e_nsErr_t *p_err) {
  int16_t i_ret = 0;
  uint8_t rssi2complMSB;
  uint8_t rssi2complLSB;

  /* read the RSSI register and wait for a valid value */
  do {
    cc112x_spiRegRead(CC112X_RSSI0, &rssi2complLSB, 1);
  } while ((rssi2complLSB & RF_RSSI_VALID_MSK) == 0);

  /* read 2nd RSSI register and add to value */
  cc112x_spiRegRead(CC112X_RSSI1, &rssi2complMSB, 1);
  i_ret = ((int8_t)(rssi2complMSB) << RF_RSSI_LOW_LEN) |
          ((int8_t)(rssi2complLSB) >> RF_RSSI_LOW_OFST);

  /* get real RSSI from offset and resolution */
  *p_val = (int16_t)((i_ret * RF_RSSI_RES) - RF_RSSI_OFFSET);
}


/**
 * @brief   Perform self-testing
 * @param   p_err
 */
static void cc112x_selfTest(e_nsErr_t *p_err) {
  uint8_t dummy_data[10] = {0x09, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  uint8_t ix;


  trace_printf("RF_TEST");
  /*
  * turn the radio on
  */
  cc112x_On(p_err);

  /*
  * put radio to IDLE state
  */
  cc112x_gotoIdle();

  /*
  * put radio to RX state multiple times
  */
  cc112x_gotoRX();


  /*
  * alternate between IDLE and RX
  */
  for (ix = 0; ix < 10; ix++) {
    cc112x_gotoIdle();
    cc112x_gotoRX();
  }

  /*
  * put radio to TX state
  */
  for (ix = 0; ix < 3; ix++) {
    cc112x_Send(dummy_data, sizeof(dummy_data), p_err);
    assert_equ("TX: ", NETSTK_ERR_NONE, *p_err);

    /* delay 500ms*/
    rt_tmr_delay(500);
  }
}


/*
 ********************************************************************************
 *                               DRIVER DEFINITION
 ********************************************************************************
 */
const s_nsRF_t rf_driver_ticc112x = {
    "CC112X",
    cc112x_Init,
    cc112x_On,
    cc112x_Off,
    cc112x_Send,
    cc112x_Recv,
    cc112x_Ioctl,
};

/*
 ********************************************************************************
 *                                   END OF FILE
 ********************************************************************************
 */
