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

#include "lib_tmr.h"
#include "lib_port.h"
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
typedef enum e_rf_state
{
    RF_STATE_NON_INIT,
    RF_STATE_INIT,
    RF_STATE_SLEEP,
    RF_STATE_ERR,
    RF_STATE_IDLE,

    /* RX Submachine states */
    RF_STATE_RX_LISTENING,
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

}e_rfState_t;


/*
********************************************************************************
*                                LOCAL DEFINES
********************************************************************************
*/

#ifndef CC112X_PART_NUMBER
#define CC112X_PART_NUMBER                  0x48
#endif /* #ifndef CC112X_PART_NUMBER */

#ifndef CC112X_PART_VERSION
#define CC112X_PART_VERSION                 0x21
#endif /* #ifndef CC112X_PART_VERSION */

#define RF_SEM_POST(_event_)                evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)
#define RF_SEM_WAIT(_event_)                evproc_regCallback(_event_, cc112x_eventHandler)

#define RF_CFG_MAX_PACKET_LENGTH            (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)

#define RF_IS_IN_TX(_chip_status)           ((_chip_status) & 0x20)
#define RF_IS_IN_RX(_chip_status)           ((_chip_status) & 0x10)

#define RF_CCA_MODE_NONE                    (uint8_t)( 0x00 )
#define RF_CCA_MODE_RSSI_BELOW_THR          (uint8_t)( 0x24 )

#define RF_CHIP_STATE_IDLE                  (uint8_t)( 0x00 )
#define RF_CHIP_STATE_RX                    (uint8_t)( 0x10 )
#define RF_CHIP_STATE_TX                    (uint8_t)( 0x20 )

#define RF_MARC_STATUS_TX_FINI              (uint8_t)( 0x40 )
#define RF_MARC_STATUS_RX_FINI              (uint8_t)( 0x80 )

#define RF_GET_CHIP_STATE(_chip_status)     (uint8_t)((_chip_status) & 0x70)

/*!< Set RF to fixed packet length mode */
#define RF_SET_FIXED_PKT_MODE() \
    do {    \
        uint8_t _wr_byte = RF_FIXED_PACKET_LENGTH;          \
        cc112x_spiRegWrite(CC112X_PKT_CFG0, &_wr_byte, 1);  \
    } while (0)

/*!< Disable all RF interrupts */
#define RF_EXTI_DISABLED()  \
    do {    \
        bsp_extIntDisable(E_TARGET_EXT_INT_0);  \
        bsp_extIntDisable(E_TARGET_EXT_INT_1);  \
        bsp_extIntDisable(E_TARGET_EXT_INT_2);  \
        bsp_extIntClear(E_TARGET_EXT_INT_0);    \
        bsp_extIntClear(E_TARGET_EXT_INT_1);    \
        bsp_extIntClear(E_TARGET_EXT_INT_2);    \
    } while (0)

/*!< check if RF is in one of reception states */
#define RF_IS_RX_BUSY() \
    ((rf_state == RF_STATE_RX_SYNC)             ||  \
     (rf_state == RF_STATE_RX_PORTION_MIDDLE)   ||  \
     (rf_state == RF_STATE_RX_PORTION_LAST)     ||  \
     (rf_state == RF_STATE_RX_FINI))

/*!< Set RF to RX mode */
#define RF_GOTO_RX()    \
    do {    \
        uint8_t _chip_status;                                           \
        do {                                                            \
            _chip_status = cc112x_spiCmdStrobe(CC112X_SRX);             \
        } while (RF_GET_CHIP_STATE(_chip_status) != RF_CHIP_STATE_RX);  \
    } while (0)

/*
 * CC112x has 128-byte TX FIFO and 128-byte RX-FIFO
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
#define RF_CFG_FIFO_THR                     (uint8_t)( 120u )
#define RF_CFG_MAX_VARIABLE_LENGTH          (uint8_t)( 255u )

#define RF_CFG_BYTES_IN_RX_FIFO             (uint8_t)( 121u )

#define RF_CFG_FIFO_SIZE                    (uint8_t)( 128u )
#define RF_CFG_AVAI_BYTES_IN_TX_FIFO        (uint8_t)( 121u )   // in TI example using 122?
#define RF_CFG_BYTES_IN_TX_FIFO             (uint8_t)( RF_CFG_FIFO_SIZE - RF_CFG_AVAI_BYTES_IN_TX_FIFO)

#define RF_FIXED_PACKET_LENGTH              (uint8_t)( 0x00u )

#define RF_INT_CFG_TX_FIFO_THR              E_TARGET_EXT_INT_0
#define RF_INT_CFG_TX_FINI                  E_TARGET_EXT_INT_1
#define RF_INT_CFG_TX_CCA_DONE              E_TARGET_EXT_INT_2

#define RF_INT_CFG_EDGE_TX_FIFO_THR         E_TARGET_INT_EDGE_FALLING
#define RF_INT_CFG_EDGE_TX_FINI             E_TARGET_INT_EDGE_FALLING
#define RF_INT_CFG_EDGE_TX_CCA_DONE         E_TARGET_INT_EDGE_RISING

#define RF_INT_CFG_RX_FIFO_THR              E_TARGET_EXT_INT_0
#define RF_INT_CFG_RX_SYNC                  E_TARGET_EXT_INT_1
#define RF_INT_CFG_RX_FINI                  E_TARGET_EXT_INT_2

#define RF_INT_CFG_EDGE_RX_FIFO_THR         E_TARGET_INT_EDGE_RISING
#define RF_INT_CFG_EDGE_RX_SYNC             E_TARGET_INT_EDGE_RISING
#define RF_INT_CFG_EDGE_RX_FINI             E_TARGET_INT_EDGE_FALLING

#ifndef RF_WD_ENABLE
#define RF_WD_ENABLE                        TRUE
#endif /* #endif RF_WD_ENABLE */

#if RF_WD_ENABLE
#ifndef RF_WD_TIMER_MS
#define RF_WD_TIMER_MS                      300
#endif /* #ifndef RF_WD_TIMER_MS */
#endif /* #if RF_WD_ENABLE */

/*
********************************************************************************
*                                LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t      *rf_netstk;
static uint8_t      rf_rxBuf[RF_CFG_MAX_PACKET_LENGTH];
static uint16_t     rf_rxBufLen;
static uint16_t     rf_byteLeft;
static e_rfState_t  rf_state = RF_STATE_NON_INIT;

static uint8_t      rf_fixedPktLenMode;
static uint8_t      rf_iterations;
static uint8_t     *rf_bufIx;
static uint8_t      rf_txLastPortion;
static uint8_t      rf_worEn;

static uint8_t      rf_chanNum;
static e_nsRfOpMode rf_opMode;
static uint8_t      rf_regVerify;

#if RF_WD_ENABLE
static LIB_TMR      rf_tmrWD;
#endif /* #if RF_WD_ENABLE */

/*
********************************************************************************
*                           LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void cc112x_Init (void *p_netstk, e_nsErr_t *p_err);
static void cc112x_On (e_nsErr_t *p_err);
static void cc112x_Off (e_nsErr_t *p_err);
static void cc112x_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void cc112x_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err);
static void cc112x_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

static void cc112x_rxByteLeftChk(void);
static void cc112x_isrRxSyncReceived(void *p_arg);
static void cc112x_isrRxFifoAboveThreshold(void *p_arg);
static void cc112x_isrRxPacketReceived(void *p_arg);
static void cc112x_isrTxFifoBelowThreshold(void *p_arg);
static void cc112x_isrTxPacketSent(void *p_arg);

#if RF_WD_ENABLE
static void cc112x_wdCB(void*);
#endif /* #if RF_WD_ENABLE */
static void cc112x_eventHandler(c_event_t c_event, p_data_t p_data);
static void cc112x_configureRegs(const s_regSettings_t *p_regs, uint8_t len);
static void cc112x_calibrateRF(void);
static void cc112x_calibrateRCOsc(void);
static void cc112x_cca(e_nsErr_t *p_err);

static void cc112x_reset(void);
static void cc112x_chkPartnumber(e_nsErr_t *p_err);
static void cc112x_chkReset(void);
static void cc112x_waitRdy(void);
static void cc112x_gotoSleep(void);
static void cc112x_gotoRx(void);
static void cc112x_gotoIdle(void);

static void cc112x_txPowerSet(int8_t power, e_nsErr_t *p_err);
static void cc112x_txPowerGet(int8_t *p_power, e_nsErr_t *p_err);
static void cc112x_chanNumSet(uint8_t chan_num, e_nsErr_t *p_err);
static void cc112x_opModeSet(e_nsRfOpMode mode, e_nsErr_t *p_err);

/*
********************************************************************************
*                           LOCAL FUNCTIONS DEFINITIONS
********************************************************************************
*/
static void cc112x_Init (void *p_netstk, e_nsErr_t *p_err)
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

    uint8_t len;

    RF_EXTI_DISABLED();

    /* indicates radio is in state initialization */
    rf_state = RF_STATE_INIT;

    /* store pointer to global netstack structure */
    rf_netstk = (s_ns_t *)p_netstk;

    /* initialize SPI handle */
    cc112x_spiInit();

    /* reset the transceiver */
    cc112x_reset();

    /* check part number */
    cc112x_chkPartnumber(p_err);
    if (*p_err != NETSTK_ERR_NONE) {
        emb6_errorHandler(p_err);
    }

    /* set RF registers to default values */
    len = sizeof(cc112x_cfg_ieee802154g_default) / sizeof(s_regSettings_t);
    cc112x_configureRegs(cc112x_cfg_ieee802154g_default, len);

    /* calibrate radio */
    cc112x_calibrateRF();

    /* calibrate RC oscillator */
    cc112x_calibrateRCOsc();

    /* initialize local variables */
    RF_SEM_WAIT(NETSTK_RF_EVENT);
    memset(rf_rxBuf, 0, sizeof(rf_rxBuf));
    rf_rxBufLen = 0;
    rf_worEn = FALSE;    /* enable WOR mode by default */

    /* configure operating mode and channel number */
    rf_chanNum = 0;
    rf_opMode = NETSTK_RF_OP_MODE_CSM;

#if RF_WD_ENABLE
    /* create watchdog timer */
    Tmr_Create(&rf_tmrWD,
               LIB_TMR_TYPE_ONE_SHOT,
               RF_WD_TIMER_MS,
               cc112x_wdCB,
               NULL);
#endif /* #if RF_WD_ENABLE */


    /* goto state idle */
    cc112x_gotoIdle();
}


static void cc112x_On (e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    /* first set RF to state idle */
    cc112x_gotoIdle();

    /* go to state RX */
    cc112x_gotoRx();

    /* indicate successful operation */
    *p_err = NETSTK_ERR_NONE;
}


static void cc112x_Off (e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    /* put transceiver to state idle */
    cc112x_gotoIdle();

    /* go to state sleep */
    cc112x_gotoSleep();

    /* indicate successful operation */
    *p_err = NETSTK_ERR_NONE;
}


static void cc112x_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }

    if ((p_data == NULL) ||
        (len == 0)) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        return;
    }
#endif

    if ((rf_state != RF_STATE_RX_LISTENING) &&
        (rf_state != RF_STATE_IDLE)) {
        *p_err = NETSTK_ERR_BUSY;
    } else {
#if LOGGER_ENABLE
        /*
         * Logging
         */
        uint16_t data_len = len;
        uint8_t *p_dataptr = p_data;
        LOG_RAW("RADIO_TX: ");
        while (data_len--) {
            LOG_RAW("%02x", *p_dataptr++);
        }
        LOG_RAW("\n\r\n\r");
#endif

        if (len > RF_CFG_MAX_PACKET_LENGTH) {
            /* packet length is out of range, and therefore transmission is
             * refused */
            *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            return;
        }

        /*
         * entry actions
         */
        uint8_t reg_len, write_byte;

        /* go to state IDLE and flush TX FIFO */
        LED_TX_ON();
        cc112x_spiCmdStrobe(CC112X_SIDLE);
        cc112x_spiCmdStrobe(CC112X_SFTX);

        /* disable RF external interrupts */
        RF_EXTI_DISABLED();

        /* configure RF GPIOs with infinite packet length mode */
        rf_fixedPktLenMode = FALSE;
        rf_txLastPortion = FALSE;
        reg_len = sizeof(cc112x_cfg_tx) / sizeof(s_regSettings_t);
        cc112x_configureRegs(cc112x_cfg_tx, reg_len);

        /* set packet length mode based on length of packet to send */
        if (len > RF_CFG_MAX_VARIABLE_LENGTH) {
            /*
             * do actions
             */
            rf_state = RF_STATE_TX_BUSY;
            rf_byteLeft = len;
            rf_bufIx = p_data;

            /* set fixed packet length */
            write_byte = len % (RF_CFG_MAX_VARIABLE_LENGTH + 1);
            cc112x_spiRegWrite(CC112X_PKT_LEN, &write_byte, 1);

            /* configure RF external interrupts */
            bsp_extIntRegister(RF_INT_CFG_TX_FIFO_THR, RF_INT_CFG_EDGE_TX_FIFO_THR, cc112x_isrTxFifoBelowThreshold);
            bsp_extIntRegister(RF_INT_CFG_TX_FINI, RF_INT_CFG_EDGE_TX_FINI, cc112x_isrTxPacketSent);

            bsp_extIntEnable(RF_INT_CFG_TX_FIFO_THR);
            bsp_extIntEnable(RF_INT_CFG_TX_FINI);

            /* write packet to send into TX FIFO */
            cc112x_spiTxFifoWrite(p_data, RF_CFG_FIFO_SIZE);
            rf_byteLeft -= RF_CFG_FIFO_SIZE;
            rf_bufIx += RF_CFG_FIFO_SIZE;
            rf_iterations = (rf_byteLeft / RF_CFG_AVAI_BYTES_IN_TX_FIFO);

            /* enter TX mode */
            cc112x_spiCmdStrobe(CC112X_STX);
        } else {
            /*
             * do actions
             */
            /* go to state TX_BUSY */
            rf_state = RF_STATE_TX_BUSY;
            rf_txLastPortion = TRUE;

            /* set fixed packet length mode */
            RF_SET_FIXED_PKT_MODE();
            rf_fixedPktLenMode = TRUE;

            /* set fixed packet length */
            write_byte = (uint8_t)(len % (RF_CFG_MAX_VARIABLE_LENGTH + 1));
            cc112x_spiRegWrite(CC112X_PKT_LEN, &write_byte, 1);

            /* using only interrupt PKT_SYNC_RXTX on falling edge is sufficient */
            bsp_extIntRegister(RF_INT_CFG_TX_FINI, RF_INT_CFG_EDGE_TX_FINI, cc112x_isrTxPacketSent);
            bsp_extIntEnable(RF_INT_CFG_TX_FINI);

            /* write packet to send into TX FIFO */
            cc112x_spiTxFifoWrite(p_data, len);

            /* enter TX mode */
            cc112x_spiCmdStrobe(CC112X_STX);
        }

#if RF_WD_ENABLE
        Tmr_Stop( &rf_tmrWD );
        Tmr_Start( &rf_tmrWD );
#endif /* #if RF_WD_ENABLE */

        /* wait for packet to be sent */
        uint16_t iteration = 0xffff;
        while ((rf_state == RF_STATE_TX_BUSY) &&
               (iteration > 0)) {
            iteration--;
        }

        /*
         * Exit actions
         */
        if (rf_state == RF_STATE_TX_FINI) {
            *p_err = NETSTK_ERR_NONE;
        } else {
            cc112x_chkReset();
            *p_err = NETSTK_ERR_TX_TIMEOUT;
        }

        LED_TX_OFF();
        cc112x_gotoRx();
    }
}


static void cc112x_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif


}


static void cc112x_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
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
                rf_worEn = *((uint8_t *)p_val);
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
static void cc112x_gotoSleep(void)
{
    /* disable RF external interrupts */
    RF_EXTI_DISABLED();

    /* enter state Sleep */
    rf_state = RF_STATE_SLEEP;
    cc112x_spiCmdStrobe(CC112X_SPWD);
}


static void cc112x_gotoRx(void)
{
    uint8_t reg_len;
    uint8_t write_byte;

    /* go to state IDLE and flush RX FIFO */
    cc112x_spiCmdStrobe(CC112X_SIDLE);
    cc112x_spiCmdStrobe(CC112X_SFRX);

    /* disable RF external interrupts */
    RF_EXTI_DISABLED();

    /* configure RF GPIOs */
    reg_len = sizeof(cc112x_cfg_rx_wor) / sizeof(s_regSettings_t);
    cc112x_configureRegs(cc112x_cfg_rx_wor, reg_len);

    /* infinite packet length mode by default */
    rf_fixedPktLenMode = FALSE;

    /* configure RF external interrupts */
    bsp_extIntRegister(RF_INT_CFG_RX_SYNC, RF_INT_CFG_EDGE_RX_SYNC, cc112x_isrRxSyncReceived);
    bsp_extIntRegister(RF_INT_CFG_RX_FIFO_THR, RF_INT_CFG_EDGE_RX_FIFO_THR, cc112x_isrRxFifoAboveThreshold);
    bsp_extIntRegister(RF_INT_CFG_RX_FINI, RF_INT_CFG_EDGE_RX_FINI, cc112x_isrRxPacketReceived);

    /* enable RX interrupts */
    bsp_extIntEnable(RF_INT_CFG_RX_SYNC);
    bsp_extIntEnable(RF_INT_CFG_RX_FIFO_THR);

    /* set receive mode */
    if (rf_worEn) {
        /* enable RX termination on bad packets */
        write_byte = 0x09;
        cc112x_spiRegWrite(CC112X_RFEND_CFG0, &write_byte, 1);

        /* set preamble length to 24 bytes */
        write_byte = 0x31;
        cc112x_spiRegWrite(CC112X_PREAMBLE_CFG1, &write_byte, 1);

        /* enter state Sniff */
        cc112x_spiCmdStrobe(CC112X_SWOR);
    } else {
        /* disable RX termination on bad packets regardless of the RXOFF_MODE */
        write_byte = 0;
        cc112x_spiRegWrite(CC112X_RFEND_CFG0, &write_byte, 1);

        /* set preamble length to 4 bytes */
        write_byte = 0x19;
        cc112x_spiRegWrite(CC112X_PREAMBLE_CFG1, &write_byte, 1);

        /* Strobe RX */
        uint8_t chip_status;
        do {
            chip_status = cc112x_spiCmdStrobe(CC112X_SRX);
        } while (RF_GET_CHIP_STATE(chip_status) != RF_CHIP_STATE_RX);
    }
    rf_state = RF_STATE_RX_LISTENING;
}


static void cc112x_gotoIdle(void)
{
    uint8_t chip_status;

    /* disable RF external interrupts */
    RF_EXTI_DISABLED();

    /* issue strobe IDLE */
    do {
        chip_status = cc112x_spiCmdStrobe(CC112X_SIDLE);
    } while (RF_GET_CHIP_STATE(chip_status) != RF_CHIP_STATE_IDLE);

    /* flush TXFIFO, RXFIFO */
    cc112x_spiCmdStrobe(CC112X_SFRX);
    cc112x_spiCmdStrobe(CC112X_SFTX);

    /* set RF driver state to idle */
    rf_state = RF_STATE_IDLE;
}


static void cc112x_reset(void)
{
    cc112x_spiCmdStrobe(CC112X_SRES);
    cc112x_waitRdy();
}


static void cc112x_chkPartnumber(e_nsErr_t *p_err)
{
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

static void cc112x_chkReset(void)
{
    e_nsErr_t err;
    uint8_t read_byte;

    /* check one of the registers */
    cc112x_spiRegRead(CC112X_FREQ2, &read_byte, 1);

    if (read_byte != rf_regVerify ) {
        uint8_t rf_chanNum_st = rf_chanNum;
        uint8_t rf_opMode_st = rf_opMode;

        cc112x_Init( rf_netstk, &err );
        cc112x_opModeSet( rf_opMode_st, &err );
        cc112x_chanNumSet( rf_chanNum_st, &err );
    }
}

static void cc112x_waitRdy(void)
{
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
static void cc112x_rxByteLeftChk(void)
{
    /* if incoming bytes can be stored in RX FIFO then set to fixed packet
     * length mode */
    if ((rf_byteLeft < (RF_CFG_MAX_VARIABLE_LENGTH + 1)) &&
        (rf_fixedPktLenMode == FALSE)) {
        /* set fixed packet length mode */
        RF_SET_FIXED_PKT_MODE();
        rf_fixedPktLenMode = TRUE;
    }

    /* disable RX FIFO THR when number of remaining bytes less than the
     * threshold and go to a state to receive the last packet portion */
    if (rf_byteLeft <= RF_CFG_BYTES_IN_RX_FIFO) {
        rf_state = RF_STATE_RX_PORTION_LAST;
        bsp_extIntDisable(RF_INT_CFG_RX_FIFO_THR);
    }
}

static void cc112x_isrRxSyncReceived(void *p_arg)
{
    /* avoid compiler warning of unused parameters */
    (void)&p_arg;

    /* achieve MARC_STATUS to determine what caused the interrupt */
    uint8_t marc_status;
    cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);

    if (rf_state == RF_STATE_RX_LISTENING) {
        uint8_t num_rx_bytes;
        uint16_t iteration;
        uint16_t pkt_len;

        /* go to state RX SYCN */
        rf_state = RF_STATE_RX_SYNC;
        LED_RX_ON();

#if RF_WD_ENABLE
        /* start the timeout and schedule an event*/
        Tmr_Stop( &rf_tmrWD );
        Tmr_Start( &rf_tmrWD );
#endif /* #if RF_WD_ENABLE */

        /* 
         * Wait until entire PHY header is received or number of register-
         * reading attempts exceeds the predefined max value
         */
        iteration = 0xffff;
        num_rx_bytes = 0;
        while ((num_rx_bytes < PHY_HEADER_LEN) && (iteration > 0)) {
            iteration--;
            cc112x_spiRegRead(CC112X_NUM_RXBYTES, &num_rx_bytes, 1);
        }

        if (num_rx_bytes < PHY_HEADER_LEN) {
            /* invalid packet, then goto RX state */
            cc112x_gotoRx();
        } else {
            /* parse PHY header for packet length */
            cc112x_spiRxFifoRead(rf_rxBuf, PHY_HEADER_LEN);
            pkt_len = phy_framer802154_getPktLen(rf_rxBuf, PHY_HEADER_LEN);

            /* make sure that the packet length is acceptable */
            if ((pkt_len == 0) ||
                (pkt_len > RF_CFG_MAX_PACKET_LENGTH)) {
                /* invalid packet length, then goto RX state */
                cc112x_gotoRx();
            } else {
                rf_state = RF_STATE_RX_PORTION_MIDDLE;

                /* set RX buffer attributes in corresponds to the incoming packet */
                rf_rxBufLen = PHY_HEADER_LEN + pkt_len;
                rf_byteLeft = pkt_len;
                rf_bufIx = &rf_rxBuf[PHY_HEADER_LEN];

                /* check number of remaining bytes */
                cc112x_rxByteLeftChk();

                /* set fixed packet length */
                uint8_t write_byte;
                write_byte = rf_rxBufLen % (RF_CFG_MAX_VARIABLE_LENGTH + 1);
                cc112x_spiRegWrite(CC112X_PKT_LEN, &write_byte, 1);

                /* enable PKT_SYCN_RXTX interrupt on falling edge, indicating entire
                 * packet arrives */
                bsp_extIntClear(RF_INT_CFG_RX_FINI);
                bsp_extIntEnable(RF_INT_CFG_RX_FINI);
            }
        }
    }
    else if( rf_state != RF_STATE_INIT )
    {
        emb6_errorHandler(NULL);
    }

    /* clear ISR flag */
    bsp_extIntClear(RF_INT_CFG_RX_SYNC);
}


static void cc112x_isrRxFifoAboveThreshold(void *p_arg)
{
    /* avoid compiler warning of unused parameters */
    (void)&p_arg;

    /* achieve MARC_STATUS to determine what caused the interrupt */
    uint8_t marc_status;
    cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);

    /* only receive middle portions of packet here */
    if (rf_state == RF_STATE_RX_PORTION_MIDDLE) {

        /* read RF_CFG_BYTES_IN_RX_FIFO bytes from the RX FIFO */
        cc112x_spiRxFifoRead(rf_bufIx, RF_CFG_BYTES_IN_RX_FIFO);
        rf_byteLeft -= RF_CFG_BYTES_IN_RX_FIFO;
        rf_bufIx += RF_CFG_BYTES_IN_RX_FIFO;

        /* check number of remaining bytes */
        cc112x_rxByteLeftChk();
    }

    /* clear ISR flag */
    bsp_extIntClear(RF_INT_CFG_RX_FIFO_THR);
}


static void cc112x_isrRxPacketReceived(void *p_arg)
{
    /* avoid compiler warning of unused parameters */
    (void)&p_arg;

    uint8_t marc_status;
    uint8_t is_rx_ok;

    /* achieve MARC_STATUS to determine what caused the interrupt */
    cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);

    /* check reception process result */
    is_rx_ok = (rf_state == RF_STATE_RX_PORTION_LAST) &&
               (marc_status == RF_MARC_STATUS_RX_FINI);

    if (is_rx_ok) {
        /* indicate that reception process has finished */
        rf_state = RF_STATE_RX_FINI;

        /* read remaining bytes */
        cc112x_spiRxFifoRead(rf_bufIx, rf_byteLeft);
        rf_byteLeft = 0;

#if RF_WD_ENABLE
        /* stop WD timer */
        Tmr_Stop( &rf_tmrWD );
#endif /* #if RF_WD_ENABLE */

        /* signal complete reception interrupt */
        RF_SEM_POST(NETSTK_RF_EVENT);
    }

    /* clear ISR flag */
    bsp_extIntClear(RF_INT_CFG_RX_FINI);
    LED_RX_OFF();
}


/**
 * @brief   This function runs every time the TX FIFO is drained below
 *          127 - FIFO_THR = 127 - 120 = 7 [bytes]
 * @param   p_arg
 */
static void cc112x_isrTxFifoBelowThreshold(void *p_arg)
{
    /* avoid compiler warning of unused parameters */
    (void)&p_arg;

    if (rf_txLastPortion == TRUE) {
        /* fill up the TX FIFO with remaining bytes */
        cc112x_spiTxFifoWrite(rf_bufIx, rf_byteLeft);
        rf_byteLeft = 0;

        /* disable interrupt RF_INT_CFG_TX_FIFO_THR */
        bsp_extIntDisable(RF_INT_CFG_TX_FIFO_THR);
    } else {
        /* fill up the TX FIFO */
        cc112x_spiTxFifoWrite(rf_bufIx, RF_CFG_AVAI_BYTES_IN_TX_FIFO);

        if ((rf_byteLeft < (RF_CFG_MAX_VARIABLE_LENGTH + 1 - RF_CFG_BYTES_IN_TX_FIFO)) && (rf_fixedPktLenMode == FALSE)) {
            /* set fixed packet length mode */
            RF_SET_FIXED_PKT_MODE();
            rf_fixedPktLenMode = TRUE;
        }

        /* update TX attributes */
        rf_byteLeft -= RF_CFG_AVAI_BYTES_IN_TX_FIFO;
        rf_bufIx += RF_CFG_AVAI_BYTES_IN_TX_FIFO;

        rf_iterations--;
        if (rf_iterations == 0) {
            rf_txLastPortion = TRUE;
        }
    }

    /* clear ISR flag */
    bsp_extIntClear(RF_INT_CFG_TX_FIFO_THR);
}

static void cc112x_isrTxPacketSent(void *p_arg)
{
    uint8_t marc_status;
    uint8_t is_tx_ok;

    /* achieve MARC_STATUS to determine what caused the interrupt */
    cc112x_spiRegRead(CC112X_MARC_STATUS1, &marc_status, 1);

    /* check TX process result */
    is_tx_ok = (marc_status == RF_MARC_STATUS_TX_FINI) &&
               (rf_state == RF_STATE_TX_BUSY) &&
               (rf_txLastPortion == TRUE);
    if (is_tx_ok) {
        /* TX process has successfully finished */
        rf_state = RF_STATE_TX_FINI;
    } else {
        /* flush TX FIFO */
        cc112x_spiCmdStrobe(CC112X_SFTX);

        /* error occurs, RF shall be reset */
        rf_state = RF_STATE_ERR;
    }

#if RF_WD_ENABLE
    /* stop WD timer */
    Tmr_Stop( &rf_tmrWD );
#endif /* #if RF_WD_ENABLE */

    /* clear ISR flag */
    bsp_extIntClear(RF_INT_CFG_TX_FINI);
}

#if RF_WD_ENABLE
static void cc112x_wdCB(void* param)
{
    /* call the event handler */
    cc112x_eventHandler( 0, NULL );
}
#endif /* #if RF_WD_ENABLE */

static void cc112x_eventHandler(c_event_t c_event, p_data_t p_data)
{
    /* set the error code to default */
    e_nsErr_t err = NETSTK_ERR_NONE;

    /* finalize reception process */
    if (rf_state == RF_STATE_RX_FINI) {
        /*
         * entry action
         */
        rf_state = RF_STATE_IDLE;
        LED_RX_OFF();

        /*
         * do actions:
         * (1)  Signal the next higher layer of the received frame
         */
#if LOGGER_ENABLE
        /*
         * Logging
         */
        uint16_t data_len = rf_rxBufLen;
        uint8_t *p_dataptr = rf_rxBuf;
        LOG_RAW("RADIO_RX: ");
        while (data_len--) {
            LOG_RAW("%02x", *p_dataptr++);
        }
        LOG_RAW("\n\r\n\r");
#endif

        rf_netstk->phy->recv(rf_rxBuf, rf_rxBufLen, &err);

        /*
         * exit actions
         * (1)  Clear RX buffer if invalid CRC is present
         * (2)  Enter idle listening state if the RF is not there yet
         */
        if (err != NETSTK_ERR_NONE) {
            rf_rxBufLen = 0;
            rf_bufIx = rf_rxBuf;
            memset(rf_rxBuf, 0, sizeof(rf_rxBuf));
        }
        if (rf_state != RF_STATE_RX_LISTENING) {
            cc112x_gotoRx();
        }
    }

#if RF_WD_ENABLE
    if( Tmr_StateGet( &rf_tmrWD ) == LIB_TMR_STATE_FINISHED )
    {
        if( (rf_state != RF_STATE_IDLE) &&
            (rf_state != RF_STATE_RX_LISTENING) &&
            (rf_state != RF_STATE_SLEEP) )
        {
            /* rf state timeout. stop the timer and reset RF */
            Tmr_Stop( &rf_tmrWD );
            /* check reset */
            cc112x_chkReset();
            /* reset RF */
            cc112x_gotoRx();
        }
    }
#endif /* #if RF_WD_ENABLE */
}



/*
********************************************************************************
*                               MISCELLANEOUS
********************************************************************************
*/
static void cc112x_configureRegs(const s_regSettings_t *p_regs, uint8_t len)
{
    uint8_t ix;
    uint8_t data;

    for (ix = 0; ix < len; ix++) {
        data = p_regs[ix].data;
        cc112x_spiRegWrite(p_regs[ix].addr, &data, 1);
    }
}


static void cc112x_calibrateRF(void)
{
    uint8_t marc_state;


    /* calibrate radio and wait until the calibration is done */
    cc112x_spiCmdStrobe(CC112X_SCAL);
    do {
        cc112x_spiRegRead(CC112X_MARCSTATE, &marc_state, 1);
    } while (marc_state != 0x41);
}


static void cc112x_calibrateRCOsc(void)
{
    uint8_t temp;

    /* Read current register value */
    cc112x_spiRegRead(CC112X_WOR_CFG0, &temp,1);

    /* Mask register bit fields and write new values */
    temp = (temp & 0xF9) | (0x02 << 1);

    /* Write new register value */
    cc112x_spiRegWrite(CC112X_WOR_CFG0, &temp,1);

    /* Strobe IDLE to calibrate the RCOSC */
    cc112x_spiCmdStrobe(CC112X_SIDLE);

    /* Disable RC calibration */
    temp = (temp & 0xF9) | (0x00 << 1);
    cc112x_spiRegWrite(CC112X_WOR_CFG0, &temp, 1);
}


static void cc112x_cca(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    uint8_t rssi_status;
    rf_status_t chip_status;
    uint8_t cca_mode;
    e_rfState_t e_rf_state_store = rf_state;

    /* set the returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    if ((rf_state != RF_STATE_RX_LISTENING) &&
        (rf_state != RF_STATE_IDLE)) {
        *p_err = NETSTK_ERR_BUSY;
    } else {

        /* disable RF external interrupts */
        RF_EXTI_DISABLED();

        /*
         * Entry action
         */
        rf_state = RF_STATE_CCA_BUSY;


        /*
         * Do actions
         */
        /* enter CCA operation */
        cca_mode = RF_CCA_MODE_RSSI_BELOW_THR;
        cc112x_spiRegWrite(CC112X_PKT_CFG2, &cca_mode, 1);

        /* Strobe RX */
        do {
            chip_status = cc112x_spiCmdStrobe(CC112X_SRX);
            if (chip_status & 0x60) {
                /* RX FIFO error then flush RX FIFO */
                chip_status = cc112x_spiCmdStrobe(CC112X_SFRX);
            }
        } while (RF_GET_CHIP_STATE(chip_status) != RF_CHIP_STATE_RX);


        do {
            cc112x_spiRegRead(CC112X_RSSI0, &rssi_status, 1);
        } while( (rssi_status & 0x02) == 0 );
        if ( (rssi_status & 0x04) )
            *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
        else
            *p_err = NETSTK_ERR_NONE;

        /*
         * Exit actions
         */
        /* reset CCA_MODE */
        cca_mode = RF_CCA_MODE_NONE;
        cc112x_spiRegWrite(CC112X_PKT_CFG2, &cca_mode, 1);

        if( e_rf_state_store == RF_STATE_IDLE )
            cc112x_gotoIdle();
        else
            /* put transceiver to RX */
            cc112x_gotoRx();
    }
}


static void cc112x_txPowerSet(int8_t power, e_nsErr_t *p_err)
{
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

static void cc112x_txPowerGet(int8_t *p_power, e_nsErr_t *p_err)
{
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
static void cc112x_chanNumSet(uint8_t chan_num, e_nsErr_t *p_err)
{
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
        rf_regVerify = write_byte;

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
static void cc112x_opModeSet(e_nsRfOpMode mode, e_nsErr_t *p_err)
{
    if (rf_opMode < NETSTK_RF_OP_MODE_MAX) {
        rf_opMode = mode;
        *p_err = NETSTK_ERR_NONE;
    } else {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    }
}


/*
********************************************************************************
*                               DRIVER DEFINITION
********************************************************************************
*/
const s_nsRF_t RFDrvCC112x =
{
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
