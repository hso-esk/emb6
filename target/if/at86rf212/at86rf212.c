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
 * \addtogroup at86rf212
 * @{
 */
/*! \file   at86rf212.c

    \author Artem Yushev 

    \brief  AT86RF212 Transceiver initialization code.

    \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#include "emb6.h"
#include "bsp.h"
#include "board_conf.h"
#include "at86rf212.h"
#include "at86rf212_regmap.h"
#include "at86rf_hal_spi.h"

#include "evproc.h"
#include "packetbuf.h"
#include "linkaddr.h"

/*==============================================================================
                                     MACROS
==============================================================================*/
#if (NETSTK_SUPPORT_HW_CRC != TRUE)
#error "missing or wrong radio checksum setting in board_conf.h"
#endif

#define     LOGGER_ENABLE        LOGGER_RADIO
#include    "logger.h"


#define bsp_clrPin(pin)           bsp_pinSet(pin, 0)
#define bsp_setPin(pin)           bsp_pinSet(pin, 1)
#define bsp_getPin(pin)           bsp_pinGet(pin)

#define move_ind(a)               a++; if (a >= RF212_CONF_RX_BUFFERS) a=0;
#define    move_head_ind()        move_ind(c_rxframe_head);
#define move_tail_ind()           move_ind(c_rxframe_tail);
/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                          VARIABLE DECLARATIONS
==============================================================================*/
st_rxframe_t                      gps_rxframe[RF212_CONF_RX_BUFFERS];
uint8_t                           pc_buffer[RF212_MAX_TX_FRAME_LENGTH + RF212_CHECKSUM_LEN];
static  uint8_t                   c_rxframe_head;
static  uint8_t                   c_rxframe_tail;
static  uint8_t                   c_receive_on;
static  uint8_t                   c_channel;
static  int8_t                    c_rssi_base_val = -100;
static  uint8_t                   c_last_correlation;
static  uint8_t                   c_last_rssi;
static  uint8_t                   c_smallest_rssi;
static  uint8_t                   c_rf212_pending;
static  uint8_t                   c_pckCounter = 0;
static  uint8_t                   c_power;
static  uint8_t                   c_sensitivity;
static  void *                    p_slpTrig = NULL;
static  void *                    p_rst = NULL;
static  void *                    p_irq = NULL;

/* Pointer to the PHY structure */
static  const s_nsPHY_t*          p_phy = NULL;

/*               Output Power                               dBm,  EU1,  EU2  */
static int16_t txpower[TXPWR_LIST_LEN][3] =    { {   5,    0, 0xe8 },
        {   4, 0x62, 0xe9 },
        {   3, 0x63, 0xea },
        {   2, 0x64, 0xeb },
        {   1, 0x65, 0xab },
        {   0, 0x66, 0xac },
        {  -1, 0x47, 0xad },
        {  -2, 0x48, 0x48 },
        {  -3, 0x28, 0x28 },
        {  -4, 0x29, 0x29 },
        {  -5, 0x2a, 0x2a },
        {  -6, 0x08, 0x08 },
        {  -7, 0x09, 0x09 },
        {  -8, 0x0a, 0x0a },
        {  -9, 0x0b, 0x0b },
        { -10, 0x0c, 0x0c },
        { -11, 0x0d, 0x0d }  };

extern uip_lladdr_t uip_lladdr;
/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/
/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
==============================================================================*/
/* Radio transceiver local functions */
static  void                    _rf212_fRead(st_rxframe_t * ps_rx_frame);
static  void                    _rf212_fWrite(uint8_t*  pc_write_buffer,
                                              uint8_t   c_length);
static  void                    _rf212_setChannel(uint8_t c);
static  void                    _rf212_smReset(void);
static  uint8_t                 _rf212_isIdle(void);
static  void                    _rf212_waitIdle(void);
static  uint8_t                 _rf212_getState(void);
static  e_rf212_sm_status_t     _rf212_setTrxState(uint8_t c_new_state);
static  e_radio_tx_status_t     _rf212_transmit(uint8_t c_len);
static  void                    _rf212_callback(c_event_t c_event,p_data_t p_data);
static  void                    _isr_callback(void *);
static  int8_t                  _rf212_prepare(const void * p_payload, uint8_t c_len);
static  int8_t                  _rf212_read(void *p_buf, uint8_t c_bufsize);
static  void                    _rf212_setPanAddr(unsigned pan,unsigned addr,
                                                  const uint8_t ieee_addr[8]);
static void                     _rf212_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);
static  int8_t                  _rf212_intON(void);
static  int8_t                  _rf212_intOFF(void);
static  void                    _rf212_extON(e_nsErr_t *p_err);
static  void                    _rf212_extOFF(e_nsErr_t *p_err);
static  uint8_t                 _rf212_getTxPower(void);
static  void                    _rf212_setTxPower(uint8_t power);
static  void                    _rf212_setPower(int8_t power);
static  int8_t                  _rf212_getPower(void);
static  void                    _rf212_setSensitivity(int8_t sens);
static  int8_t                  _rf212_getSensitivity(void);
static  int8_t                  _rf212_getRSSI(void);
static  void                    _rf212_wReset(void);
static  void                    _rf212_promisc(uint8_t value);
static  void                    _rf212_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static  void                    _rf212_init(void *p_netstk, e_nsErr_t *p_err);
static  void                    _spiBitWrite(uint8_t c_addr, uint8_t c_mask, uint8_t c_off,uint8_t c_data);

#if PRINT_PCK_STAT
static    void                     _show_stat(void *);
#endif /* PRINT_PCK_STAT */

/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/
const s_nsRF_t rf_driver_at212 = {
        .name               = "rf212",
        .init               = &_rf212_init,
        .send               = &_rf212_send,
        .on                 = &_rf212_extON,
        .off                = &_rf212_extOFF,
        .ioctrl             = &_rf212_Ioctl,
};
/*==============================================================================
                                LOCAL FUNCTIONS
==============================================================================*/
static void _spiBitWrite(uint8_t c_addr, uint8_t c_mask, uint8_t c_off, uint8_t c_data)
{
    uint8_t c_value = at86rf_halSpiRegRead(RF212_READ_COMMAND | c_addr);
    c_value &= ~c_mask;
    c_data <<= c_off;
    c_data &= c_mask;
    c_data |= c_value;
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | c_addr, c_data);
}



/*----------------------------------------------------------------------------*/
/** \brief  Transfer a frame from the radio transceiver to a RAM buffer
 *
 *          This version is optimized for use with RF212BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *          If the frame length is out of the defined bounds, the length, lqi and crc
 *          are set to zero.
 *
 *  \param  ps_rxframe    Pointer to the data structure where the frame is stored.
 */
static void _rf212_fRead(st_rxframe_t * ps_rxframe)
{
    uint16_t     c_flen = 0;

    /* CRC was checked in hardware, but redoing the checksum here ensures the rx buffer
     * is not being overwritten by the next packet. Since that lengthy computation makes
     * such overwrites more likely, we skip it and hope for the best.
     * Without the check a full buffer is read in 320us at 2x spi clocking.
     * The 802.15.4 standard requires 640us after a greater than 18 byte frame.
     * With a low interrupt latency overwrites should never occur.
     */
    at86rf_halSpiFrameRead(0x20, ps_rxframe->data, &c_flen);
    ps_rxframe->length = c_flen;
    /*Read LQI value for this frame.*/
    ps_rxframe->lqi = ps_rxframe->data[c_flen + 1];

    /* If crc was calculated set crc field in rx_frame_t accordingly.
     * Else show the crc has passed the hardware check.
     */
    ps_rxframe->crc   = TRUE;
    /*Check for correct frame length. Bypassing this test can result in a buffer overrun! */
    if ( (c_flen <= MIN_FRAME_LENGTH) || (c_flen > MAX_FRAME_LENGTH)) {
        /* Length test failed */
        ps_rxframe->length = 0;
        ps_rxframe->lqi    = 0;
        ps_rxframe->crc    = FALSE;
    }
} /*  _rf212_fRead() */

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  pc_buff            Pointer to data that is to be written to frame buffer.
 *  \param  c_len           Length of data. The maximum length is 127 bytes.
 */
static void _rf212_fWrite(uint8_t * pc_buff, uint8_t c_len)
{
    at86rf_halSpiFrameWrite(0x60, pc_buff, c_len);
} /*  _rf212_fWrite() */

/*----------------------------------------------------------------------------*/
/** \brief  This function set current operating channel.
 *
 */
static void     _rf212_setChannel(uint8_t c)
{
    /* Wait for any transmission to end. */
    _rf212_waitIdle();
    c_channel=c;
    _spiBitWrite(RG_PHY_CC_CCA, SR_CHANNEL, c);
} /*  _rf212_setChannel() */

/*----------------------------------------------------------------------------*/
/** \brief  This function will reset the state machine (to TRX_OFF) from any of
 *          its states, except for the SLEEP state.
 */
static void _rf212_smReset(void)
{
    bsp_clrPin(p_slpTrig);
    bsp_delayUs(E_TIME_NOCLK_TO_WAKE);
    _spiBitWrite(RG_TRX_STATE,SR_TRX_CMD, CMD_FORCE_TRX_OFF );
    bsp_delayUs(E_TIME_CMD_FORCE_TRX_OFF);
} /*  _rf212_smReset() */

/*----------------------------------------------------------------------------*/
/** \brief  This function return the status of a radio chip
 *
 *  \return radio idle state
 */
static uint8_t _rf212_isIdle(void)
{
    uint8_t c_rstate = 0;
    if (bsp_getPin(p_slpTrig)) {
        return 1;
    }
    else {
        c_rstate = at86rf_halSpiBitRead(RF212_READ_COMMAND | RG_TRX_STATUS, SR_TRX_STATUS);
        //        c_rstate = bsp_spiSubRead(SR_TRX_STATUS);
        if ((c_rstate != BUSY_TX_ARET) && \
                (c_rstate != BUSY_RX_AACK) && \
                //          (c_rstate != STATE_TRANSITION) &&
                (c_rstate != BUSY_RX) && \
                (c_rstate != BUSY_TX))
        {
            return 1;
        } else {
            return 0;
        }
    }
}  /*  _rf212_isIdle() */

/*----------------------------------------------------------------------------*/
/** \brief  This function waits until the transceiver will be idle
 *
 */
static void _rf212_waitIdle(void)
{
    int i;
    for (i=0;i<10000;i++) {  //to avoid potential hangs
        // while (1) {
        if (_rf212_isIdle())
            break;
    }
} /*  _rf212_waitIdle() */

/*----------------------------------------------------------------------------*/
/** \brief  This function return the Radio Transceivers current state.
 *
 *  \retval     P_ON               When the external supply voltage (VDD) is
 *                                 first supplied to the transceiver IC, the
 *                                 system is in the P_ON (Poweron) mode.
 *  \retval     BUSY_RX            The radio transceiver is busy receiving a
 *                                 frame.
 *  \retval     BUSY_TX            The radio transceiver is busy transmitting a
 *                                 frame.
 *  \retval     RX_ON              The RX_ON mode enables the analog and digital
 *                                 receiver blocks and the PLL frequency
 *                                 synthesizer.
 *  \retval     TRX_OFF            In this mode, the SPI module and crystal
 *                                 oscillator are active.
 *  \retval     PLL_ON             Entering the PLL_ON mode from TRX_OFF will
 *                                 first enable the analog voltage regulator. The
 *                                 transceiver is ready to transmit a frame.
 *  \retval     BUSY_RX_AACK       The radio was in RX_AACK_ON mode and received
 *                                 the Start of Frame Delimiter (SFD). State
 *                                 transition to BUSY_RX_AACK is done if the SFD
 *                                 is valid.
 *  \retval     BUSY_TX_ARET       The radio transceiver is busy handling the
 *                                 auto retry mechanism.
 *  \retval     RX_AACK_ON         The auto acknowledge mode of the radio is
 *                                 enabled and it is waiting for an incomming
 *                                 frame.
 *  \retval     TX_ARET_ON         The auto retry mechanism is enabled and the
 *                                 radio transceiver is waiting for the user to
 *                                 send the TX_START command.
 *  \retval     RX_ON_NOCLK        The radio transceiver is listening for
 *                                 incomming frames, but the CLKM is disabled so
 *                                 that the controller could be sleeping.
 *                                 However, this is only true if the controller
 *                                 is run from the clock output of the radio.
 *  \retval     RX_AACK_ON_NOCLK   Same as the RX_ON_NOCLK state, but with the
 *                                 auto acknowledge module turned on.
 *  \retval     BUSY_RX_AACK_NOCLK Same as BUSY_RX_AACK, but the controller
 *                                 could be sleeping since the CLKM pin is
 *                                 disabled.
 *  \retval     STATE_TRANSITION   The radio transceiver's state machine is in
 *                                 transition between two states.
 */
static uint8_t _rf212_getState(void)
{
    return at86rf_halSpiBitRead(RF212_READ_COMMAND | RG_TRX_STATUS, SR_TRX_STATUS);
    //    return bsp_spiSubRead(SR_TRX_STATUS);
} /*  _rf212_getState() */

/*----------------------------------------------------------------------------*/
/** \brief  This function will change the current state of the radio
 *          transceiver's internal state machine.
 *
 *  \param     new_state        Here is a list of possible states:
 *             - RX_ON        Requested transition to RX_ON state.
 *             - TRX_OFF      Requested transition to TRX_OFF state.
 *             - PLL_ON       Requested transition to PLL_ON state.
 *             - RX_AACK_ON   Requested transition to RX_AACK_ON state.
 *             - TX_ARET_ON   Requested transition to TX_ARET_ON state.
 *
 *  \retval    E_RADIO_SUCCESS          Requested state transition completed
 *                                  successfully.
 *  \retval    E_RADIO_INVALID_ARGUMENT Supplied function parameter out of bounds.
 *  \retval    E_RADIO_WRONG_STATE      Illegal state to do transition from.
 *  \retval    E_RADIO_BUSY_STATE       The radio transceiver is busy.
 *  \retval    E_RADIO_TIMED_OUT        The state transition could not be completed
 *                                  within resonable time.
 */
static e_rf212_sm_status_t _rf212_setTrxState(uint8_t c_new_state)
{
    uint8_t c_orig_state;
    e_rf212_sm_status_t e_status;

    /*Check function paramter and current state of the radio transceiver.*/
    if (!((c_new_state == TRX_OFF)    ||
            (c_new_state == RX_ON)      ||
            (c_new_state == PLL_ON)     ||
            (c_new_state == RX_AACK_ON) ||
            (c_new_state == TX_ARET_ON))){
        return E_RADIO_INVALID_ARGUMENT;
    }

    if (bsp_getPin(p_slpTrig)) {
        return E_RADIO_WRONG_STATE;
    }

    /* Wait for radio to finish previous operation */
    _rf212_waitIdle();
    //   for(;;)
    //   {
    c_orig_state = _rf212_getState();
    //      if (c_orig_state != BUSY_TX_ARET &&
    //          c_orig_state != BUSY_RX_AACK &&
    //          c_orig_state != BUSY_RX &&
    //          c_orig_state != BUSY_TX)
    //          break;
    //  }

    if (c_new_state == c_orig_state){
        return E_RADIO_SUCCESS;
    }


    /* At this point it is clear that the requested new_state is: */
    /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON. */

    /* The radio transceiver can be in one of the following states: */
    /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON, TX_ARET_ON. */

    if(c_new_state == TRX_OFF){
        _rf212_smReset(); /* Go to TRX_OFF from any state. */
    } else {
        /* It is not allowed to go from RX_AACK_ON or TX_AACK_ON and directly to */
        /* TX_AACK_ON or RX_AACK_ON respectively. Need to go via RX_ON or PLL_ON. */
        if ((c_new_state == TX_ARET_ON) &&
                (c_orig_state == RX_AACK_ON)){
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            /* The final state transition to TX_ARET_ON is handled after the if-else if. */
            _spiBitWrite(RG_TRX_STATE, SR_TRX_CMD, PLL_ON);
            //            bsp_spiSubWrite(SR_TRX_CMD, PLL_ON);
            bsp_delayUs(E_TIME_STATE_TRANSITION_PLL_ACTIVE);
        } else if ((c_new_state == RX_AACK_ON) &&
                (c_orig_state == TX_ARET_ON)){
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            /* The final state transition to RX_AACK_ON is handled after the if-else if. */
            _spiBitWrite(RG_TRX_STATE, SR_TRX_CMD, RX_ON);
            //            bsp_spiSubWrite(SR_TRX_CMD, RX_ON);
            bsp_delayUs(E_TIME_STATE_TRANSITION_PLL_ACTIVE);
        }

        /* Any other state transition can be done directly. */
        _spiBitWrite(RG_TRX_STATE, SR_TRX_CMD, c_new_state);
        //        bsp_spiSubWrite(SR_TRX_CMD, c_new_state);

        /* When the PLL is active most states can be reached in 1us. However, from */
        /* TRX_OFF the PLL needs time to activate. */
        if (c_orig_state == TRX_OFF){
            bsp_delayUs(E_TIME_TRX_OFF_TO_PLL_ACTIVE);
        } else {
            bsp_delayUs(E_TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
    } /*  end: if(new_state == TRX_OFF) ... */

    /*Verify state transition.*/
    e_status = E_RADIO_TIMED_OUT;

    if (_rf212_getState() == c_new_state){
        e_status = E_RADIO_SUCCESS;
    }
    return e_status;
} /*  _rf212_setTrxState() */

/*----------------------------------------------------------------------------*/
/** \brief  This function will copy data prepared for transmission
 *             into the frame buffer of at86rf212
 *
 *  \param     c_len        How much data should be transmitted.
 *
 *  \retval    RADIO_TX_OK                 Transmission was successfully done.
 *  \retval    RADIO_TX_COLLISION          Frame collision has happened (channek busy).
 *  \retval    RADIO_TX_NOACK           No acknowledgment received.
 *  \retval    RADIO_TX_ERR                Error while transmitting.
 */
static e_radio_tx_status_t _rf212_transmit(uint8_t c_len)
{
    int8_t                    c_txpower;
    uint8_t                 c_total_len;
    uint8_t                 c_tx_result;
    uint8_t                 c_ndx;
    /* If radio is sleeping we have to turn it on first */
    /* This automatically does the PLL calibrations */
    if (bsp_getPin(p_slpTrig)) {
        bsp_clrPin(p_slpTrig);
        bsp_delayUs(2*E_TIME_SLEEP_TO_TRX_OFF); //extra delay depends on board capacitance
        //    bsp_delayUs(E_TIME_SLEEP_TO_TRX_OFF+E_TIME_SLEEP_TO_TRX_OFF/2);
    } else {
#if RADIO_CONF_CALIBRATE_INTERVAL
        /* If nonzero, do periodic calibration. See clock.c */
        if (_rf212_calibrate) {
            _spiBitWrite( RG_PLL_CF, SR_PLL_CF_START, 1); //takes 80us max
            //            bsp_spiSubWrite(SR_PLL_CF_START,1);   //takes 80us max
            _spiBitWrite( RG_PLL_DCU, SR_PLL_DCU_START, 1); //takes 6us, concurrently
            //            bsp_spiSubWrite(SR_PLL_DCU_START,1); //takes 6us, concurrently
            _rf212_calibrate=0;
            _rf212_calibrated=1;
            bsp_delayUs(80); //?
        }
#endif
    }

    /* Wait for any previous operation or state transition to finish */
    _rf212_waitIdle();
    /* Prepare to transmit */
#if RF212_CONF_AUTORETRIES
    if ((c_tx_result = _rf212_setTrxState(TX_ARET_ON)) != E_RADIO_SUCCESS) {
        LOG_ERR("Change transceiver SM state - failed (code %02X)",c_tx_result);
        return RADIO_TX_ERR;
    }
#else
    _rf212_setTrxState(PLL_ON);
#endif

    c_txpower = 0;

    if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        /* Remember the current transmission power */
        c_txpower = _rf212_getTxPower();
        /* Set the specified transmission power */
        _rf212_setPower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
    }

    c_total_len = c_len + RF212_CHECKSUM_LEN;

    _rf212_fWrite(pc_buffer, c_total_len);
    /* Toggle the SLP_TR pin to initiate the frame transmission */
    bsp_setPin(p_slpTrig);
    bsp_clrPin(p_slpTrig);
    LOG_DBG("_rf212_transmit: %d", (int)c_total_len);

    /* We wait until transmission has ended so that we get an
     accurate measurement of the transmission time.*/
    _rf212_waitIdle();

    /* Get the transmission result */
#if RF212_CONF_AUTORETRIES
    c_tx_result = at86rf_halSpiBitRead(RF212_READ_COMMAND | RG_TRX_STATE, SR_TRAC_STATUS);
#else
    c_tx_result = RF212_TX_SUCCESS;
#endif

    /* Restore the transmission power */
    if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        _rf212_setTxPower(c_txpower & 0xff);
    }

    if(c_receive_on) {
        _rf212_intON();
    } else {
#if RADIOALWAYSON
        /* Enable reception */
        _rf212_intON();
#else
        LOG_DBG("_rf212_transmit: turning radio off");
        _rf212_intOFF();
#endif
    }

    LOG_RAW("_rf212_txFrame: ");
    for(c_ndx = 0; c_ndx < c_total_len; c_ndx++)
        LOG_RAW("%02x", pc_buffer[c_ndx]);
    LOG_RAW("\n\r");

#if RF212_INSERTACK
    ack_pending = 0;
#endif
    switch (c_tx_result) {
    case RF212_TX_SUCCESS:
    case RF212_TX_SUC_DPEND:
#if RF212_INSERTACK
        /* Not PAN broadcast to FFFF, and ACK was requested and received */
        if (!((pc_buffer[5]==0xff) && (pc_buffer[6]==0xff)) && (pc_buffer[0]&(1<<6)))
            ack_pending=1;
#endif
        return RADIO_TX_OK;
    case RF212_TX_CH_ACCFAIL:
        LOG_ERR("_rf212_transmit: CSMA channel access fail");
        return RADIO_TX_COLLISION;
    case RF212_TX_NO_ACK:
        LOG_ERR("_rf212_transmit: ACK not received");
        return RADIO_TX_NOACK;
    case RF212_TX_INVALID:
    default:
        LOG_ERR("_rf212_transmit: Invalid");
        return RADIO_TX_ERR;
    }
} /*  _rf212_transmit() */

/*----------------------------------------------------------------------------*/
/** \brief  This function prepares data for transmitting
 *
 *  \param         p_payload     Pointer to data to be transmitted
 *  \param        c_len        Data length
 *
 *  \retval        none
 */
/*---------------------------------------------------------------------------*/
static int8_t _rf212_prepare(const void * p_payload, uint8_t c_len)
{
    uint8_t     c_flen;
    uint8_t        *pc_buf;
#if RF212_CONF_CHECKSUM
    uint16_t checksum;
#endif
#if RF212_INSERTACK
    /* The sequence number is needed to construct the ack packet */
    ack_seqnum=*(((uint8_t *)payload)+2);
#endif

#if RF212_CONF_CHECKSUM
    checksum = crc16_data(payload, payload_len, 0);
#endif

    /* Copy payload to RAM buffer */
    c_flen = c_len + RF212_CHECKSUM_LEN;
    if (c_flen > RF212_MAX_TX_FRAME_LENGTH){
        LOG_DBG("packet too large (%d, max: %d)",c_flen,RF212_MAX_TX_FRAME_LENGTH);
        return 1;
    }
    pc_buf=&pc_buffer[0];
    memcpy(pc_buf,p_payload,c_len);
    pc_buf+=c_len;

#if RF212_CONF_CHECKSUM
    memcpy(pbuf,&checksum,CHECKSUM_LEN);
    pbuf+=CHECKSUM_LEN;
#endif
    /*------------------------------------------------------------*/

#ifdef RF212BB_HOOK_TX_PACKET
#if !RF212_CONF_CHECKSUM
    { // Add a checksum before we log the packet out
        uint16_t checksum;
        checksum = crc16_data(payload, payload_len, 0);
        memcpy(buffer+total_len-CHECKSUM_LEN,&checksum,CHECKSUM_LEN);
    }
#endif /* RF212_CONF_CHECKSUM */
    RF212BB_HOOK_TX_PACKET(buffer,total_len);
#endif

    return 0;
} /*  _rf212_prepare() */


/*----------------------------------------------------------------------------*/
/** \brief  This function reads packet that was uploaded from Radio in ISR
 *            he two-byte checksum is appended but the returned length does not include it.
 *             Frames are buffered in the interrupt routine so this routine
 *             does not access the hardware or change its status.
 *             However, this routine must be called with interrupts disabled to avoid ISR
 *             writing to the same buffer we are reading.
 *  \param         none
 *
 *  \return        0            In case of error
 *  \return        length        length of a read packet.
 */
/*---------------------------------------------------------------------------*/
static int8_t _rf212_read(void *p_buf, uint8_t c_bufsize)
{
    uint8_t     c_len;
    uint8_t        *pc_framep;
#if FOOTER_LEN
    uint8_t footer[FOOTER_LEN];
#endif
#if RF212_CONF_CHECKSUM
    uint16_t checksum;
#endif
#if RF212_INSERTACK
    /* Return an ACK to the mac layer */
    if(ack_pending && bufsize == 3){
        ack_pending=0;
        uint8_t *buff=(uint8_t *)buf;
        buff[0]=2;
        buff[1]=0;
        buff[2]=ack_seqnum;
        return bufsize;
    }
#endif

    /* The length includes the twp-byte checksum but not the LQI byte */
    c_len = gps_rxframe[c_rxframe_head].length;
    if (c_len==0) {
        LOG_ERR( "c_len is 0");
        move_head_ind();
        return 0;
    }

#if RADIOALWAYSON
    if (c_receive_on) {
#else
        if (!c_receive_on) {
            LOG_ERR("Radio txrx was switched off.");
            move_head_ind();
            return 0;
        }
#endif

        if(c_len > RF212_MAX_TX_FRAME_LENGTH) {
            /* Oops, we must be out of sync. */
            gps_rxframe[c_rxframe_head].length=0;
            LOG_ERR("Radio out of sync.");
            move_head_ind();
            return 0;
        }

        if(c_len <= RF212_CHECKSUM_LEN) {
            gps_rxframe[c_rxframe_head].length=0;
            LOG_ERR("C_LEN too small");
            move_head_ind();
            return 0;
        }

        if(c_len - RF212_CHECKSUM_LEN > c_bufsize) {
            gps_rxframe[c_rxframe_head].length=0;
            LOG_ERR("c_bufsize too small");
            move_head_ind();
            return 0;
        }
        /* Transfer the frame, stripping the footer, but copying the checksum */
        pc_framep=&(gps_rxframe[c_rxframe_head].data[0]);
        memcpy(p_buf,pc_framep,c_len - RF212_CHECKSUM_LEN + RF212_CHECKSUM_LEN);
        c_last_correlation = gps_rxframe[c_rxframe_head].lqi;

        /* Clear the length field to allow buffering of the next packet */
        gps_rxframe[c_rxframe_head].length=0;
        move_head_ind();
        //    /* If another packet has been buffered, schedule another receive poll */
        //    if (gps_rxframe[c_rxframe_head].length)
        //        evproc_putEvent(E_EVPROC_TAIL,EVENT_TYPE_PCK_LL,NULL);
        //        _rf212_callback();

        /* Point to the checksum */
        pc_framep += c_len - RF212_CHECKSUM_LEN;
#if RF212_CONF_CHECKSUM
        memcpy(&checksum,pc_framep,RF212_CHECKSUM_LEN);
#endif /* RF212_CONF_CHECKSUM */
        pc_framep += RF212_CHECKSUM_LEN;
        pc_framep += RF212_TIMESTAMP_LEN;
#if FOOTER_LEN
        memcpy(footer,framep,FOOTER_LEN);
#endif
#if RF212_CONF_CHECKSUM
        if(checksum != crc16_data(p_buf, c_len - RF212_CHECKSUM_LEN, 0)) {
            //PRINTF("checksum failed 0x%04x != 0x%04x\n\r",
            //  checksum, crc16_data(buf, len - AUX_LEN, 0));
        }
#if FOOTER_LEN
        if(footer[1] & FOOTER1_CRC_OK &&
                checksum == crc16_data(p_buf, c_len - RF212_CHECKSUM_LEN, 0)) {
#endif
#endif /* RF212_CONF_CHECKSUM */

            /* Get the received signal strength for the packet, 0-84 dB above rx threshold */
#if RF212_CONF_AUTOACK
            // _rf212_last_rssi = bsp_spiSubRead(SR_ED_LEVEL);  //0-84 resolution 1 dB
            c_last_rssi = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_PHY_ED_LEVEL); //0-84, resolution 1 dB

            //    c_last_rssi =bsp_spiRead(RG_PHY_ED_LEVEL);  //0-84, resolution 1 dB
#else
            /* last_rssi will have been set at RX_START interrupt */
            //  _rf212_last_rssi = 3*bsp_spiSubRead(SR_RSSI);    //0-28 resolution 3 dB
#endif

            /* Save the smallest rssi. The display routine can reset by setting it to zero */
            if ((c_smallest_rssi == 0) || (c_last_rssi < c_smallest_rssi))
                c_smallest_rssi = c_last_rssi;

            //   _rf212_last_correlation = rxframe[rxframe_head].lqi;
            packetbuf_set_attr(PACKETBUF_ATTR_RSSI, c_last_rssi);
            packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, c_last_correlation);

#if RF212_CONF_CHECKSUM
#if FOOTER_LEN
        } else {
            len = AUX_LEN;
        }
#endif
#endif


        /* Here return just the data length. The checksum is however still in the buffer for packet sniffing */
        return c_len - RF212_CHECKSUM_LEN;

#if RADIOALWAYSON
    } else { //Stack thought radio was off
        return 0;
    }
#endif
} /*  _rf212_read() */

/*----------------------------------------------------------------------------*/
/** \brief  This function assign a PAN address to a node
 *  \param         pan            Person area network identifier
 *  \param        addr        address
 *  \param        ieee_addr
 *
 *  \return        0            In case of error
 *  \return        length        length of a read packet.
 */
/*---------------------------------------------------------------------------*/
static void _rf212_setPanAddr(unsigned pan,    unsigned addr, const uint8_t prc_ieee_addr[8])
{
    LOG_INFO("PAN=%x Short Addr=%x",pan,addr);
    uint8_t c_abyte;
    c_abyte = pan & 0xFF;
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_PAN_ID_0, c_abyte);

    c_abyte = (pan >> 8*1) & 0xFF;
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_PAN_ID_1, c_abyte);


    c_abyte = addr & 0xFF;
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_SHORT_ADDR_0, c_abyte);

    c_abyte = (addr >> 8*1) & 0xFF;
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_SHORT_ADDR_1, c_abyte);


    if (prc_ieee_addr != NULL) {
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_7, *prc_ieee_addr++);
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_6, *prc_ieee_addr++);
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_5, *prc_ieee_addr++);
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_4, *prc_ieee_addr++);
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_3, *prc_ieee_addr++);
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_2, *prc_ieee_addr++);
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_1, *prc_ieee_addr++);
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IEEE_ADDR_0, *prc_ieee_addr++);
    }
} /* _rf212_setPanAddr() */

/*---------------------------------------------------------------------------*/
uint8_t _rf212_getTxPower(void)
{
    uint8_t pwr = TX_PWR_UNDEFINED;
    if (bsp_getPin(p_slpTrig)) {
        LOG_DBG("_rf212_getTxPower:Sleeping");
    } else {
        pwr = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_PHY_TX_PWR);
    }
    return pwr;
} /* _rf212_getTxPower() */


/*---------------------------------------------------------------------------*/
void _rf212_setTxPower(uint8_t power)
{
    uint8_t i, pwr = 0;
    for (i=0;i<TXPWR_LIST_LEN;i++)
    {
        if (txpower[i][TXPWR_BAND] == power)
        {
            pwr = 1;
            break;
        }
    }
    if (pwr == 0)
    {
        power = 0x48;
    }
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_PHY_TX_PWR, power);
    c_power = power;
} /* _rf212_setTxPower() */

void _rf212_setPower(int8_t power)
{
    uint8_t pwr = 0x48, i;
    if (power > TX_PWR_MAX) {
        power = TX_PWR_MAX;
    } else if (power < TX_PWR_MIN) {
        power = TX_PWR_MIN;
    }
    for (i=0;i<TXPWR_LIST_LEN;i++)
    {
        if (txpower[i][TXPWR_DBM] == power)
        {
            pwr = txpower[i][TXPWR_BAND];
            break;
        }
    }
    _rf212_setTxPower(pwr);
}

int8_t _rf212_getPower(void)
{
    uint8_t i;
    for (i=0;i<TXPWR_LIST_LEN;i++)
    {
        if (txpower[i][TXPWR_BAND] == c_power)
        {
            return txpower[i][TXPWR_DBM];
        }
    }
    return -99;
}

void _rf212_setSensitivity(int8_t sens)
{
    int8_t s = 0;
    s = (sens - c_rssi_base_val) / 3.1;
    if (s > 15)
    {
        s = 15;
    } else if (s < 1) {
        s = 0;
    }
    _spiBitWrite(RG_RX_SYN, SR_RX_PDT_LEVEL, s);
    c_sensitivity = s;
}

int8_t _rf212_getSensitivity(void)
{
    return (c_rssi_base_val + 3.1 * c_sensitivity);
}

int8_t _rf212_getRSSI(void)
{
    return (c_rssi_base_val + 1.03 * c_last_rssi);
}

/*==============================================================================
  radio_wreset()
 =============================================================================*/
void _rf212_wReset(void)
{
    uint8_t c_tempReg;
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IRQ_MASK, RF212_SUPPORTED_INTERRUPT_MASK);

    /* Set up number of automatic retries 0-15 (0 implies PLL_ON sends instead of the extended TX_ARET mode */
    _spiBitWrite(RG_XAH_CTRL_0, SR_MAX_FRAME_RETRIES, RF212_CONF_AUTORETRIES);

    /* Set up carrier sense/clear c_channel assesment parameters for extended operating mode */
    _spiBitWrite(RG_XAH_CTRL_0, SR_MAX_CSMA_RETRIES, 5);  //highest allowed retries
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_CSMA_BE, 0x80); //min backoff exponent 0, max 8 (highest allowed)
    c_tempReg = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_PHY_RSSI);
    at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_CSMA_SEED_0, c_tempReg); //upper two RSSI reg bits RND_VALUE are random

    /* CCA Mode Mode 1=Energy above threshold  2=Carrier sense only  3=Both 0=Either (RF231 only) */
    //bsp_spiSubWrite(SR_E_CCA_MODE,1);  //1 is the power-on default

    /* Carrier sense threshold (not implemented in RF212 or RF231) */
    // bsp_spiSubWrite(SR_E_CCA_CS_THRES,1);

    /* set initial sensitivity */
    _rf212_setSensitivity(mac_phy_config.init_sensitivity);

    /* CCA energy threshold = -91dB + 2*SR_E_CCA_ED_THRESH. Reset defaults to -77dB */
    /* Use RF212 base of -91;  RF231 base is -90 according to datasheet */
#ifdef RF212_CONF_E_CCA_THRES
#if RF212_CONF_E_CCA_THRES < -91
#warning RF212_CONF_E_CCA_THRES below hardware limit, setting to -91dBm
    _spiBitWrite(RG_E_CCA_THRES, SR_E_CCA_ED_THRES, 0);
    //  bsp_spiSubWrite(SR_E_CCA_ED_THRES,0);
#elif RF212_CONF_E_CCA_THRES > -61
#warning RF212_CONF_E_CCA_THRES above hardware limit, setting to -61dBm
    _spiBitWrite(RG_E_CCA_THRES, SR_E_CCA_ED_THRES, 15);
    //  bsp_spiSubWrite(SR_E_CCA_ED_THRES,15);
#else
    _spiBitWrite(RG_E_CCA_THRES, SR_E_CCA_ED_THRES, (RF212_CONF_E_CCA_THRES+91)/2);
    //  bsp_spiSubWrite(SR_E_CCA_ED_THRES,(RF212_CONF_E_CCA_THRES+91)/2);
#endif
#endif

    /* Use automatic CRC unless manual is specified */
#if RF212_CONF_CHECKSUM
    //  bsp_spiSubWrite(SR_TX_AUTO_CRC_ON, 0);
    _spiBitWrite(RG_TRX_CTRL_1, SR_TX_AUTO_CRC_ON, 0);
#else
    _spiBitWrite(RG_TRX_CTRL_1, SR_TX_AUTO_CRC_ON, 1);
#endif

    /* set wireless mode */
    if (mac_phy_config.modulation != MODULATION_QPSK100)
    {
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_TRX_CTRL_2, 0x00);
        c_rssi_base_val = -100;
    } else {
        at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_TRX_CTRL_2, 0x08);
        c_rssi_base_val = -98;
    }

    /* set initial tx power */
    _rf212_setPower(mac_phy_config.init_power);


} /* _rf212_wReset() */

static void _rf212_promisc(uint8_t value)
{
    uint8_t ac_addr[8];
    if (value) {
        memset(&ac_addr, 0, 8);
        _rf212_setPanAddr(0x0000, 0, ac_addr);
        _spiBitWrite(SR_AACK_PROM_MODE, 1);
        _spiBitWrite(SR_AACK_DIS_ACK, 1);
    }
    else {
        _spiBitWrite(SR_AACK_PROM_MODE, 0);
        _spiBitWrite(SR_AACK_DIS_ACK, 0);
    }
}

/*---------------------------------------------------------------------------*/
static void _rf212_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    int8_t c_ret;
    if( len > 255 ) {
        *p_err = NETSTK_ERR_RF_SEND;
        return;
    }


    if((c_ret = _rf212_prepare(p_data, len))) {
        LOG_ERR("_rf212_send: Unable to send, prep failed (%d)",c_ret);
        *p_err = NETSTK_ERR_RF_SEND;
    }

    switch( _rf212_transmit(len) ) {
        case RADIO_TX_OK:
            *p_err = NETSTK_ERR_NONE;
            break;

        case RADIO_TX_NOACK:
            *p_err = NETSTK_ERR_TX_NOACK;
            break;

        default:
            *p_err = NETSTK_ERR_RF_SEND;
            break;
    }

    return;
} /* _rf212_send() */

static void _rf212_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == (e_nsErr_t *)0) {
        return;
    }

    if (p_val == (void *)0) {
        return;
    }
#endif


    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_RF_TXPOWER_SET:
            _rf212_setPower( *(int8_t*)p_val );
            break;

        case NETSTK_CMD_RF_TXPOWER_GET:
            *(int8_t*)p_val = _rf212_getPower();
            break;

        case NETSTK_CMD_RF_RSSI_GET:
            *((int8_t *)p_val) = _rf212_getRSSI();
            break;

        case NETSTK_CMD_RF_SENS_SET:
            _rf212_setSensitivity( *(int8_t*)p_val );
            break;

        case NETSTK_CMD_RF_SENS_GET:
            *((int8_t *)p_val) = _rf212_getSensitivity();
            break;

        case NETSTK_CMD_RF_PROMISC_SET:
            _rf212_promisc( *(uint8_t*)p_val );
            break;

        case NETSTK_CMD_RF_IS_RX_BUSY:
        case NETSTK_CMD_RF_IS_TX_BUSY:
        case NETSTK_CMD_RF_CCA_GET:
        case NETSTK_CMD_RF_RF_SWITCH_SET:
        case NETSTK_CMD_RF_ANT_DIV_SET:
        default:
            /* unsupported commands are treated in same way */
            *p_err = NETSTK_ERR_CMD_UNSUPPORTED;
            break;
    }
}

/*----------------------------------------------------------------------------*/
/** \brief  This function turns on the radio transceiver
 *
 *    \param    none
 *
 *    \retval    none
 */
/*----------------------------------------------------------------------------*/
static int8_t _rf212_intON(void)
{
    c_receive_on = 1;
    /* If radio is off (slptr high), turn it on */
    if (bsp_getPin(p_slpTrig)) {
#if RF212BB_CONF_LEDONPORTE1
        PORTE|=(1<<PE1); //ledon
#endif
        /* SPI based radios. The wake time depends on board capacitance.
         * Make sure the delay is long enough, as using SPI too soon will reset the MCU!
         * Use 2x the nominal value for safety. 1.5x is not long enough for Raven!
         */
        //  uint8_t sreg = SREG;cli();
        bsp_clrPin(p_slpTrig);
        bsp_delayUs(2*E_TIME_SLEEP_TO_TRX_OFF);
        //  bsp_delayUs(E_TIME_SLEEP_TO_TRX_OFF+E_TIME_SLEEP_TO_TRX_OFF/2);
        //  SREG=sreg;
    }

#if RF212_CONF_AUTOACK
    // _rf212_setTrxState(is_promiscuous?RX_ON:RX_AACK_ON);
    if (_rf212_setTrxState(RX_AACK_ON) != E_RADIO_SUCCESS)
        printf("Aack set failed\n\r");
#else
    _rf212_setTrxState(RX_ON);
#endif
    _rf212_waitIdle();
    return 1;
} /* _rf212_intON() */

/*---------------------------------------------------------------------------*/
static int8_t _rf212_intOFF(void)
{
    c_receive_on = 0;
#if RF212BB_CONF_LEDONPORTE1
    PORTE&=~(1<<PE1); //ledoff
#endif
#ifdef RF212BB_HOOK_RADIO_OFF
    RF212BB_HOOK_RADIO_OFF();
#endif

    /* Wait for any transmission to end */
    _rf212_waitIdle();

#if RADIOALWAYSON
    /* Do not transmit autoacks when stack thinks radio is off */
    _rf212_setTrxState(RX_ON);
#else
    /* Force the device into TRX_OFF. */
    _rf212_smReset();
#if RADIOSLEEPSWHENOFF
    /* Sleep Radio */
    bsp_setPin(SLP_TR_PORT,SLP_TR_PIN);
    ENERGEST_OFF(ENERGEST_TYPE_LED_RED);
#endif
#endif /* RADIOALWAYSON */

    return 0;
} /* _rf212_off() */

static void _rf212_extON(e_nsErr_t *p_err)
{
    *p_err = NETSTK_ERR_NONE;
    if (c_receive_on)
        return;
    _rf212_intON();
    return;
} /* _rf212_extON() */

static void _rf212_extOFF(e_nsErr_t *p_err)
{
    *p_err = NETSTK_ERR_NONE;
    if (c_receive_on == 0)
        return;

    /*
     * If we are currently receiving a packet, we still call off(),
     * as that routine waits until Rx is complete (packet uploaded in ISR
     * so no worries about losing it). If using RX_AACK_MODE, chances are
     * the packet is not for us and will be discarded.
     */
    if (!_rf212_isIdle()) {
        LOG_INFO("_rf212_extOFF: busy receiving.");
    }
    _rf212_intOFF();
    return;
} /* _rf212_extON() */

//#if PRINT_PCK_STAT
//static void _show_stat(void * ptr)
//{
//    printf("%lu<-%lu<-%lu\n\r",bsp_getTick(),pck_cntr_in,pck_cntr_out);
//    pck_cntr_in=0;
//    pck_cntr_out=0;
//    ctimer_restart(&pckstat_ct);
//}
//#endif /* PRINT_PCK_STAT */

/*==============================================================================
  _rf212_init()
 =============================================================================*/
static void _rf212_init(void *p_netstk, e_nsErr_t *p_err)
{
    uint8_t     i;
    uint8_t     c_tvers;
    uint8_t     c_tmanu;
    linkaddr_t  un_addr;

    *p_err = NETSTK_ERR_NONE;

    /* Wait in case VCC just applied */
    bsp_delayUs(E_TIME_TO_ENTER_P_ON);
    /* Init spi interface and transceiver. Transceiver utilize spi interface. */
    p_rst = bsp_pinInit( EN_HAL_PIN_RFCTRL0);
    p_slpTrig = bsp_pinInit( EN_HAL_PIN_RFCTRL1);
    p_irq = bsp_pinInit( EN_HAL_PIN_RFCTRL2);
    at86rf_halSpiInit();

    if ((p_rst != NULL) && (p_slpTrig != NULL) && (p_netstk != NULL))
    {
        /* configure external interrupt */
        hal_pinIRQRegister(p_irq, EN_HAL_IRQEDGE_RISING, _isr_callback);
        hal_pinIRQEnable(p_irq);

        /* Set receive buffers empty and point to the first */
        for (i=0;i<RF212_CONF_RX_BUFFERS;i++)
            gps_rxframe[i].length=0;
        c_rxframe_head=0;
        c_rxframe_tail=0;


        bsp_clrPin(p_rst);
        bsp_clrPin(p_slpTrig);
        /* On powerup a E_TIME_RESET delay is needed here, however on some other MCU reset
         * (JTAG, WDT, Brownout) the radio may be sleeping. It can enter an uncertain
         * state (sending wrong hardware FCS for example) unless the full wakeup delay
         * is done.
         * Wake time depends on board capacitance; use 2x the nominal delay for safety.
         * See www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=78725
         */
        bsp_delayUs(2*E_TIME_SLEEP_TO_TRX_OFF);
        bsp_setPin(p_rst);

        /* Force transition to TRX_OFF */
        _spiBitWrite(RG_TRX_STATE, SR_TRX_CMD, CMD_FORCE_TRX_OFF);
        bsp_delayUs(E_TIME_P_ON_TO_TRX_OFF);

        /* Verify that it is a supported version */
        /* Note gcc optimizes this away if DEBUG is not set! */
        //ATMEGA128RFA1 - version 4, ID 31
        c_tvers = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_VERSION_NUM);
        c_tmanu = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_MAN_ID_0);

        if ((c_tvers != RF212_REVA) && (c_tvers != RF212_REVB)) {
            LOG_INFO("Unsupported version %u",c_tvers);
            *p_err = NETSTK_ERR_INIT;
            goto error;
        }
        if (c_tmanu != SUPPORTED_MANUFACTURER_ID) {
            LOG_INFO("Unsupported manufacturer ID %u",c_tmanu);
            *p_err = NETSTK_ERR_INIT;
            goto error;
        }

        LOG_INFO("Version %u, ID %u",c_tvers,c_tmanu);

        _rf212_wReset();
        /* Leave radio in on state (?)*/
        _rf212_intON();
        //_spiBitWrite(SR_AACK_PROM_MODE, 1);
        if (mac_phy_config.mac_address == NULL) {
            *p_err = NETSTK_ERR_INIT;
        }
        else {
            memcpy((void *)&un_addr.u8,  &mac_phy_config.mac_address, 8);
            memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
            _rf212_setPanAddr(mac_phy_config.pan_id, 0, (uint8_t *)&un_addr.u8);
            linkaddr_set_node_addr(&un_addr);
            _rf212_setChannel(CHANNEL_802_15_4);

            LOG_INFO("MAC address %x:%x:%x:%x:%x:%x:%x:%x",    \
                    un_addr.u8[0],un_addr.u8[1],\
                    un_addr.u8[2],un_addr.u8[3],\
                    un_addr.u8[4],un_addr.u8[5],\
                    un_addr.u8[6],un_addr.u8[7]);

            evproc_regCallback(EVENT_TYPE_PCK_LL,_rf212_callback);
            if (((s_ns_t*)p_netstk)->phy != NULL) {
                p_phy = ((s_ns_t*)p_netstk)->phy;
                *p_err = NETSTK_ERR_NONE;
            } else {
                *p_err = NETSTK_ERR_INIT;
            }
        }
    }
    error:
    return;
} /* _rf212_init() */

/*----------------------------------------------------------------------------*/
/** \brief  This function is called from ISR and it reads packet form
 *             the frame buffer
 *
 *  \param     none
 *
 *  \retval    none
 */
/*---------------------------------------------------------------------------*/
static void _rf212_callback(c_event_t c_event, p_data_t p_data)
{
    int8_t c_len;
    uint8_t c_ndx;
    e_nsErr_t s_err;

    c_rf212_pending = 0;
    // The case where c_pckCounter is less or equal to 0 is not possible, however...
    c_pckCounter = (c_pckCounter > 0) ? (c_pckCounter -1) : c_pckCounter;

    //c_pckCounter = (c_pckCounter > 0)?c_pckCounter--:c_pckCounter;
    packetbuf_clear();

    /* Turn off interrupts to avoid ISR writing to the same buffers we are reading. */
    bsp_enterCritical();

    LOG_RAW("_rf212_rxFrame: ");
    for(c_ndx = 0; c_ndx < gps_rxframe[c_rxframe_head].length; c_ndx++)
        LOG_RAW("%02x", gps_rxframe[c_rxframe_head].data[c_ndx]);
    LOG_RAW("\n\r");

    c_len = _rf212_read(packetbuf_dataptr(), PACKETBUF_SIZE);

    /* Restore interrupts. */
    bsp_exitCritical();
    LOG_DBG("%u bytes lqi %u",c_len,c_last_correlation);

    if((c_len > 0) && (p_phy != NULL)) {
        s_err = NETSTK_ERR_NONE;
        packetbuf_set_datalen(c_len);
        p_phy->recv( packetbuf_dataptr(), c_len, &s_err );
    }
} /*  _rf212_callback() */

/*==============================================================================
                                 INTERRUPTS HANDLER FUNCTIONS
==============================================================================*/
/*----------------------------------------------------------------------------*/
/** \brief  This function handles INT5 global interrupt request from radiotxrx
 *
 *    \param    none
 *
 *    \retval    none
 */
/*----------------------------------------------------------------------------*/
void _isr_callback(void * p_input)
{
    uint8_t c_state;
    uint8_t c_int_src; /* used after bsp_spiTranOpen/CLOSE block */
    uint8_t c_isr_mask;
    //    if (c_pckCounter<RF212_CONF_RX_BUFFERS)
    //    {
    //        c_pckCounter++;
    //    }
    c_pckCounter = (c_pckCounter < RF212_CONF_RX_BUFFERS) ? (c_pckCounter + 1) : c_pckCounter;
    /* Using SPI bus from ISR is generally a bad idea... */
    c_int_src = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_IRQ_STATUS);
    /* Note: all IRQ are not always automatically disabled when running in ISR */
    /*Handle the incomming interrupt. Prioritized.*/
    //    printf("Int source = %d\n\r",c_int_src);
    if ((c_int_src & RX_START_MASK)){
#if !RF212_CONF_AUTOACK
        bsp_spiTxRx(RF212_READ_COMMAND | SR_RSSI,  &c_last_rssi);
        c_last_rssi *= 3;
        //        c_last_rssi = 3 * bsp_spiSubRead(SR_RSSI);
#endif
    } else if (c_int_src & TRX_END_MASK){
        c_state = at86rf_halSpiBitRead(RF212_READ_COMMAND | RG_TRX_STATUS, SR_TRX_STATUS);
        //        c_state = bsp_spiSubRead(SR_TRX_STATUS);
        if( (c_state == BUSY_RX_AACK) || \
                (c_state == RX_ON) ||          \
                (c_state == BUSY_RX) ||      \
                (c_state == RX_AACK_ON))
        {
            /* Received packet interrupt */
            /* Buffer the frame and call _rf212_int to schedule poll for rf212 receive process */
            /* Save the rssi for printing in the main loop */
#ifdef RF212_MIN_RX_POWER
#if RF212_CONF_AUTOACK
            c_last_rssi = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_PHY_ED_LEVEL);
#endif
            if (c_last_rssi >= RF212_MIN_RX_POWER) {
#endif
                _rf212_fRead(&gps_rxframe[c_rxframe_tail]);
                move_tail_ind();
                if (c_receive_on /* && (c_pckCounter < RF212_CONF_RX_BUFFERS) */)
                    evproc_putEvent(E_EVPROC_HEAD,EVENT_TYPE_PCK_LL,NULL);

            }
        } else if (c_int_src & TRX_UR_MASK){
        } else if (c_int_src & PLL_UNLOCK_MASK){
        } else if (c_int_src & PLL_LOCK_MASK){
        } else if (c_int_src & BAT_LOW_MASK){
            /*  Disable BAT_LOW interrupt to prevent endless interrupts. The interrupt */
            /*  will continously be asserted while the supply voltage is less than the */
            /*  user-defined voltage threshold. */
            c_isr_mask = at86rf_halSpiRegRead(RF212_READ_COMMAND | RG_IRQ_MASK);
            //        uint8_t c_isr_mask = bsp_spiRead(RG_IRQ_MASK);
            c_isr_mask &= ~BAT_LOW_MASK;
            at86rf_halSpiRegWrite(RF212_WRITE_COMMAND | RG_IRQ_MASK, c_isr_mask);
            //        bsp_spiWrite(RG_IRQ_MASK, c_isr_mask);
        }
    }

    /** @} */
