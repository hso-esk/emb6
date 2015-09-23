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
 *   \addtogroup at86rf230
 *   @{
*/
/*! \file   at86rf230.c

    \author Artem Yushev 

    \brief  AT86RF230 Transceiver initialization code.

    \version 0.0.1
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#include "emb6.h"
#include "emb6_conf.h"
#include "bsp.h"

#include "at86rf212b.h"
#include "at86rf212b_regmap.h"

#include "evproc.h"
#include "packetbuf.h"

/*==============================================================================
                                     MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_RADIO
#include    "logger.h"

#define     bsp_clrPin(pin)                        bsp_pin(E_BSP_PIN_CLR, pin)
#define     bsp_setPin(pin)                        bsp_pin(E_BSP_PIN_SET, pin)


/** Read register command for at86rfxxx. Must be applied as OR argument */
#define RF230_READ_COMMAND                    0x80

/** Write register command for at86rfxxx. Must be applied as OR argument */
#define RF230_WRITE_COMMAND                    0xC0

/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                          VARIABLE DECLARATIONS
==============================================================================*/
        st_rxframe_t             gps_rxframe[RF230_CONF_RX_BUFFERS];
        uint8_t                 pc_buffer[RF230_MAX_TX_FRAME_LENGTH + RF230_CHECKSUM_LEN];
static    uint8_t                 c_rxframe_head;
static    uint8_t                 c_rxframe_tail;
static    uint8_t                 c_receive_on = 0;
static uint8_t                     c_channel;
static    uint8_t                 c_last_correlation;
static    uint8_t                    c_last_rssi;
static    uint8_t                    c_smallest_rssi;
static    uint8_t                    c_rf230_pending;
static    void *                     p_spi = NULL;
static    void *                     p_slpTrig = NULL;
static     void *                     p_rst = NULL;
/* Pointer to the lmac structure */
static  s_nsLowMac_t*            p_lmac = NULL;

/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/
const uint8_t mac_address[8] = {RADIO_MAC_ADDR};
/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
==============================================================================*/
/* Radio transceiver local functions */
static void                     _rf230_fRead(st_rxframe_t * ps_rx_frame);
static void                        _rf230_fWrite(uint8_t * pc_write_buffer, uint8_t c_length);
static void                     _rf230_setChannel(uint8_t c);
static void                     _rf230_smReset(void);
static uint8_t                     _rf230_isIdle(void);
static void                     _rf230_waitIdle(void);
static uint8_t                     _rf230_getState(void);
static e_rf230_sm_status_t        _rf230_setTrxState(uint8_t c_new_state);
static e_radio_tx_status_t         _rf230_transmit(uint8_t c_len);
static void                     _rf230_callback(c_event_t c_event, p_data_t p_data);
static void                     _isr_callback(void * p_input);
static int8_t                     _rf230_prepare(const void * p_payload, uint8_t c_len);
static int8_t                     _rf230_read(void *p_buf, uint8_t c_bufsize);
static    void                     _rf230_setPanAddr(unsigned pan,unsigned addr,const uint8_t ieee_addr[8]);
static    int8_t                    _rf230_intON(void);
static    int8_t                    _rf230_intOFF(void);
static    int8_t                     _rf230_extON(void);
static    int8_t                     _rf230_extOFF(void);
static    uint8_t                 _rf230_getTxPower(void);
static    void                     _rf230_setTxPower(uint8_t c_power);
static    void                     _rf230_wReset(void);
static  void                    _rf239_promisc(void);
static    int8_t                     _rf230_send(const void *pr_payload, uint8_t c_len);
static    int8_t                     _rf230_init(s_ns_t* ns);
static    void                     _spiBitWrite(void * p_spi, uint8_t c_addr,uint8_t c_mask,
                                                                uint8_t c_off,uint8_t c_data);
/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/
const s_nsIf_t rf230_driver = {
        "at86rf30b",
        _rf230_init,
        _rf230_send,
        _rf230_extON,
        _rf230_extOFF,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        _rf230_promisc,
};
/*==============================================================================
                                LOCAL FUNCTIONS
==============================================================================*/
static    void _spiBitWrite(void * p_spi,uint8_t c_addr,uint8_t c_mask, uint8_t c_off,uint8_t c_data)
{
    uint8_t c_value = bsp_spiRegRead(p_spi,RF230_READ_COMMAND | c_addr);
    c_value &= ~c_mask;
    c_data <<= c_off;
    c_data &= c_mask;
    c_data |= c_value;
    bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | c_addr, c_data);
}


/*----------------------------------------------------------------------------*/
/** \brief  Transfer a frame from the radio transceiver to a RAM buffer
 *
 *          This version is optimized for use with RF230BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *          If the frame length is out of the defined bounds, the length, lqi and crc
 *          are set to zero.
 *
 *  \param  ps_rxframe    Pointer to the data structure where the frame is stored.
 */
static void _rf230_fRead(st_rxframe_t * ps_rxframe)
{
    uint8_t *     pc_rxdata = NULL;
    uint16_t     c_flen = 0;

    pc_rxdata = (ps_rxframe->data);


    /* CRC was checked in hardware, but redoing the checksum here ensures the rx buffer
     * is not being overwritten by the next packet. Since that lengthy computation makes
     * such overwrites more likely, we skip it and hope for the best.
     * Without the check a full buffer is read in 320us at 2x spi clocking.
     * The 802.15.4 standard requires 640us after a greater than 18 byte frame.
     * With a low interrupt latency overwrites should never occur.
     */
    bsp_spiFrameRead(p_spi, 0x20, pc_rxdata, &c_flen);

    ps_rxframe->length = c_flen;
    /*Read LQI value for this frame.*/
    ps_rxframe->lqi = pc_rxdata[c_flen + 1];

    /* If crc was calculated set crc field in rx_frame_t accordingly.
     * Else show the crc has passed the hardware check.
     */
    ps_rxframe->crc   = TRUE;
    /*Check for correct frame length. Bypassing this test can result in a buffer overrun! */
    if ( (c_flen <= MIN_FRAME_LENGTH) || (c_flen >= MAX_FRAME_LENGTH)) {
        /* Length test failed */
        ps_rxframe->length = 0;
        ps_rxframe->lqi    = 0;
        ps_rxframe->crc    = FALSE;
    }
} /*  _rf230_fRead() */

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  pc_buff            Pointer to data that is to be written to frame buffer.
 *  \param  c_len           Length of data. The maximum length is 127 bytes.
 */
static void _rf230_fWrite(uint8_t * pc_buff, uint8_t c_len)
{
   /* Optionally truncate length to maximum frame length.
     * Not doing this is a fast way to know when the application needs fixing!
     */
//  length &= 0x7f;

    /* Send Frame Transmit (long mode) command and frame length */
//    bsp_spiRegWrite(p_spi, 0x60, c_len);

    /* Download to the Frame Buffer.
     * When the FCS is autogenerated there is no need to transfer the last two bytes
     * since they will be overwritten.
     */
//#if !RF230_CONF_CHECKSUM
//        c_len -= 2;
//#endif
    bsp_spiFrameWrite(p_spi, 0x60, pc_buff, c_len);
} /*  _rf230_fWrite() */

/*----------------------------------------------------------------------------*/
/** \brief  This function set current operating channel.
 *
 */
static void     _rf230_setChannel(uint8_t c)
{
    /* Wait for any transmission to end. */
    _rf230_waitIdle();
    c_channel=c;
    _spiBitWrite(p_spi, RG_PHY_CC_CCA, SR_CHANNEL, c);
} /*  _rf230_setChannel() */

/*----------------------------------------------------------------------------*/
/** \brief  This function will reset the state machine (to TRX_OFF) from any of
 *          its states, except for the SLEEP state.
 */
static void _rf230_smReset(void)
{
    bsp_clrPin(p_slpTrig);
    bsp_delay_us(E_TIME_NOCLK_TO_WAKE);
    _spiBitWrite(p_spi, RG_TRX_STATE,SR_TRX_CMD, CMD_FORCE_TRX_OFF );
    bsp_delay_us(E_TIME_CMD_FORCE_TRX_OFF);
} /*  _rf230_smReset() */

/*----------------------------------------------------------------------------*/
/** \brief  This function return the status of a radio chip
 *
 *  \return radio idle state
 */
static uint8_t _rf230_isIdle(void)
{
    uint8_t c_rstate = 0;
    if (bsp_pin(E_BSP_PIN_GET, p_slpTrig)) {
        return 1;
    }
    else {
        c_rstate = bsp_spiBitRead(p_spi, RF230_READ_COMMAND | RG_TRX_STATUS, SR_TRX_STATUS);
//        c_rstate = bsp_spiSubRead(SR_TRX_STATUS);
        if ((c_rstate != BUSY_TX_ARET) && \
          (c_rstate != BUSY_RX_AACK) && \
          (c_rstate != STATE_TRANSITION) && \
          (c_rstate != BUSY_RX) && \
          (c_rstate != BUSY_TX))
        {
            return 1;
        } else {
            return 0;
        }
    }
}  /*  _rf230_isIdle() */

/*----------------------------------------------------------------------------*/
/** \brief  This function waits until the transceiver will be idle
 *
 */
static void _rf230_waitIdle(void)
{
    int i;
    for (i=0;i<10000;i++) {  //to avoid potential hangs
        // while (1) {
        if (_rf230_isIdle())
            break;
    }
} /*  _rf230_waitIdle() */

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
static uint8_t _rf230_getState(void)
{
    return bsp_spiBitRead(p_spi, RF230_READ_COMMAND | RG_TRX_STATUS, SR_TRX_STATUS);
//    return bsp_spiSubRead(SR_TRX_STATUS);
} /*  _rf230_getState() */

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
static e_rf230_sm_status_t _rf230_setTrxState(uint8_t c_new_state)
{
    uint8_t c_orig_state;
    e_rf230_sm_status_t e_status;

    /*Check function paramter and current state of the radio transceiver.*/
    if (!((c_new_state == TRX_OFF)    ||
          (c_new_state == RX_ON)      ||
          (c_new_state == PLL_ON)     ||
          (c_new_state == RX_AACK_ON) ||
          (c_new_state == TX_ARET_ON))){
        return E_RADIO_INVALID_ARGUMENT;
    }

    if (bsp_pin(E_BSP_PIN_GET, p_slpTrig)) {
        return E_RADIO_WRONG_STATE;
    }

    /* Wait for radio to finish previous operation */
    _rf230_waitIdle();
 //   for(;;)
 //   {
        c_orig_state = _rf230_getState();
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
        _rf230_smReset(); /* Go to TRX_OFF from any state. */
    } else {
        /* It is not allowed to go from RX_AACK_ON or TX_AACK_ON and directly to */
        /* TX_AACK_ON or RX_AACK_ON respectively. Need to go via RX_ON or PLL_ON. */
        if ((c_new_state == TX_ARET_ON) &&
            (c_orig_state == RX_AACK_ON)){
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            /* The final state transition to TX_ARET_ON is handled after the if-else if. */
            _spiBitWrite(p_spi, RG_TRX_STATE, SR_TRX_CMD, PLL_ON);
//            bsp_spiSubWrite(SR_TRX_CMD, PLL_ON);
            bsp_delay_us(E_TIME_STATE_TRANSITION_PLL_ACTIVE);
        } else if ((c_new_state == RX_AACK_ON) &&
                 (c_orig_state == TX_ARET_ON)){
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            /* The final state transition to RX_AACK_ON is handled after the if-else if. */
            _spiBitWrite(p_spi, RG_TRX_STATE, SR_TRX_CMD, RX_ON);
//            bsp_spiSubWrite(SR_TRX_CMD, RX_ON);
            bsp_delay_us(E_TIME_STATE_TRANSITION_PLL_ACTIVE);
        }

        /* Any other state transition can be done directly. */
        _spiBitWrite(p_spi, RG_TRX_STATE, SR_TRX_CMD, c_new_state);
//        bsp_spiSubWrite(SR_TRX_CMD, c_new_state);

        /* When the PLL is active most states can be reached in 1us. However, from */
        /* TRX_OFF the PLL needs time to activate. */
        if (c_orig_state == TRX_OFF){
            bsp_delay_us(E_TIME_TRX_OFF_TO_PLL_ACTIVE);
        } else {
            bsp_delay_us(E_TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
    } /*  end: if(new_state == TRX_OFF) ... */

    /*Verify state transition.*/
    e_status = E_RADIO_TIMED_OUT;

    if (_rf230_getState() == c_new_state){
        e_status = E_RADIO_SUCCESS;
    }
    return e_status;
} /*  _rf230_setTrxState() */

/*----------------------------------------------------------------------------*/
/** \brief  This function will copy data prepared for transmission
 *             into the frame buffer of at86rf230
 *
 *  \param     c_len        How much data should be transmitted.
 *
 *  \retval    RADIO_TX_OK                 Transmission was successfully done.
 *  \retval    RADIO_TX_COLLISION          Frame collision has happened (channek busy).
 *  \retval    RADIO_TX_NOACK           No acknowledgment received.
 *  \retval    RADIO_TX_ERR                Error while transmitting.
 */
static e_radio_tx_status_t _rf230_transmit(uint8_t c_len)
{
    int8_t                    c_txpower;
    uint8_t                 c_total_len;
    uint8_t                 c_tx_result;
    /* If radio is sleeping we have to turn it on first */
    /* This automatically does the PLL calibrations */
    if (bsp_pin(E_BSP_PIN_GET, p_slpTrig)) {
        bsp_clrPin(p_slpTrig);
        bsp_delay_us(2*E_TIME_SLEEP_TO_TRX_OFF); //extra delay depends on board capacitance
        //    bsp_delay_us(E_TIME_SLEEP_TO_TRX_OFF+E_TIME_SLEEP_TO_TRX_OFF/2);
    } else {
#if RADIO_CONF_CALIBRATE_INTERVAL
        /* If nonzero, do periodic calibration. See clock.c */
        if (_rf230_calibrate) {
            _spiBitWrite(p_spi,  RG_PLL_CF, SR_PLL_CF_START, 1); //takes 80us max
//            bsp_spiSubWrite(SR_PLL_CF_START,1);   //takes 80us max
            _spiBitWrite(p_spi,  RG_PLL_DCU, SR_PLL_DCU_START, 1); //takes 6us, concurrently
//            bsp_spiSubWrite(SR_PLL_DCU_START,1); //takes 6us, concurrently
            _rf230_calibrate=0;
            _rf230_calibrated=1;
            bsp_delay_us(80); //?
        }
#endif
    }

    /* Wait for any previous operation or state transition to finish */
    _rf230_waitIdle();
    /* Prepare to transmit */
#if RF230_CONF_AUTORETRIES
    if ((c_tx_result = _rf230_setTrxState(TX_ARET_ON)) != E_RADIO_SUCCESS) {
        LOG_ERR("Change transceiver SM state - failed (code %02X)\n\r",c_tx_result);
        return RADIO_TX_ERR;
    }
#else
    _rf230_setTrxState(PLL_ON);
#endif

    c_txpower = 0;

    if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        /* Remember the current transmission power */
        c_txpower = _rf230_getTxPower();
        /* Set the specified transmission power */
        _rf230_setTxPower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
    }

    c_total_len = c_len + RF230_CHECKSUM_LEN;

    _rf230_fWrite(pc_buffer, c_total_len);
    /* Toggle the SLP_TR pin to initiate the frame transmission */
    bsp_pin(E_BSP_PIN_SET, p_slpTrig);
    bsp_pin(E_BSP_PIN_CLR, p_slpTrig);
    LOG_DBG("_rf230_transmit: %d\n\r", (int)c_total_len);

    /* We wait until transmission has ended so that we get an
     accurate measurement of the transmission time.*/
    _rf230_waitIdle();

/* Get the transmission result */
#if RF230_CONF_AUTORETRIES
    c_tx_result = bsp_spiBitRead(p_spi, RF230_READ_COMMAND | RG_TRX_STATE, SR_TRAC_STATUS);
//    c_tx_result = bsp_spiSubRead(SR_TRAC_STATUS);
#else
    c_tx_result = RF230_TX_SUCCESS;
#endif

    /* Restore the transmission power */
    if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        _rf230_setTxPower(c_txpower & 0xff);
    }

    if(c_receive_on) {
        _rf230_intON();
    } else {
    #if RADIOALWAYSON
    /* Enable reception */
        _rf230_intON();
    #else
        _rf230_intOFF();
        LOG_DBG("%s\n\r","_rf230_transmit: turning radio off");
    #endif
    }

#if RF230_INSERTACK
   ack_pending = 0;
#endif
    switch (c_tx_result) {
        case RF230_TX_SUCCESS:
        case RF230_TX_SUC_DPEND:
#if RF230_INSERTACK
            /* Not PAN broadcast to FFFF, and ACK was requested and received */
            if (!((pc_buffer[5]==0xff) && (pc_buffer[6]==0xff)) && (pc_buffer[0]&(1<<6)))
            ack_pending=1;
#endif
            return RADIO_TX_OK;
            return RADIO_TX_OK;
        case RF230_TX_CH_ACCFAIL:
            LOG_ERR("%s","_rf230_transmit: CSMA channel access fail");
            return RADIO_TX_COLLISION;
        case RF230_TX_NO_ACK:
            LOG_ERR("%s","_rf230_transmit: ACK not received\n\r");
            return RADIO_TX_NOACK;
        case RF230_TX_INVALID:
        default:
            LOG_ERR("%s","_rf230_transmit: Invalid\n\r");
            return RADIO_TX_ERR;
    }
} /*  _rf230_transmit() */



/*----------------------------------------------------------------------------*/
/** \brief  This function is called from ISR and it reads packet form
 *             the frame buffer
 *
 *  \param     none
 *
 *  \retval    none
 */
/*---------------------------------------------------------------------------*/
static void _rf230_callback(c_event_t c_event, p_data_t p_data)
{
    int8_t c_len;

    c_rf230_pending = 0;

    packetbuf_clear();

    /* Turn off interrupts to avoid ISR writing to the same buffers we are reading. */
    bsp_enterCritical();

    c_len = _rf230_read(packetbuf_dataptr(), PACKETBUF_SIZE);

    /* Restore interrupts. */
    bsp_exitCritical();
    LOG_DBG("%u bytes lqi %u\n\r",c_len,c_last_correlation);

    if((c_len > 0) && (p_lmac != NULL)) {
          packetbuf_set_datalen(c_len);
          p_lmac->input();
    }
    //bsp_led(E_BSP_LED_GREEN,E_BSP_LED_TOGGLE);
} /*  _rf230_callback() */

/*----------------------------------------------------------------------------*/
/** \brief  This function prepares data for transmitting
 *
 *  \param         p_payload     Pointer to data to be transmitted
 *  \param        c_len        Data length
 *
 *  \retval        none
 */
/*---------------------------------------------------------------------------*/
static int8_t _rf230_prepare(const void * p_payload, uint8_t c_len)
{
    uint8_t     c_flen;
    uint8_t        *pc_buf;
#if RF230_CONF_CHECKSUM
  uint16_t checksum;
#endif
#if RF230_INSERTACK
/* The sequence number is needed to construct the ack packet */
  ack_seqnum=*(((uint8_t *)payload)+2);
#endif

#if RF230_CONF_CHECKSUM
  checksum = crc16_data(payload, payload_len, 0);
#endif

  /* Copy payload to RAM buffer */
  c_flen = c_len + RF230_CHECKSUM_LEN;
  if (c_flen > RF230_MAX_TX_FRAME_LENGTH){
    LOG_DBG("packet too large (%d, max: %d)\n\r",c_flen,RF230_MAX_TX_FRAME_LENGTH);
    return 1;
  }
  pc_buf=&pc_buffer[0];
  memcpy(pc_buf,p_payload,c_len);
  pc_buf+=c_len;

#if RF230_CONF_CHECKSUM
  memcpy(pbuf,&checksum,CHECKSUM_LEN);
  pbuf+=CHECKSUM_LEN;
#endif
/*------------------------------------------------------------*/

#ifdef RF230BB_HOOK_TX_PACKET
#if !RF230_CONF_CHECKSUM
  { // Add a checksum before we log the packet out
    uint16_t checksum;
    checksum = crc16_data(payload, payload_len, 0);
    memcpy(buffer+total_len-CHECKSUM_LEN,&checksum,CHECKSUM_LEN);
  }
#endif /* RF230_CONF_CHECKSUM */
  RF230BB_HOOK_TX_PACKET(buffer,total_len);
#endif

  return 0;
} /*  _rf230_prepare() */


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
static int8_t _rf230_read(void *p_buf, uint8_t c_bufsize)
{
    uint8_t     c_len;
    uint8_t        *pc_framep;
#if FOOTER_LEN
  uint8_t footer[FOOTER_LEN];
#endif
#if RF230_CONF_CHECKSUM
  uint16_t checksum;
#endif
#if RF230_INSERTACK
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
#if RADIOALWAYSON && DEBUGFLOWSIZE
   if (c_receive_on==0) {if (debugflow[debugflowsize-1]!='z') DEBUGFLOW('z');} //cxmac calls with radio off?
#endif
    return 0;
  }

#if RADIOALWAYSON
    if (c_receive_on) {
#else
        if (bsp_pin(E_BSP_PIN_GET, p_slpTrig)) {
            return 0;
        }
        if (!c_receive_on) {
            return 0;
        }
#endif

    if(c_len > RF230_MAX_TX_FRAME_LENGTH) {
        /* Oops, we must be out of sync. */
        gps_rxframe[c_rxframe_head].length=0;
        return 0;
    }

    if(c_len <= RF230_CHECKSUM_LEN) {
        gps_rxframe[c_rxframe_head].length=0;
        return 0;
    }

    if(c_len - RF230_CHECKSUM_LEN > c_bufsize) {
        gps_rxframe[c_rxframe_head].length=0;
        return 0;
    }
    /* Transfer the frame, stripping the footer, but copying the checksum */
    pc_framep=&(gps_rxframe[c_rxframe_head].data[0]);
    memcpy(p_buf,pc_framep,c_len - RF230_CHECKSUM_LEN + RF230_CHECKSUM_LEN);
    c_last_correlation = gps_rxframe[c_rxframe_head].lqi;

    /* Clear the length field to allow buffering of the next packet */
    gps_rxframe[c_rxframe_head].length=0;
    c_rxframe_head++;
    if (c_rxframe_head >= RF230_CONF_RX_BUFFERS)
        c_rxframe_head=0;
    /* If another packet has been buffered, schedule another receive poll */
    if (gps_rxframe[c_rxframe_head].length && c_receive_on)
        evproc_putEvent(E_EVPROC_HEAD,EVENT_TYPE_PCK_LL,NULL);
//        _rf230_callback();

    /* Point to the checksum */
    pc_framep += c_len - RF230_CHECKSUM_LEN;
#if RF230_CONF_CHECKSUM
    memcpy(&checksum,pc_framep,RF230_CHECKSUM_LEN);
#endif /* RF230_CONF_CHECKSUM */
    pc_framep += RF230_CHECKSUM_LEN;
    pc_framep += RF230_TIMESTAMP_LEN;
#if FOOTER_LEN
    memcpy(footer,framep,FOOTER_LEN);
#endif
#if RF230_CONF_CHECKSUM
    if(checksum != crc16_data(p_buf, c_len - RF230_CHECKSUM_LEN, 0)) {
        //PRINTF("checksum failed 0x%04x != 0x%04x\n\r",
        //  checksum, crc16_data(buf, len - AUX_LEN, 0));
    }
#if FOOTER_LEN
    if(footer[1] & FOOTER1_CRC_OK &&
            checksum == crc16_data(p_buf, c_len - RF230_CHECKSUM_LEN, 0)) {
#endif
#endif /* RF230_CONF_CHECKSUM */

/* Get the received signal strength for the packet, 0-84 dB above rx threshold */
#if RF230_CONF_AUTOACK
 //   _rf230_last_rssi = bsp_spiSubRead(SR_ED_LEVEL);  //0-84 resolution 1 dB
        c_last_rssi = bsp_spiRegRead(p_spi, RF230_READ_COMMAND | RG_PHY_ED_LEVEL); //0-84, resolution 1 dB
//    c_last_rssi =bsp_spiRead(RG_PHY_ED_LEVEL);  //0-84, resolution 1 dB
#else
/* last_rssi will have been set at RX_START interrupt */
//  _rf230_last_rssi = 3*bsp_spiSubRead(SR_RSSI);    //0-28 resolution 3 dB
#endif

    /* Save the smallest rssi. The display routine can reset by setting it to zero */
    if ((c_smallest_rssi == 0) || (c_last_rssi < c_smallest_rssi))
        c_smallest_rssi = c_last_rssi;

 //   _rf230_last_correlation = rxframe[rxframe_head].lqi;
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, c_last_rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, c_last_correlation);

#if RF230_CONF_CHECKSUM
#if FOOTER_LEN
    } else {
        len = AUX_LEN;
    }
#endif
#endif

#ifdef RF230BB_HOOK_RX_PACKET
  RF230BB_HOOK_RX_PACKET(buf,len);
#endif

  /* Here return just the data length. The checksum is however still in the buffer for packet sniffing */
  return c_len - RF230_CHECKSUM_LEN;

#if RADIOALWAYSON
} else { //Stack thought radio was off
   return 0;
}
#endif
} /*  _rf230_read() */

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
static void _rf230_setPanAddr(unsigned pan,
                                    unsigned addr,
                                    const uint8_t prc_ieee_addr[8])
{
  LOG_INFO("PAN=%x Short Addr=%x\n\r",pan,addr);
  uint8_t c_abyte;
  c_abyte = pan & 0xFF;
  bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND |RG_PAN_ID_0, c_abyte);
//  bsp_spiWrite(RG_PAN_ID_0,c_abyte);
  c_abyte = (pan >> 8*1) & 0xFF;
  bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND |RG_PAN_ID_1, c_abyte);
//  bsp_spiWrite(RG_PAN_ID_1, c_abyte);

  c_abyte = addr & 0xFF;
  bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND |RG_SHORT_ADDR_0, c_abyte);
//  bsp_spiWrite(RG_SHORT_ADDR_0, c_abyte);
  c_abyte = (addr >> 8*1) & 0xFF;
  bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND |RG_SHORT_ADDR_1, c_abyte);
//  bsp_spiWrite(RG_SHORT_ADDR_1, c_abyte);

  if (prc_ieee_addr != NULL) {
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_7, *prc_ieee_addr++);
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_6, *prc_ieee_addr++);
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_5, *prc_ieee_addr++);
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_4, *prc_ieee_addr++);
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_3, *prc_ieee_addr++);
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_2, *prc_ieee_addr++);
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_1, *prc_ieee_addr++);
      bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IEEE_ADDR_0, *prc_ieee_addr++);
  }
} /* _rf230_setPanAddr() */

/*---------------------------------------------------------------------------*/
uint8_t _rf230_getTxPower(void)
{
    uint8_t c_power = TX_PWR_UNDEFINED;
    if (bsp_pin(E_BSP_PIN_GET, p_slpTrig)) {
        LOG_DBG("%s\n\r","_rf230_getTxPower:Sleeping");
    } else {
        c_power = bsp_spiBitRead(p_spi, RF230_READ_COMMAND | RG_PHY_TX_PWR, SR_TX_PWR);
//        c_power = bsp_spiSubRead(SR_TX_PWR);
    }
    return c_power;
} /* _rf230_getTxPower() */


/*---------------------------------------------------------------------------*/
void _rf230_setTxPower(uint8_t c_power)
{
    if (c_power > TX_PWR_MAX){
        c_power=TX_PWR_MAX;
    }
    if (bsp_pin(E_BSP_PIN_GET, p_slpTrig)) {
        LOG_DBG("%s\n\r","_rf230_setTxPower:Sleeping");  //happens with cxmac
    } else {
        _spiBitWrite(p_spi, RG_PHY_TX_PWR, SR_TX_PWR, c_power);
//        bsp_spiSubWrite(SR_TX_PWR, c_power);
    }
} /* _rf230_setTxPower() */

/*==============================================================================
  radio_wreset()
 =============================================================================*/
void _rf230_wReset(void)
{
    uint8_t c_tempReg;
    bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IRQ_MASK, RF230_SUPPORTED_INTERRUPT_MASK);

  /* Set up number of automatic retries 0-15 (0 implies PLL_ON sends instead of the extended TX_ARET mode */
    _spiBitWrite(p_spi, RG_XAH_CTRL_0, SR_MAX_FRAME_RETRIES, RF230_CONF_AUTORETRIES);

 /* Set up carrier sense/clear c_channel assesment parameters for extended operating mode */
    _spiBitWrite(p_spi, RG_XAH_CTRL_0, SR_MAX_CSMA_RETRIES, 5);  //highest allowed retries
    bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_CSMA_BE, 0x80); //min backoff exponent 0, max 8 (highest allowed)
    c_tempReg = bsp_spiRegRead(p_spi, RF230_READ_COMMAND | RG_PHY_RSSI);
    bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_CSMA_SEED_0, c_tempReg); //upper two RSSI reg bits RND_VALUE are random

  /* CCA Mode Mode 1=Energy above threshold  2=Carrier sense only  3=Both 0=Either (RF231 only) */
    //bsp_spiSubWrite(SR_E_CCA_MODE,1);  //1 is the power-on default

  /* Receiver sensitivity. If nonzero rf231/128rfa1 saves 0.5ma in rx mode */
  /* Not implemented on rf230 but does not hurt to write to it */
#ifdef RF230_MIN_RX_POWER
#if RF230_MIN_RX_POWER > 84
#warning rf231 power threshold clipped to -48dBm by hardware register
    bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_RX_SYN, 0xf);
//    bsp_spiWrite(RG_RX_SYN, 0xf);
#elif RF230_MIN_RX_POWER < 0
#error RF230_MIN_RX_POWER can not be negative!
#endif
    bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_RX_SYN, RF230_MIN_RX_POWER/6 + 1); //1-15 -> -90 to -48dBm
//  bsp_spiWrite(RG_RX_SYN, RF230_MIN_RX_POWER/6 + 1); //1-15 -> -90 to -48dBm
#endif

  /* CCA energy threshold = -91dB + 2*SR_E_CCA_ED_THRESH. Reset defaults to -77dB */
  /* Use RF230 base of -91;  RF231 base is -90 according to datasheet */
#ifdef RF230_CONF_E_CCA_THRES
#if RF230_CONF_E_CCA_THRES < -91
#warning RF230_CONF_E_CCA_THRES below hardware limit, setting to -91dBm
    _spiBitWrite(p_spi, RG_E_CCA_THRES, SR_E_CCA_ED_THRES, 0);
//  bsp_spiSubWrite(SR_E_CCA_ED_THRES,0);
#elif RF230_CONF_E_CCA_THRES > -61
#warning RF230_CONF_E_CCA_THRES above hardware limit, setting to -61dBm
    _spiBitWrite(p_spi, RG_E_CCA_THRES, SR_E_CCA_ED_THRES, 15);
//  bsp_spiSubWrite(SR_E_CCA_ED_THRES,15);
#else
      _spiBitWrite(p_spi, RG_E_CCA_THRES, SR_E_CCA_ED_THRES, (RF230_CONF_E_CCA_THRES+91)/2);
//  bsp_spiSubWrite(SR_E_CCA_ED_THRES,(RF230_CONF_E_CCA_THRES+91)/2);
#endif
#endif

  /* Use automatic CRC unless manual is specified */
#if RF230_CONF_CHECKSUM
//  bsp_spiSubWrite(SR_TX_AUTO_CRC_ON, 0);
      _spiBitWrite(p_spi, RG_PHY_TX_PWR, SR_TX_AUTO_CRC_ON, 0);
#else
      _spiBitWrite(p_spi, RG_PHY_TX_PWR, SR_TX_AUTO_CRC_ON, 1);
#endif

      /* Limit tx power for testing miniature Raven mesh */
#if USE_MAX_RADIO_POWER == TRUE
            _rf230_setTxPower(TX_PWR_MAX );  //0=3dbm 15=-17.2dbm
#else
            _rf230_setTxPower(TX_PWR_MIN);  //0=3dbm 15=-17.2dbm
#endif
} /* _rf230_wReset() */

static void _rf230_promisc(uint8_t value)
{
    uint8_t ac_addr[8];
    if (value) {
        memset(&ac_addr, 0, 8);
        _rf212b_setPanAddr(0x0000, 0, ac_addr);
        _spiBitWrite(p_spi, SR_AACK_PROM_MODE, 1);
        _spiBitWrite(p_spi, SR_AACK_DIS_ACK, 1);
    }
    else {
        _spiBitWrite(p_spi, SR_AACK_PROM_MODE, 0);
        _spiBitWrite(p_spi, SR_AACK_DIS_ACK, 0);
    }
}

/*---------------------------------------------------------------------------*/
static int8_t _rf230_send(const void *pr_payload, uint8_t c_len)
{
    int8_t c_ret = 0;
#ifdef RF230BB_HOOK_IS_SEND_ENABLED
    if(!RF230BB_HOOK_IS_SEND_ENABLED()) {
        LOG_ERROR("%s","failed\n\r");
        return c_ret;
    }
#endif
    if((c_ret = _rf230_prepare(pr_payload, c_len))) {
        LOG_ERR("_rf230_send: Unable to send, prep failed (%d)\n\r",c_ret);
        return c_ret;
    }
    c_ret = _rf230_transmit(c_len);
    if (c_ret != RADIO_TX_OK) {
        //bsp_led(E_BSP_LED_RED,E_BSP_LED_TOGGLE);
        LOG_ERR("Send failed with code %d \n\r",c_ret);
    }
    else {
        //bsp_led(E_BSP_LED_YELLOW,E_BSP_LED_TOGGLE);
    }

    return c_ret;
} /* _rf230_send() */


/*----------------------------------------------------------------------------*/
/** \brief  This function turns on the radio transceiver
 *
 *    \param    none
 *
 *    \retval    none
 */
/*----------------------------------------------------------------------------*/
static int8_t _rf230_intON(void)
{
    c_receive_on = 1;
/* If radio is off (slptr high), turn it on */
    if (bsp_pin(E_BSP_PIN_GET, p_slpTrig)) {
    #if RF230BB_CONF_LEDONPORTE1
        PORTE|=(1<<PE1); //ledon
    #endif
    /* SPI based radios. The wake time depends on board capacitance.
     * Make sure the delay is long enough, as using SPI too soon will reset the MCU!
     * Use 2x the nominal value for safety. 1.5x is not long enough for Raven!
     */
    //  uint8_t sreg = SREG;cli();
        bsp_clrPin(p_slpTrig);
        bsp_delay_us(2*E_TIME_SLEEP_TO_TRX_OFF);
    //  bsp_delay_us(E_TIME_SLEEP_TO_TRX_OFF+E_TIME_SLEEP_TO_TRX_OFF/2);
    //  SREG=sreg;
    }

#if RF230_CONF_AUTOACK
 // _rf230_setTrxState(is_promiscuous?RX_ON:RX_AACK_ON);
    if (_rf230_setTrxState(RX_AACK_ON) != E_RADIO_SUCCESS)
        printf("Aack set failed\n\r");
#else
    _rf230_setTrxState(RX_ON);
#endif
    _rf230_waitIdle();
    return 1;
} /* _rf230_intON() */

/*---------------------------------------------------------------------------*/
static int8_t _rf230_intOFF(void)
{
    c_receive_on = 0;
#if RF230BB_CONF_LEDONPORTE1
    PORTE&=~(1<<PE1); //ledoff
#endif
#ifdef RF230BB_HOOK_RADIO_OFF
    RF230BB_HOOK_RADIO_OFF();
#endif

    /* Wait for any transmission to end */
    _rf230_waitIdle();

#if RADIOALWAYSON
/* Do not transmit autoacks when stack thinks radio is off */
    _rf230_setTrxState(RX_ON);
#else
    /* Force the device into TRX_OFF. */
    _rf230_smReset();
#if RADIOSLEEPSWHENOFF
    /* Sleep Radio */
    bsp_setPin(SLP_TR_PORT,SLP_TR_PIN);
    ENERGEST_OFF(ENERGEST_TYPE_LED_RED);
#endif
#endif /* RADIOALWAYSON */

    return 0;
} /* _rf230_off() */

static int8_t _rf230_extON(void)
{
    if (c_receive_on)
        return 1;
    _rf230_intON();
    return 1;
} /* _rf230_extON() */

static int8_t _rf230_extOFF(void)
{
    if (c_receive_on == 0)
        return 0;

    /*
     * If we are currently receiving a packet, we still call off(),
     * as that routine waits until Rx is complete (packet uploaded in ISR
     * so no worries about losing it). If using RX_AACK_MODE, chances are
     * the packet is not for us and will be discarded.
     */
    if (!_rf230_isIdle()) {
        LOG_INFO("%s\n\r","_rf230_extOFF: busy receiving.");
    }
    _rf230_intOFF();
    return 0;
} /* _rf230_extON() */

/*==============================================================================
  _rf230_init()
 =============================================================================*/
static int8_t _rf230_init(s_ns_t* ns)
{
    uint8_t     i;
    uint8_t     c_tvers;
    uint8_t     c_tmanu;
    uint8_t        c_ret = 0;
    linkaddr_t     un_addr;
    /* Wait in case VCC just applied */
    bsp_delay_us(E_TIME_TO_ENTER_P_ON);
    /* Init spi interface and transceiver. Transceiver utilize spi interface. */
    p_rst = bsp_pinInit( E_TARGET_RADIO_RST);
    p_slpTrig = bsp_pinInit( E_TARGET_RADIO_SLPTR);
    p_spi = bsp_spiInit();

    if ((p_spi == NULL) || ((p_rst == NULL)) || (p_slpTrig == NULL) ||
        (ns == NULL)) {
        return 0;
    }
    bsp_extIntInit(E_TARGET_RADIO_INT, _isr_callback);

    /* Set receive buffers empty and point to the first */
    for (i=0;i<RF230_CONF_RX_BUFFERS;i++)
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
    bsp_delay_us(2*E_TIME_SLEEP_TO_TRX_OFF);
    bsp_pin(E_BSP_PIN_SET, p_rst);

    /* Force transition to TRX_OFF */
    _spiBitWrite(p_spi, RG_TRX_STATE, SR_TRX_CMD, CMD_FORCE_TRX_OFF);
    bsp_delay_us(E_TIME_P_ON_TO_TRX_OFF);

    /* Verify that it is a supported version */
    /* Note gcc optimizes this away if DEBUG is not set! */
    //ATMEGA128RFA1 - version 4, ID 31
    c_tvers = bsp_spiRegRead(p_spi, RF230_READ_COMMAND | RG_VERSION_NUM);
    c_tmanu = bsp_spiRegRead(p_spi, RF230_READ_COMMAND | RG_MAN_ID_0);

    if ((c_tvers != RF230_REVA) && (c_tvers != RF230_REVB))
        LOG_INFO("Unsupported version %u\n\r",c_tvers);
    if (c_tmanu != SUPPORTED_MANUFACTURER_ID)
        LOG_INFO("Unsupported manufacturer ID %u\n\r",c_tmanu);

    LOG_INFO("Version %u, ID %u\n\r",c_tvers,c_tmanu);

    _rf230_wReset();
      /* Leave radio in on state (?)*/
    _rf230_intON();
    if (mac_phy_config.mac_address == NULL) {
                c_ret = 0;
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

        evproc_regCallback(EVENT_TYPE_PCK_LL,_rf230_callback);
        if (ns->lmac != NULL) {
            p_lmac = ns->lmac;
            c_ret = 1;
        } else {
            c_ret = 0;
        }
    }

    return 1;
} /* _rf230_init() */

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
    uint8_t i;
    /* Using SPI bus from ISR is generally a bad idea... */
    c_int_src = bsp_spiRegRead(p_spi, RF230_READ_COMMAND | RG_IRQ_STATUS);
    /* Note: all IRQ are not always automatically disabled when running in ISR */
    /*Handle the incomming interrupt. Prioritized.*/
    printf("Int source = %d\n\r",c_int_src);
    if ((c_int_src & RX_START_MASK)){
#if !RF230_CONF_AUTOACK
        bsp_spiTxRx(p_spi, RF230_READ_COMMAND | SR_RSSI,  &c_last_rssi);
        c_last_rssi *= 3;
//        c_last_rssi = 3 * bsp_spiSubRead(SR_RSSI);
#endif
    } else if (c_int_src & TRX_END_MASK){
        c_state = bsp_spiBitRead(p_spi, RF230_READ_COMMAND | RG_TRX_STATUS, SR_TRX_STATUS);
//        c_state = bsp_spiSubRead(SR_TRX_STATUS);
        if( (c_state == BUSY_RX_AACK) || \
            (c_state == RX_ON) ||          \
            (c_state == BUSY_RX) ||      \
            (c_state == RX_AACK_ON))
        {
            /* Received packet interrupt */
            /* Buffer the frame and call _rf230_int to schedule poll for rf230 receive process */
            /* Save the rssi for printing in the main loop */
#ifdef RF230_MIN_RX_POWER
#if RF230_CONF_AUTOACK
            c_last_rssi = bsp_spiRegRead(p_spi, RF230_READ_COMMAND | RG_PHY_ED_LEVEL);
//            c_last_rssi=bsp_spiRead(RG_PHY_ED_LEVEL);
#endif
            if (c_last_rssi >= RF230_MIN_RX_POWER) {
#endif
                _rf230_fRead(&gps_rxframe[c_rxframe_tail]);
                for (i=0;i<RF230_CONF_RX_BUFFERS;i++) {
                    if ((i != c_rxframe_tail) && (memcmp(gps_rxframe[c_rxframe_tail].data,gps_rxframe[i].data, MAX_FRAME_LENGTH) == 0)) {
                        gps_rxframe[c_rxframe_tail].length = 0;
                        LOG_INFO("%s\n\r","Duplicate packet detected");
                    }
                }
                if (gps_rxframe[c_rxframe_tail].length == 0)
                    return;
                else {
                    c_rxframe_tail++;
                    if (c_rxframe_tail >= RF230_CONF_RX_BUFFERS)
                        c_rxframe_tail=0;
                    if (c_receive_on)
                        evproc_putEvent(E_EVPROC_HEAD,EVENT_TYPE_PCK_LL,NULL);
                }
#ifdef RF230_MIN_RX_POWER
            }
#endif
        }
    } else if (c_int_src & TRX_UR_MASK){
    } else if (c_int_src & PLL_UNLOCK_MASK){
    } else if (c_int_src & PLL_LOCK_MASK){
    } else if (c_int_src & BAT_LOW_MASK){
        /*  Disable BAT_LOW interrupt to prevent endless interrupts. The interrupt */
        /*  will continously be asserted while the supply voltage is less than the */
        /*  user-defined voltage threshold. */
        c_isr_mask = bsp_spiRegRead(p_spi, RF230_READ_COMMAND | RG_IRQ_MASK);
//        uint8_t c_isr_mask = bsp_spiRead(RG_IRQ_MASK);
        c_isr_mask &= ~BAT_LOW_MASK;
        bsp_spiRegWrite(p_spi, RF230_WRITE_COMMAND | RG_IRQ_MASK, c_isr_mask);
//        bsp_spiWrite(RG_IRQ_MASK, c_isr_mask);
     }
}

/** @} */
