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
 * \addtogroup bsp
 * @{
 * \addtogroup if    PHY interfaces
 * @{
 *
 * \defgroup at86rf212 Radio transceiver (AT86RF212) library
 *
 * The at86rf212 library provides function for proper initializing, sending
 * and receiving packets.
 *
 * @{
 */
/*! \file   at86rf212.h

    \author Artem Yushev artem.yushev@hs-offenbrug.de

    \brief  Hardware dependent initialization header file for AT86RF212.

    \details For adaptation to another MAC/PHY layer at least 4 functions have
            to be implemented in accordance to \ref netstack_interface definition. Also
            after copying frame into the internal buffer and returning from the
            interrupt service routine \ref netstack_lowMac.input() should be called.
            See _rf212_callback() and ISR(INT5_vect) functions.

    \version 0.0.1
*/
/*============================================================================*/

#ifndef AT86RF212_H_
#define AT86RF212_H_

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#include "emb6.h"

/*==============================================================================
                                     MACROS
==============================================================================*/

/*! Defined in the ATMEL documentation.
 Result code for a successful transmitting transaction */
#define RF212_TX_SUCCESS                        0
#define RF212_TX_SUC_DPEND                        1        ///< Also successful transaction
#define RF212_TX_CH_ACCFAIL                        3        ///< Channel access failed
#define RF212_TX_NO_ACK                            5        ///< No acknowledgment received
#define RF212_TX_INVALID                        7        ///< Invalid transaction

#define BAT_LOW_MASK                               ( 0x80 ) ///< Mask for the BAT_LOW interrupt.
#define TRX_UR_MASK                                ( 0x40 ) ///< Mask for the TRX_UR interrupt.
#define TRX_END_MASK                               ( 0x08 ) ///< Mask for the TRX_END interrupt.
#define RX_START_MASK                              ( 0x04 ) ///< Mask for the RX_START interrupt.
#define PLL_UNLOCK_MASK                            ( 0x02 ) ///< Mask for the PLL_UNLOCK interrupt.
#define PLL_LOCK_MASK                              ( 0x01 ) ///< Mask for the PLL_LOCK interrupt.

#define MIN_FRAME_LENGTH                           ( 0x03 ) ///< A frame should be at least 3 bytes.
#define MAX_FRAME_LENGTH                           ( 0x7F ) ///< A frame should no more than 127 bytes.

#define RF212_CONF_AUTOACK                       TRUE    //!< Define autoacknoledgment
#define RF212_CONF_AUTORETRIES                    2        //!< Amount of autoretries after failed transmitting
#define RF212_CONF_RX_BUFFERS                     3        //!< Define using of rx buffers
//#define    RF212_MIN_RX_POWER                        0         // RX sensitivity reduced down to -48 dBm
#define RF212_CONF_CHECKSUM                     0        //!< RF212_CONF_CHECKSUM=0 for automatic hardware checksum
#define RF212_CHECKSUM_LEN                         2        //!< Length of a checksum in a frame
#define RF212_TIMESTAMP_LEN                        0        //!< Timestamp length if it was defined

//!< Driver is working with devices which has only this part number
#define SUPPORTED_PART_NUMBER                  ( 2 )
//!< Driver is working with devices which has only this manufacturer id
#define SUPPORTED_MANUFACTURER_ID              ( 31 )
#define RF212_REVA                             ( 1 )    //!< Revision of a transceiver
#define RF212_REVB                             ( 2 )    //!< Revision of a transceiver
#define RF212B_REVA                            ( 3 )

/*! RF212 does not support RX_START interrupts in extended mode, but it seems harmless to always enable it.
 In non-extended mode this allows RX_START to sample the RF rssi at the end of the preamble */
#define RF212_SUPPORTED_INTERRUPT_MASK        ( 0x08 )  //enable trx end only
//#define RF212_SUPPORTED_INTERRUPT_MASK        ( 0x0F ) //disable bat low, trx underrun
//#define RF212_SUPPORTED_INTERRUPT_MASK         ( 0x0C )  //!< disable bat low, trx underrun, pll lock/unlock
//#define RF212_SUPPORTED_INTERRUPT_MASK         ( 0x2C )

//#define RF212_MIN_CHANNEL                      ( 11 )
//#define RF212_MAX_CHANNEL                      ( 26 )
#define RF212_MIN_ED_THRESHOLD                 ( 0 )    //!< Minimum energy threshold level
#define RF212_MAX_ED_THRESHOLD                 ( 15 )    //!< Maximum energy threshold level
#define RF212_MAX_TX_FRAME_LENGTH              ( 127 ) /**< 127 Byte PSDU. */
//#define RF212_MAX_PACKET_LEN                   127



#define    OQPSK_100_SIN_RC_100                    ( 0x08 )   //IEEE 802.15.4-2003/2006:  channel page 0, channel 0
#define    BPSK_20                                    ( 0x00 )   //IEEE 802.15.4-2006:        channel page 2, channel 0
#define CHANNEL_802_15_4                          ( 0x00 )   //Define default working channel

#define TXPWR_DBM                                 0
#define TXPWR_LIST_LEN                             17


#if (MODULATION == MODULATION_QPSK100)
#define TXPWR_BAND                                     1
//!< Define maximum possible transmitting power in dBm
#define TX_PWR_MAX                                     -1
//!< Define minimum possible transmitting power in dBm
#define TX_PWR_MIN                                   -11
#elif (MODULATION == MODULATION_BPSK20)
#define TXPWR_BAND                                     1
//!< Define maximum possible transmitting power in dBm
#define TX_PWR_MAX                                     4
//!< Define minimum possible transmitting power in dBm
#define TX_PWR_MIN                                   -11
#endif /* MODULATION == MODULATION_BPSK20*/



#define TX_PWR_UNDEFINED                           (TX_PWR_MIN+1)

#define WITH_SEND_CCA                             0
#define TIMESTAMP_LEN                             RF212_TIMESTAMP_LEN
#define FOOTER_LEN                                 0

#ifndef RF212_CONF_CHECKSUM
#define RF212_CONF_CHECKSUM                     0     //!< RF212_CONF_CHECKSUM=0 for automatic hardware checksum
#endif


#ifndef RF212_CONF_AUTOACK
#define RF212_CONF_AUTOACK                         1    //!< Autoack setting ignored in non-extended mode
#endif


#if RF212_CONF_AUTOACK
//static bool is_promiscuous;                    //!< We need to turn off autoack in promiscuous mode
#endif

/*! RF212_CONF_AUTORETRIES is 1 plus the number written to the hardware.
 Valid range 1-16, zero disables extended mode. */
#ifndef RF212_CONF_AUTORETRIES
#define RF212_CONF_AUTORETRIES                     3
#endif

/** In extended mode (AUTORETRIES>0) the tx routine waits for hardware
 * processing of an expected ACK and returns RADIO_TX_OK/NOACK result.
 * In non-extended mode the ACK is treated as a normal rx packet.
 * If the caller needs the ACK to be returned as an rx packet,
 * RF212_INSERTACK will generate one based on the hardware result.
 * This is triggered when the read routine is called with a buffer
 * length of three (the ack length).
 * In extended nmode it can be enabled by default to support either
 * method. In nonextended mode it would pass an extra ACK to RDCs
 * that use the TX_OK result to signal a successful ACK.
 * Adds 100 bytes of program flash and two bytes of RAM.
 */
#if RF320_CONF_INSERTACK && RF212_CONF_AUTORETRIES
#define RF212_INSERTACK                         1
uint8_t ack_pending,ack_seqnum;
#endif

/*! RF212_CONF_CSMARETRIES is number of random-backoff/CCA retries.
 The hardware will accept 0-7, but 802.15.4-2003 only allows 5 maximum */
#ifndef RF212_CONF_CSMARETRIES
#define RF212_CONF_CSMARETRIES                     5
#endif

/*! Automatic and manual CRC both append 2 bytes to packets */
#if RF212_CONF_CHECKSUM || defined(RF212BB_HOOK_TX_PACKET)
#include "crc16.h"
#endif
#define CHECKSUM_LEN                             2

/** \brief  This macro defines the start value for the RADIO_* status constants.
 *
 *          It was chosen to have this macro so that the user can define where
 *          the status returned from the TAT starts. This can be useful in a
 *          system where numerous drivers are used, and some range of status codes
 *          are occupied.
 *
 *  \see radio_status_t
 */
#define RADIO_STATUS_START_VALUE                  ( 0x40 )


#define RF212_SPI_TRANSFER(to_write)            (bsp_spiTranWrite(to_write),    \
                                                bsp_spiTranWait(), /* gcc extension, alternative inline function */        \
                                                bsp_spiTranRead() )
/*==============================================================================
                                     ENUMS
==============================================================================*/
/**
 * \enum E_RF212_E_CCA_MODE
 *
 * \brief  This enumeration defines the possible modes available for the
 *          Clear Channel Assessment algorithm.
 *
 *          These constants are extracted from the datasheet.
 *
 */
typedef enum E_RF212_E_CCA_MODE{
//    E_CCA_ED                   = 0,    /**< Use energy detection above threshold mode. */ conflicts with atmega128rfa1 mcu definition
    E_CCA_ENERGY_DETECT         = 0,    /**< Use energy detection above threshold mode. */
    E_CCA_CARRIER_SENSE         = 1,    /**< Use carrier sense mode. */
    E_CCA_CARRIER_SENSE_WITH_ED = 2     /**< Use a combination of both energy detection and carrier sense. */
}e_rf212_cca_mode_t;


/**
 * \enum    E_RF212_E_CLKM_SPEED
 *
 * \brief  This enumeration defines the possible CLKM speeds.
 *
 *          These constants are extracted from the RF212 datasheet.
 *
 */
typedef enum E_RF212_E_CLKM_SPEED{
    E_CLKM_DISABLED      = 0, /**< description */
    E_CLKM_1MHZ          = 1, /**< description */
    E_CLKM_2MHZ          = 2, /**< description */
    E_CLKM_4MHZ          = 3, /**< description */
    E_CLKM_8MHZ          = 4, /**< description */
    E_CLKM_16MHZ         = 5  /**< description */
}e_rf212_clkm_speed_t;

/**
 * \enum     E_RF212_TIMING
 *
 * \brief  RF212 hardware delay times, from datasheet
 *
 */
typedef enum E_RF212_TIMING{
    E_TIME_TO_ENTER_P_ON               = 350, /**<  Transition time from VCC is applied to P_ON - most favorable case! */
    E_TIME_P_ON_TO_TRX_OFF             = 1, /**<  Transition time from P_ON to TRX_OFF. */
    E_TIME_SLEEP_TO_TRX_OFF            = 380, /**<  Transition time from SLEEP to TRX_OFF. */
    E_TIME_RESET                       = 1,   /**<  Time to hold the RST pin low during reset */
    E_TIME_ED_MEASUREMENT              = 140, /**<  Time it takes to do a ED measurement. */
    E_TIME_CCA                         = 140, /**<  Time it takes to do a CCA. */
    E_TIME_PLL_LOCK                    = 150, /**<  Maximum time it should take for the PLL to lock. */
    E_TIME_FTN_TUNING                  = 25,  /**<  Maximum time it should take to do the filter tuning. */
    E_TIME_NOCLK_TO_WAKE               = 6,   /**<  Transition time from *_NOCLK to being awake. */
    E_TIME_CMD_FORCE_TRX_OFF           = 1,   /**<  Time it takes to execute the FORCE_TRX_OFF command. */
    E_TIME_TRX_OFF_TO_PLL_ACTIVE       = 200, /**<  Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON. */
    E_TIME_STATE_TRANSITION_PLL_ACTIVE = 1,   /**<  Transition time from PLL active state to another. */
}e_rf212_timing_t;


/**
 * \enum     E_RF212_SM_STATUS
 *
 * \brief  This enumeration defines the possible return values for the TAT API
 *          functions.
 *
 *          These values are defined so that they should not collide with the
 *          return/status codes defined in the IEEE 802.15.4 standard.
 *
 */
typedef enum E_RF212_SM_STATUS{
    E_RADIO_SUCCESS = RADIO_STATUS_START_VALUE,  /**< The requested service was performed successfully. */
    E_RADIO_UNSUPPORTED_DEVICE,         /**< The connected device is not an Atmel AT86RF212. */
    E_RADIO_INVALID_ARGUMENT,           /**< One or more of the supplied function arguments are invalid. */
    E_RADIO_TIMED_OUT,                  /**< The requested service timed out. */
    E_RADIO_WRONG_STATE,                /**< The end-user tried to do an invalid state transition. */
    E_RADIO_BUSY_STATE,                 /**< The radio transceiver is busy receiving or transmitting. */
    E_RADIO_STATE_TRANSITION_FAILED,    /**< The requested state transition could not be completed. */
    E_RADIO_E_CCA_IDLE,                   /**< Channel is clear, available to transmit a new frame. */
    E_RADIO_E_CCA_BUSY,                   /**< Channel busy. */
    E_RADIO_TRX_BUSY,                   /**< Transceiver is busy receiving or transmitting data. */
    E_RADIO_BAT_LOW,                    /**< Measured battery voltage is lower than voltage threshold. */
    E_RADIO_BAT_OK,                     /**< Measured battery voltage is above the voltage threshold. */
    E_RADIO_CRC_FAILED,                 /**< The CRC failed for the actual frame. */
    E_RADIO_CHANNEL_ACCESS_FAILURE,     /**< The channel access failed during the auto mode. */
    E_RADIO_NO_ACK,                     /**< No acknowledge frame was received. */
}e_rf212_sm_status_t;
/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/

/** \struct st_rxframe_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see _rf212_fRead
 */
typedef struct {
    uint8_t length;                           /**< Length of frame. */
    uint8_t data[ MAX_FRAME_LENGTH ];         /**< Actual frame data. */
    uint8_t lqi;                              /**< LQI value for received frame. */
    bool crc;                                 /**< Flag - did CRC pass for received frame? */
}st_rxframe_t ;

#endif /* AT86RF212_H_ */
/** @} */
/** @} */
/** @} */
