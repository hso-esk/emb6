#ifndef __RF_H__
#define __RF_H__
#ifndef __DECL_RF_H__
#define __DECL_RF_H__
#endif /* #ifndef __DECL_RF_H__ */

/*============================================================================*/
/**
 * \file    rf.h
 *
 * \author  Manuel Schappacher
 *
 * \brief   Low Level RF-Chip access.
 *
 *          This module provides low level access to the RF Chip.
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/

#include <stdint.h>

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** RF is busy */
#define RF_BUSY                         (0)
/** RF is ready */
#define RF_READY                        (1)

/** Transmit successful */
#define RF_TX_SUCCESS                   (0)
/** Error during transmission */
#define RF_TX_ERROR                     (-1)
/** Collision during transmission */
#define RF_TX_COLLISION                 (-2)
/** requested Tx power was invalid */
#define RF_TX_POWERR                    (-3)

/** minimum available transmit power in dBm */
#define RF_TXPOWER_MIN                  (-16)
/** maximum available transmit power in dBm */
#define RF_TXPOWER_MAX                  (14)

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 *  Available RF modes.
 */
typedef enum E_RF_MODE_T
{
  /* TX/RX disabled */
  E_RF_MODE_TRXOFF = 0,
  /* regular mode */
  E_RF_MODE_NORMAL,
  /* Power save mode */
  E_RF_MODE_PWRSAVE

} e_rf_mode_t;

/**
 * Available channels.
 */
typedef enum E_RF_CHANNEL_T
{
  /* testing channel */
  E_RF_CHANNEL_TEST = 0,
  /* 868@50kpbs */
  E_RF_CHANNEL_868MHZ_50KBPS,
  /* 434@50kpbs */
  E_RF_CHANNEL_434MHZ_50KBPS,
  /* 434@20kpbs */
  E_RF_CHANNEL_434MHZ_20KBPS,
  E_RF_CHANNEL_MAX

} e_rf_channel_t;

/**
 * Preamble types.
 */
typedef enum E_RF_PREAMBLE_T
{
  /* short preamble */
  E_RF_PREAMBLE_SHORT = 0,
  /* long preamble */
  E_RF_PREAMBLE_MID,
  /* long preamble */
  E_RF_PREAMBLE_LONG,

  E_RF_PREAMBLE_MAX

} e_rf_preamble_t;

/**
 * PHY CCA modes.
 */
typedef enum E_RF_CCA_MODE_T
{
  /* energy above threshold */
  E_RF_CCA_MODE_ED_THRESHOLD,
  /* carrier sense only */
  E_RF_CCA_MODE_CARRIER_SENSE,
  /* carrier sense or energy above threshols */
  E_RF_CCA_MODE_CARRIER_OR_ED,
  /* aloha (always true) */
  E_RF_CCA_MODE_ALOHA,
  /* LBT */
  E_RF_CCA_MODE_CARRIER_LBT,

  E_RF_CCA_MODE_MAX

} e_rf_cca_mode_t;

/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/

/**
 * \brief   Prototype of the RX data handler.
 *
 *          This prototype shall be used to register a RX data handler. If
 *          called, the "puc_data" parameter includes the received data, the
 *          "ui_len" parameter includes the length of the received data.
 */
typedef void (*pf_rx_t)( uint8_t* puc_data, uint16_t ui_len );

/**
 * RF handler used for RX & TX.
 */
typedef struct S_RF_HANDLER_T
{
  /** Rx handler */
  pf_rx_t pf_rx;

} s_rf_handler_t;


/*============================================================================*/
/*                    FUNCTION PROTOTYPES OF THE API                          */
/*============================================================================*/


/*============================================================================*/
/**
 * \brief   Initialize RF driver.
 *
 * \return  0 on success, -1 if an error occurred.
 */
/*============================================================================*/
int rf_init( void );


/*============================================================================*/
/**
 * \brief   Register handler to the RF driver.
 *
 * \param   ps_hndl   Handler structure to register.
 *
 * \return  0 on success, -1 if an error occurred.
 */
/*============================================================================*/
int rf_registerHandler( s_rf_handler_t* ps_hndl );


/*============================================================================*/
/**
 * \brief   Entry function of the RF module.
 *
 *          This function must be called periodically as often as possible
 *          to provide a proper behaviour of the RF module.
 *
 */
/*============================================================================*/
int rf_entry( void );

/*============================================================================*/
/**
 * \brief   Check if RF is ready.
 *
 * \return  The status of the request as follows:
 *          \refRF_BUSY if RF is busy
 *          \refRF_READY if RF is ready
 */
/*============================================================================*/
uint8_t rf_ready( void );


/*============================================================================*/
/**
 * \brief   Set RF mode.
 *
 * \param   e_mode    Mode to set.
 *
 * \return  0 on success, -1 if an error occurred.
 */
/*============================================================================*/
int rf_setMode( e_rf_mode_t e_mode );


/*============================================================================*/
/**
 * \brief   Set RF channel.
 *
 * \param   e_ch    Channel to set.
 *
 * \return  0 on success, -1 if an error occurred.
 */
/*============================================================================*/
int rf_setChannel( e_rf_channel_t e_ch );

/*============================================================================*/
/**
 * \brief   Set CCA mode.
 *
 * \param   e_cca CCA mode to set.
 *
 * \return  0 on success, -1 if an error occurred.
 */
/*============================================================================*/
int rf_setCCAMode( e_rf_cca_mode_t e_cca );


/*============================================================================*/
/**
 * \brief   Transmit a data frame.
 *
 * \param   puc_data    Data to transmit.
 * \param   ui_len      Length of the data to transmit.
 * \param   c_txPower   Tx power in dBm
 *
 * \return  The status of the request as follows:
 *          \refRF_TX_SUCCESS on success
 *          \refRF_TX_ERROR on general error
 *          \refRF_TX_COLLISION if a collision was detected.
 */
/*============================================================================*/
int rf_tx( uint8_t* puc_data, uint16_t ui_len, e_rf_preamble_t e_preamble,
    int8_t c_txPower );

/*============================================================================*/
/**
 * \brief   Perform CCA.
 *
 * \return  \refRF_BUSY if channel is busy
 *          \refRF_READY if channel is idle
 */
/*============================================================================*/
uint8_t rf_cca( void );


/*============================================================================*/
/**
 * \brief   Perform Energy Detection.
 *
 * \param   pi_ed   Buffer to write the detected value to (in dBm).
 *
 * \return  0 on success, -1 if an error occurred.
 */
/*============================================================================*/
int rf_ed( int16_t* pi_ed );


#if 1 // additional functions
uint8_t rf_is_rx (void);
uint8_t rf_is_tx (void);

/* busy = 0; free = 1*/
uint8_t rf_is_channel_busy (void);
#endif

#endif /* #ifndef __RF_H__ */


