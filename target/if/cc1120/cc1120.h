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


#endif /* #ifndef __RF_H__ */


