/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_rf_6lowpan.h
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      API to the 6lowpan rf interface.
*/

#ifndef __SF_RF_6LOWPAN_H__
#define __SF_RF_6LOWPAN_H__

#ifndef __DECL_SF_RF_6LOWPAN_H__
#define __DECL_SF_RF_6LOWPAN_H__  extern
#else
#define __SF_RF_6LOWPAN_INIT_VAR__
#endif /* __DECL_SF_RF_6LOWPAN_H__ */

/*! @defgroup sf_6lowpan_rf sf_6lowpan_rf driver
    This group describes the STACKFORCE 6lowpan rf driver.
  @{  */
/*==============================================================================
                            INCLUDE FILES
==============================================================================*/

/*==============================================================================
                               DEFINES
==============================================================================*/

/*==============================================================================
                                MACROS
==============================================================================*/

/*==============================================================================
                            TYPEDEF STRUCT
==============================================================================*/
#include "evproc.h"
#define RF_SEM_POST(_event_)          evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)
#define RF_SEM_WAIT(_event_)          evproc_regCallback(_event_, cc13xx_eventHandler)

/* RSSI threshold for clear channel assessment. Here -100dBm */
#define RF_CCA_RSSI_THR               -90
#define RF_TX_POWER                   (TX_POWER + 130)


typedef enum
{
  /** No signal detected. Channel is free */
  E_RF_CCA_RESULT_IDLE,
  /** Signal detected */
  E_RF_CCA_RESULT_BUSY,
  /* Transceiver is busy with sending or receiving */
  E_RF_CCA_RESULT_INVALID_RF_STATE
}E_RF_CCA_RESULT_t;
/*==============================================================================
                            FUNCTIONS
==============================================================================*/

/*============================================================================*/
/**
 * \brief Initializes the RF module.
 *
 * \param   p_netstk  Pointer to global netstack structure
 *
 * \return    \ref true if successful.
 */
/*============================================================================*/
bool sf_rf_6lowpan_init(void *p_netstk);

/*============================================================================*/
/**
 * @brief   Send data in blocking mode
 *
 * @param   pc_data   Point to buffer storing data to send
 * @param   i_len     Length of data to send
 *
 * \return    \ref true if successful.
 */
/*============================================================================*/
uint8_t sf_rf_6lowpan_sendBlocking(uint8_t *pc_data, uint16_t  i_len);

/*============================================================================*/
/**
 * @brief  Sets the RF module to sleep mode.
 */
/*============================================================================*/
void sf_rf_6lowpan_sleep(void);

/*============================================================================*/
/**
 * @brief  Set the device in rx mode.
 *
 * @return   @ref true if successful.
 */
/*============================================================================*/
bool sf_rf_6lowpan_startRx(void);

/*============================================================================*/
/**
 * @brief  Request the current used TX-power.
 *
 * \return    Power value using the following resolution:
 *            from -130dBm (0x0) to 125dBm (0xFE).
 */
/*============================================================================*/
uint8_t sf_rf_6lowpan_getTxPower(void);

/*============================================================================*/
/**
 * @brief  Set the current used TX-power.
 *
 * @param  c_txPower    Power value using the following resolution:
 *                      from -130dBm (0x0) to 125dBm (0xFE).
 *
 * @return @ref true if successful.
 */
/*============================================================================*/
bool sf_rf_6lowpan_setTxPower(uint8_t c_txPower);

/*============================================================================*/
/**
 * @brief  Request the last received RSSI value.
 *
 * \return    RSSI value using the following resolution:
 *            from 0dBm (0x0) to -255dBm (0xFF).
 */
/*============================================================================*/
uint8_t sf_rf_6lowpan_getRssi(void);

/*============================================================================*/
/**
 * @brief  Preform a clear channel assesment
 *
 * @param c_numOfRssiMeas Number of consecutive RSSI measurements below the
 *                        threshold needed before the channel is declared Idle.
 *
 * @return  @ref E_RF_CCA_RESULT_IDLE if no signal is detected and
 *          @ref E_RF_CCA_RESULT_BUSY if a signal was detected
 */
/*============================================================================*/
E_RF_CCA_RESULT_t sf_rf_6lowpan_cca(uint8_t c_numOfRssiMeas);

/*============================================================================*/
/**
 * @brief  Eventhandler of the CC13xx used by the emb6 stack.
 */
/*============================================================================*/
void cc13xx_eventHandler(c_event_t c_event, p_data_t p_data);


/*============================================================================*/
/**
 * @brief  Select Channel
 */
/*============================================================================*/
bool sf_rf_6lowpan_chanNumSet(uint8_t chan_num);
/*! @} sf_6lowpan_rf */
#endif /* __SF_RF_6LOWPAN_H__ */
