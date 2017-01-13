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

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/

#include "rflib/rf_queue.h"
#include "rflib/rf_cc1350.h"
#include "evproc.h"

/*==============================================================================
                               DEFINES/MACROS
==============================================================================*/

#define RF_SEM_POST(_event_)          evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)
#define RF_SEM_WAIT(_event_)          evproc_regCallback(_event_, cc13xx_eventHandler)

/* RSSI threshold for clear channel assessment. Here -100dBm */
#define RF_CCA_RSSI_THR               -90
#define RF_TX_POWER                   (TX_POWER + 130)

#define ROUTINE_DONE                   0x00
#define ROUTINE_ERROR_INIT_QUEUE       0x01
#define ROUTINE_ERROR_ENABLE_RADIO     0x02
#define ROUTINE_ERROR_FS_CMD           0x03
#define ROUTINE_ERROR_INIT_RF          0x04
#define ROUTINE_ERROR_TX_CMD           0x05
#define ROUTINE_ERROR_RX_CMD           0x06
#define ROUTINE_CCA_RESULT_IDLE        0x07
#define ROUTINE_CCA_RESULT_BUSY        0x08
#define ROUTINE_CCA_ERROR_STATE        0x09
#define ROUTINE_ERROR_SETUP_RADIO      0x10
#define ROUTINE_ERROR_TX_NOACK         0x11
#define ROUTINE_FATAL_ERROR            0x12
#define ROUTINE_ERROR_DISABLE_RADIO    0x13

#define CC13xx_NUM_OFF_RSSI_CHECKS_DEFAULT     5U
#define RX_BUFF_SIZE                           400

/*==============================================================================
                            TYPEDEF ENUMS/STRUCT
==============================================================================*/

typedef enum
{
  /** No signal detected. Channel is free */
  E_RF_CCA_RESULT_IDLE,
  /** Signal detected */
  E_RF_CCA_RESULT_BUSY,
  /* Transceiver is busy with sending or receiving */
  E_RF_CCA_RESULT_INVALID_RF_STATE
}E_RF_CCA_RESULT_t;

typedef enum
{
  /** None status  */
  RF_STATUS_NONE,
  /** Init status  */
  RF_STATUS_INIT,
  /** Idle status. */
  RF_STATUS_IDLE,
  /** Sleep status. */
  RF_STATUS_SLEEP,
  /** TX: Sending is running. */
  RF_STATUS_TX,
  /** RX: Waiting for a valid preamble and syncword */
  RF_STATUS_RX_LISTEN,
  /** RX: Receiving is running. */
  RF_STATUS_RX,
  /** Performing CCA */
  RF_STATUS_CCA,
  /** Error status. */
  RF_STATUS_ERROR
} e_rf_status_t;

typedef struct
{
  rfc_radioOp_t* ps_cmdPropRadioDivSetup;
  rfc_radioOp_t* ps_cmdFs;
} s_rf_Conf_t;

typedef struct
{
  uint8_t *p_lastPkt;
  uint16_t LenLastPkt;

  /** Pointer to the buffer of the higher layer: used to copy received data from queue */
  uint8_t *p_appBuff;
  /** Receive dataQueue for RF Core to fill in data */
  dataQueue_t dataQueue;
  /** Pointer to the current data entry */
  rfc_dataEntryGeneral_t* p_currentDataEntry;
  // rfc_dataEntryPartial_t* p_currentDataEntry;
  /** The RX Output struct contains statistics about the RX operation of the radio. */
  rfc_propRxOutput_t rxStatistics;
  /** Pointer to the current rx command */
  rfc_CMD_PROP_RX_ADV_t* p_cmdPropRxAdv;
  /* Meassured RSSI value */
  uint8_t c_rssiValue;
} st_rx_cmd_t;

typedef struct
{
  /** flag used to mention that the radio layer is waiting for ACK after transmitting a packet */
    uint8_t Tx_waiting_Ack;
  /** expected sequence number for the ACK */
    uint8_t expSeqNo;
  /** Time out for waiting ACK  */
    uint16_t waitForAckTimeout;
  /** flag to set when an is received */
    uint8_t Ack_received;
  /** Signal Strength */
    uint8_t signalStrength;
  /** Pointer to the current tx command */
  rfc_CMD_PROP_TX_ADV_t* p_cmdPropTxAdv;
} st_tx_cmd_t;

typedef struct
{
  /** Number of consecutive RSSI measurements below the
          threshold needed before the channel is declared Idle */
  uint8_t c_numOfRssiMeas;
  /* cmd prop cs */
  rfc_CMD_PROP_CS_t s_cmdCS;
}st_cca_cmd_t;

typedef struct
{
  e_rf_status_t state;
  s_rf_Conf_t   conf;
  st_tx_cmd_t   tx;
  st_rx_cmd_t   rx;
  st_cca_cmd_t  cca;
} st_cc1310_t;






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
bool sf_rf_wake(void);

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

bool sf_rf_6lowpan_chanNumSet(uint8_t chan_num);

void cc13xx_eventHandler(c_event_t c_event, p_data_t p_data);

#endif /* __SF_RF_6LOWPAN_H__ */
