 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       sf_rf.h
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Internal defines, structures, enums and constants for the transceiver.
 */

#ifndef __RF_H__
#define __RF_H__

#ifndef __DECL_RF_H__
#define __DECL_RF_H__ extern
#else
#define __RF_INIT_VAR__
#endif /* __DECL_RF_H__ */

/*==============================================================================
                            DEFINES
==============================================================================*/

#include "rflib/rf_queue.h"
#include "rflib/rf_cc1350.h"

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

#define CC13xx_NUM_OFF_RSSI_CHECKS_DEFAULT 5U

typedef struct
{
  rfc_radioOp_t* ps_cmdPropRadioDivSetup;
  rfc_radioOp_t* ps_cmdFs;
} s_rf_Conf_t;

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


uint8_t         rf_driver_routine(e_rf_status_t stat);
e_rf_status_t   sf_rf_get_Status(void);
uint8_t         sf_rf_getRssi(void);
uint8_t         sf_rf_init_tx(uint8_t *pc_data, uint16_t  i_len);
uint8_t*        sf_rf_get_p_lastPkt(void);
uint16_t        sf_rf_get_LenLastPkt(void);
bool            sf_rf_set_numOfRssiMeas(uint8_t num);

/*==============================================================================
                            TYPEDEF ENUMS
==============================================================================*/

/** Enumeration of possible low power modi. */
typedef enum
{
  /** Shutdown mode */
  E_RF_POWERMODE_OFF,
  /** Low power, register settings kept */
  E_RF_POWERMODE_IDLE,
  /** Active power mode for Rx and Tx */
  E_RF_POWERMODE_RX,
  /** Active power mode */
  E_RF_POWERMODE_MAX
} E_RF_POWERMODE_t;



/** Possible postambles */
typedef enum
{
  /** No postamble to send. */
  E_RF_POSTAMBLE_NONE = 0x00,
  /** 0101 postamble to send. */
  E_RF_POSTAMBLE_ODD = 0x55,
  /** 1010 postamble to send. */
  E_RF_POSTAMBLE_EVEN = 0xAA
} E_RF_POSTAMBLE_t;


/*==============================================================================
                            FUNCTIONS
==============================================================================*/


/*============================================================================*/
/**
 * @brief Sets the channel to use for RF communication.
 * @param ps_cmdPropRadioDivSetup  Pointer to the div setup config
 * @param ps_cmdFs                 Pointer to the cmdFs config
 * @return                         @ref true if successful.
 */
/*============================================================================*/
bool sf_rf_setRfChannel(rfc_radioOp_t* ps_cmdPropRadioDivSetup,
                        rfc_radioOp_t* ps_cmdFs);



/*============================================================================*/
/**
* @brief Sets the signal strength of the rf-module.
 * @param c_signal       Signal strength from -130dBm (0x0) to 125dBm (0xFE).
 *                       0xFF is reserved. If c_signal exceeds the minimum ore
 *                       maximum tx power settings the outputpower will be set
 *                       to the next supported value.
 */
/*============================================================================*/
bool sf_rf_setSignalStrength(uint8_t c_signal);

/*============================================================================*/
/**
 * @brief Get the signal strength of the rf-module.
 * @return               Current signal strength from -130dBm (0x0) to 125dBm (0xFE).
 *                       If 0xFF is returned the value of in the PATABLE is
 *                       not known.
 */
/*============================================================================*/
uint8_t sf_rf_getSignalStrength(void);

/*============================================================================*/
/**
 * @brief  Sets the RF module to sleep mode.
 */
/*============================================================================*/
void sf_rf_sleep(void);

/*============================================================================*/
/**
 * @brief  Wakes the RF module up.
 */
/*============================================================================*/
void sf_rf_wake(void);


#endif /* __RF_H__ */
