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
 /** New telegram available. */
 #define RF_NEW_TLG                    0xFFFF

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

/* Enumeration for the sf_rf_reset() function. */
typedef enum
{
  /** Running mode */
  E_RF_MODE_RUN,
  /** Waiting */
  E_RF_MODE_WAIT
}E_RF_MODE_t;


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

/** States of RF ISR. */
typedef enum
{
  /** Idle status. */
  E_RF_STATUS_IDLE,
  /** Sleep status. */
  E_RF_STATUS_SLEEP,
  /* Transmit a long preambel. (Used by S-mode) */
  E_RF_STATUS_TX_PREPARE_LONG,
  /** TX: Sending is running. */
  E_RF_STATUS_TX,
  /** RX: Waiting for a valid preamble and syncword */
  E_RF_STATUS_RX_LISTEN,
  /** RX: Receiving is running. */
  E_RF_STATUS_RX,

} E_RF_STATUS_t;

/*==============================================================================
                            TYPEDEF STRUCT
==============================================================================*/
/* Rx config for the device */
typedef struct
{
  rfc_radioOp_t* ps_cmdPropRadioDivSetup;
  rfc_radioOp_t* ps_cmdFs;
  rfc_CMD_PROP_RX_ADV_t* ps_cmdPropRxAdv;
  bool b_return;
} s_rf_rxConf_t;
/*==============================================================================
                            CALLBACKS/EVENTS
==============================================================================*/
/*============================================================================*/
/**
 * \brief Callback which indicates a finished transmission.
 *
 * \param i_len       Length of the transmitted data.
 */
/*============================================================================*/
typedef void (*fp_rf_evt_tx)(uint16_t i_len);

/*============================================================================*/
/**
 * \brief Callback which indicates a new reception.
 *
 * \param i_len       Length of the received data.
 */
/*============================================================================*/
typedef void (*fp_rf_evt_rx)(uint16_t i_len);

/*============================================================================*/
/**
 * \brief Callback which requests rx parameters.
 *
 * \param ps_rxConf       Parameters used for receiving.
 */
/*============================================================================*/
typedef void (*fp_rf_evt_configRx)(s_rf_rxConf_t *ps_rxConf);

/*==============================================================================
                           PRIVATE FUNCTIONS
==============================================================================*/
/* Set the needed parameter to start the rx process */
void sf_rf_setConfigRx(void);
/* Request the state of the transceiver */
E_RF_STATUS_t sf_rf_getRfStatus(void);
/*==============================================================================
                            FUNCTIONS
==============================================================================*/

/*============================================================================*/
/**
 * \brief Initializes the RF module.
 *        To start the RF module the function sf_rf_reset() has to be called.
 *
 * \param e_rpid                   Protocoll identifier
 * \param ps_cmdPropRadioDivSetup  Pointer to the div setup config
 *
 * \return    \ref true if successful.
 */
/*============================================================================*/
bool sf_rf_init(rfc_RPID_t e_rpid, rfc_radioOp_t* ps_cmdPropRadioDivSetup);

/*============================================================================*/
/**
 * \brief  Sets the callbacks for TX and RX events.
 *
 * \param  fp_tx Function pointer to the tx handle function.
 * \param  fp_rx Function pointer to the rx handle function.
 * \param  fp_confRx Function pointer to requests rx parameters.
 *
 * \return true, if successfully, false otherwise.
 */
/*============================================================================*/
bool sf_rf_setCallback(fp_rf_evt_tx fp_tx, fp_rf_evt_rx fp_rx, fp_rf_evt_configRx fp_confRx);

/*============================================================================*/
/**
 * @brief   Starts the rf module.
 */
/*============================================================================*/
void sf_rf_start(void);

/*============================================================================*/
/**
 * @brief Initiates the transmission of data.
 * @param i_len             Total number of bytes to send.
 * @param b_useLongPreamble If this is set to tue a long preamble will be used.
 * @param ps_cmdPropTxAdv   Pointer to the TX-Advanced settings
 * @param ps_cmdPropRadioDivSetup  Pointer to the div setup config
 * @param ps_cmdFs          Pointer to the cmdFs config
 *
 * @return              Returns @ref false if tx-initialisation was not
 *                      successful.
 */
/*============================================================================*/
bool sf_rf_txInit(uint16_t i_len, bool b_useLongPreamble,
                  rfc_CMD_PROP_TX_ADV_t* ps_cmdPropTxAdv,
                  rfc_radioOp_t* ps_cmdPropRadioDivSetup,
                  rfc_radioOp_t* ps_cmdFs);

/*============================================================================*/
/**
 * @brief Sends data. @ref sf_rf_txInit() has to be called before first sending
 *        data (for each telegram).
 *        Sending is finished if the number of really sent bytes reaches the
 *        number of total bytes to send (c.f. sf_rf_txInit).
 * @param pc_data       Pointer to the data to transmit.
 * @param i_len         Number of bytes to transmit.
 * @return              Returns @ref true if the transmission was successful.
 */
/*============================================================================*/
bool sf_rf_txData(uint8_t *pc_data, uint16_t i_len);

/*============================================================================*/
/**
 * @brief Finalizes the transmission of data. At least @ref sf_rf_txInit()
 *        should be called before first.
 * @return              Returns @ref true if the finalisation was successful.
 */
/*============================================================================*/
bool sf_rf_txFinish(void);

/*============================================================================*/
/**
 * @brief Initiates the reception of data.
 * @param pc_quality    Pointer to the memory where to store the link quality.
 *                      The number of quality values may differ between the RF
 *                      modules. The max number of bytes for the quality field
 *                      is specified by @ref RF_QUALITY_LEN.
 *                      NULL if link quality has not to be read.
 *                      The first byte is always the RSSI value:
 *                        0xFF  no link quality available
 *                        FE  -254 dBm
 *                        ...
 *                        00  0 dBm
 * @param c_len         Number of quality bytes to read.
 * @return              Returns @ref false if the receiving parameters could not be
 *                      set successfully.
 */
/*============================================================================*/
bool sf_rf_rxInit(uint8_t *pc_quality, uint8_t c_len);

/*============================================================================*/
/**
 * @brief Retrieves the Rx-data. @ref sf_rf_rxInit() must have been called
          before! Using NULL here means that the internal buffer of the rf
          module will be used. After all bytes are received the function
          @ref sf_rf_getRxStartAddr() can be used to get the start of the
          rx-data array.
 * @param pc_data       Buffer to write the received data into.
 * @param i_len         Number of bytes to receive.
 * @return              @ref true if the data could be received successfully.
 */
/*============================================================================*/
bool sf_rf_rxData(uint8_t *pc_data, uint16_t i_len);

/*============================================================================*/
/**
 * @brief Finalizes the reception of data. At least @ref sf_rf_rxInit() should
 *        be called before.
 * @param e_mode      Indicates if the tranceiver will intermeaditly go in receive mode.
 *                    E_RF_MODE_WAIT: cleans the buffers and set the trx in IDLE mode
 *                    E_RF_MODE_RUN: resets the trx and go in rx mode
 * @return            Returns @ref true if the finalization was successful.
 */
/*============================================================================*/
bool sf_rf_rxFinish(E_RF_MODE_t e_mode);

/*============================================================================*/
/**
 * @brief Resets the RF module.
 * @param e_calibrate   Run calibration and store values, load values or neglect.
 * @return    @ref true if successful.
 */
/*============================================================================*/
bool sf_rf_reset(void);

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
 * @brief   Sets the postamble to use for the packet that is currently being sent
 *
 * @param   e_postamble   Postamble to use.
 * @return   void.
 */
/*============================================================================*/
void sf_rf_txSetPostamble(E_RF_POSTAMBLE_t e_postamble);

/*============================================================================*/
/**
 * @brief Sets the power mode of the rf-module.
 * @param e_powermode     An enumerated value for the wanted mode.
 * @return                @ref true if successful.
 */
/*============================================================================*/
bool sf_rf_setPowerMode(E_RF_POWERMODE_t e_powermode);

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
 * \brief Sets the frequency offset of the carrier.
 *
 * \param si_freqOffset Frequency offset in kHz.
 *
 * \return    \ref true if successful.
 */
/*============================================================================*/
bool sf_rf_setFrequencyOffset(int16_t si_freqOffset);

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

/*============================================================================*/
/**
 * @brief  Control function for the rf module. This function will check if the
           tranceiver is in the right state.
 */
/*============================================================================*/
void sf_rf_run(void);

/*============================================================================*/
/**
 * @brief  Get the start of the receive buffer
 *
 * @return Returns a pointer to the start of the rx buffer. I no rx buffer is
           available NULL will be returned
 */
/*============================================================================*/
uint8_t* sf_rf_getRxStartAddr(void);

/*============================================================================*/
/**
 * @brief  Test function to generate only a modulated carrier wave.
 *         Should be called in a while(true) loop.
 * @return true, if test mode could be activated, ohterwise false
 */
/*============================================================================*/
bool sf_rf_test_generate_carrier_modulated(void);

/*============================================================================*/
/**
 * @brief  Test function to generate only a carrier wave. Should be called
 *         in a while(true) loop.
 * @return true, if test mode could be activated, ohterwise false
 */
/*============================================================================*/
bool sf_rf_test_generate_carrier(void);

/*============================================================================*/
/**
 * @brief  Resets the deviation of the radio module
 *
 * @return true, if deviation was resetted successfully, otherwise false
 */
/*============================================================================*/
bool sf_rf_test_reset_deviation(void);

/*============================================================================*/
/**
 * @brief  Test function to set the radio modul in RX mode. Should be called
 *         in a while(true) loop.
 * @return true, if test mode could be activated, ohterwise false
 */
/*============================================================================*/
bool sf_rf_test_set_to_rx(void);

/*============================================================================*/
/**
 * @brief  Sets the radio module to idle mode.
 *
 * @return true, if radio module could be set to idle mode, otherwiese FLASE.
 */
/*============================================================================*/
bool sf_rf_test_set_to_idle(void);

#endif /* __RF_H__ */
