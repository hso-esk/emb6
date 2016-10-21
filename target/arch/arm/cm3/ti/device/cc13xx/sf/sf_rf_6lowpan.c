/**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       sf_rf_6lowpan.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Implementation of the 6lowpan rf interface.
 */
#ifdef __cplusplus
extern "C" {
#endif

#define __DECL_SF_RF_6LOWPAN_H__
#define __DECL_SF_RF_6LOWPAN_H__

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
#include "emb6.h"
#include "hwinit.h"

/* Stack specific include */
#include "rt_tmr.h"
#include "packetbuf.h"
#include "evproc.h"
#include "phy_framer_802154.h"

#define  LOGGER_ENABLE        LOGGER_RADIO
#include "logger.h"

#include "rflib/rf_cc1350.h"
#include "sf_rf.h"
#include "sf_cc13xx_802_15_4_ch26.h"
#include "sf_rf_6lowpan.h"
#include "bsp/ti_cc13xx/board_conf.h"


/*==============================================================================
                            DEFINES
==============================================================================*/
#ifndef CC13XX_MAX_TELEGRAM_LEN
#error Please define the maximum length needed to send or receive one telegram
#endif /* CC13XX_MAX_TELEGRAM_LEN */

#define RF_SEM_POST(_event_)          evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)
#define RF_SEM_WAIT(_event_)          evproc_regCallback(_event_, cc13xx_eventHandler)
/* RSSI threshold for clear channel assessment. Here -100dBm */
#define RF_CCA_RSSI_THR               -100
#define RF_TX_POWER                   (TX_POWER + 130)
/*==============================================================================
                            VARIABLES
==============================================================================*/
uint16_t gi_bytesToSend;
uint16_t gi_bytesSent;
uint8_t* gpc_rxBuf;
static s_ns_t *ps_rf_netstk;
uint16_t gi_bytesToReceive;
uint16_t gi_bytesReceived;
uint8_t gc_rxRssi;
/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/

void loc_evtRx(uint16_t i_len);
void loc_evtTx(uint16_t i_len);
void loc_configRx(s_rf_rxConf_t* ps_rxConf);
/*==============================================================================
                         LOCAL FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*!
 * @brief Function which handles the reception of 6lowPan telegrams.
 *
 * @param i_len       Length of the received data. @ref RF_NEW_TLG indicates
 *                    the receiption of a valid preamble and syncword
 */
/*============================================================================*/
void loc_evtRx(uint16_t i_len)
{
  /* Check if a syncword of a telegram was received */
  if(i_len == RF_NEW_TLG)
  {
    gi_bytesToReceive = PHY_HEADER_LEN;
    gi_bytesReceived = 0x00U;
    sf_rf_rxInit(&gc_rxRssi, 1U);
    /* Using NULL here means that the internal buffer of the rf module
       will be used */
    sf_rf_rxData(NULL, PHY_HEADER_LEN);
  }/* if */
  else
  {
    gi_bytesReceived += i_len;
    /* Check if the phy header was received completely */
    if(PHY_HEADER_LEN == gi_bytesReceived)
    {
      gpc_rxBuf = sf_rf_getRxStartAddr();
      if(gpc_rxBuf != NULL)
      {
        gi_bytesToReceive += phy_framer802154_getPktLen(gpc_rxBuf, PHY_HEADER_LEN);
      }
      if((gi_bytesToReceive < CC13XX_MAX_TELEGRAM_LEN) && (gpc_rxBuf != NULL))
      {
        /* Using NULL here means that the internal buffer of the rf module
           will be used */
        sf_rf_rxData(NULL, (gi_bytesToReceive-gi_bytesReceived));
      }/* if */
      else
      {
        /* Restart the rx process */
        sf_rf_rxFinish(E_RF_MODE_RUN);
      }
    }/* if */
    else if((gi_bytesReceived >= gi_bytesToReceive) &&
            (gi_bytesReceived > PHY_HEADER_LEN))
    {
#if CC13XX_RX_LED_ENABLED
      bsp_led( E_BSP_LED_3, E_BSP_LED_TOGGLE );
#endif /* #if CC13XX_RX_LED_ENABLED */
      /* signal complete reception interrupt */
      RF_SEM_POST(NETSTK_RF_EVENT);
    }/* if...else if */
  }/* else */
}/* loc_evtRx() */

/*============================================================================*/
/*!
 * @brief Function which handles the transmission of 6lowPan telegrams.
 *
 * @param i_len       Length of the transmitted data.
 */
/*============================================================================*/
void loc_evtTx(uint16_t i_len)
{
  gi_bytesSent += i_len;
}/* loc_evtTx() */

/*============================================================================*/
/**
 * @brief  Sets the configuration for RX.
 *
 * @param ps_rxConf    Pointer to the rx config struct
 */
/*============================================================================*/
void loc_configRx(s_rf_rxConf_t* ps_rxConf)
{
  if(NULL != ps_rxConf)
  {
    ps_rxConf->ps_cmdPropRadioDivSetup = (rfc_radioOp_t*)&RF_802_15_4_ch26_cmdPropRadioDivSetup;
    ps_rxConf->ps_cmdFs = (rfc_radioOp_t*)&RF_802_15_4_ch26_cmdFs;
    ps_rxConf->ps_cmdPropRxAdv = &RF_802_15_4_ch26_cmdPropRxAdv;
    ps_rxConf->b_return = true;
  }/* if */
}/* loc_configRx() */

/*============================================================================*/
/**
 * @brief  Eventhandler of the CC13xx used by the emb6 stack.
 */
/*============================================================================*/
void cc13xx_eventHandler(c_event_t c_event, p_data_t p_data)
{
  /* set the error code to default */
     e_nsErr_t err = NETSTK_ERR_NONE;

  /* finalize reception process */
  if((gi_bytesToReceive == gi_bytesReceived) &&
     (gi_bytesReceived > PHY_HEADER_LEN))
  {
    ps_rf_netstk->phy->recv(gpc_rxBuf, gi_bytesReceived, &err);
    gi_bytesReceived = 0;

    /* The transceiver shall be ready for TX request before
     * signaling upper layer of the received frame */
    sf_rf_rxFinish(E_RF_MODE_RUN);
  }/* if */
}/* cc13xx_eventHandler() */

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/** sf_rf_6lowpan_init() */
/*============================================================================*/
bool sf_rf_6lowpan_init(void *p_netstk)
{
  bool b_return = false;

  /* store pointer to global netstack structure */
  ps_rf_netstk = (s_ns_t *)p_netstk;
  /* Initialize global variabled. Set the lowest RSSI value as default -255dBm */
  gc_rxRssi = 0xFFU;

  sf_rf_setCallback(loc_evtTx, loc_evtRx, loc_configRx);
  b_return = sf_rf_init(RFC_GFSK, (rfc_radioOp_t*)&RF_802_15_4_ch26_cmdPropRadioDivSetup);
  /* Set the signal strength to the default value */
  sf_rf_setSignalStrength(RF_TX_POWER);
  sf_rf_setPowerMode(E_RF_POWERMODE_OFF);

  /* initialize local variables */
  RF_SEM_WAIT(NETSTK_RF_EVENT);

  return b_return;
}/* sf_rf_6lowpan_init() */

/*============================================================================*/
/** sf_rf_6lowpan_sendBlocking() */
/*============================================================================*/
bool sf_rf_6lowpan_sendBlocking(uint8_t *pc_data, uint16_t  i_len)
{
  bool b_return = false;

  if(sf_rf_txInit(i_len, false, &RF_802_15_4_ch26_cmdPropTxAdv,
               (rfc_radioOp_t*)&RF_802_15_4_ch26_cmdPropRadioDivSetup,
               (rfc_radioOp_t*)&RF_802_15_4_ch26_cmdFs) == true)
  {
    gi_bytesToSend = i_len;
    gi_bytesSent = 0x00U;
    if(sf_rf_txData(pc_data, i_len) == true)
    {
      do
      {
        sf_rf_run();
      } while(gi_bytesSent < gi_bytesToSend);
      b_return = true;
    }/* if */
  }/* if */

#if CC13XX_TX_LED_ENABLED
  bsp_led( E_BSP_LED_2, E_BSP_LED_TOGGLE );
#endif /* #if CC13XX_TX_LED_ENABLED */

  /* Set the tranaceiver in rx mode */
  sf_rf_reset();

  return b_return;
}/* sf_rf_6lowpan_sendBlocking() */

/*============================================================================*/
/** sf_rf_6lowpan_sleep() */
/*============================================================================*/
void sf_rf_6lowpan_sleep(void)
{
  sf_rf_sleep();
} /* sf_rf_6lowpan_sleep() */

/*============================================================================*/
/** sf_rf_6lowpan_startRx() */
/*============================================================================*/
bool sf_rf_6lowpan_startRx(void)
{
  return sf_rf_reset();
}/* sf_rf_6lowpan_startRx() */

/*============================================================================*/
/** sf_rf_6lowpan_getTxPower() */
/*============================================================================*/
uint8_t sf_rf_6lowpan_getTxPower(void)
{
  return sf_rf_getSignalStrength();
}/* sf_rf_6lowpan_getTxPower() */

/*============================================================================*/
/** sf_rf_6lowpan_setTxPower() */
/*============================================================================*/
bool sf_rf_6lowpan_setTxPower(uint8_t c_txPower)
{
  return sf_rf_setSignalStrength(c_txPower);
}/* sf_rf_6lowpan_setTxPower() */

/*============================================================================*/
/** sf_rf_6lowpan_getRssi() */
/*============================================================================*/
uint8_t sf_rf_6lowpan_getRssi(void)
{
  return gc_rxRssi;
}/* sf_rf_6lowpan_getRssi() */

/*============================================================================*/
/** sf_rf_6lowpan_cca() */
/*============================================================================*/
E_RF_CCA_RESULT_t sf_rf_6lowpan_cca(uint8_t c_numOfRssiMeas)
{
  /* Return value of this function */
  E_RF_CCA_RESULT_t e_return = E_RF_CCA_RESULT_BUSY;
  /* Structure for the carriere sense command */
  rfc_CMD_PROP_CS_t s_cmdCS;
  #if USE_TI_RTOS
  #else
  /* State of the receiver */
  rfc_cmdStatus_t cmdStatus;
  #endif /* USE_TI_RTOS */

  /* Check if we are in the expected state */
  if(sf_rf_getRfStatus() == E_RF_STATUS_RX_LISTEN)
  {
    /* Fill the carrier sense command */
    /* The command ID number 0x3805 */
    s_cmdCS.commandNo = CMD_PROP_CS;
    /* An integer telling the status of the command.
     * This value is updated by the radio CPU during operation and may be read by the system
     * CPU at any time. */
    s_cmdCS.status = 0U;
    /* Pointer to the next operation to run after this operation is done */
    s_cmdCS.pNextOp = NULL;
    /* Absolute or relative start time (depending on the value of startTrigger) */
    s_cmdCS.startTime = 0U;
    /* The type of trigger. */
    s_cmdCS.startTrigger.triggerType = TRIG_NOW;
    /* 0: No alternative trigger command */
    s_cmdCS.startTrigger.bEnaCmd = 0U;
    /* The trigger number of the CMD_TRIGGER command that triggers this action */
    s_cmdCS.startTrigger.triggerNo = 0U;
    /* 0: A trigger in the past is never triggered, or for start of commands, give an error */
    s_cmdCS.startTrigger.pastTrig = 0U;
    /* Condition for running next command: Rule for how to proceed */
    s_cmdCS.condition.rule = 1U;
    /* Number of skips if the rule involves skipping */
    s_cmdCS.condition.nSkip = 0U;
    /* 0: Keep synth running if command ends with channel Idle */
    s_cmdCS.csFsConf.bFsOffIdle = 0U;
    /* 0: Keep synth running if command ends with channel Busy */
    s_cmdCS.csFsConf.bFsOffBusy = 0U;
    /* If 1, enable RSSI as a criterion. */
    s_cmdCS.csConf.bEnaRssi = 1U;
    /* If 0, disable correlation as a criterion. */
    s_cmdCS.csConf.bEnaCorr = 0U;
    /* 0: Busy if either RSSI or correlation indicates Busy */
    s_cmdCS.csConf.operation = 0U;
    /* 1: End carrier sense on channel Busy */
    s_cmdCS.csConf.busyOp = 1U;
    /* 1: End on channel Idle */
    s_cmdCS.csConf.idleOp = 1U;
    /* 0: Timeout with channel state Invalid treated as Busy */
    s_cmdCS.csConf.timeoutRes = 0U;
    /* Set the RSSI threshold value for a busy channel */
    s_cmdCS.rssiThr = RF_CCA_RSSI_THR;
    /* Number of consecutive RSSI measurements below the threshold needed before
     * the channel is declared Idle.  */
    s_cmdCS.numRssiIdle = c_numOfRssiMeas;
    /* Number of consecutive RSSI measurements above the threshold needed before
     * the channel is declared Busy */
    s_cmdCS.numRssiBusy = c_numOfRssiMeas;
    /* The type of trigger */
    s_cmdCS.csEndTrigger.triggerType = TRIG_NEVER;
    /* 0: No alternative trigger command */
    s_cmdCS.csEndTrigger.bEnaCmd = 0U;
    /* The trigger number of the CMD_TRIGGER command that triggers this action */
    s_cmdCS.csEndTrigger.triggerNo = 0U;
    /* 0: A trigger in the past is never triggered, or for start of commands, give an error */
    s_cmdCS.csEndTrigger.pastTrig = 0U;
    /*  Time used together with csEndTrigger for ending the operation. */
    s_cmdCS.csEndTime = 0U;

    /* Set the tranceiver in a defined power state */
    sf_rf_setPowerMode(E_RF_POWERMODE_IDLE);

    sf_rf_setConfigRx();
    #if USE_TI_RTOS
    #warning Add handling for this command here
    #else
    /* Start carrier sense */
    cmdStatus = RFC_sendRadioOp((rfc_radioOp_t*)&s_cmdCS);

    /* Check if the command was processed correctly */
    if(cmdStatus == RFC_CMDSTATUS_DONE)
    {
      /* Check the status of the CS command */
      if(s_cmdCS.status == PROP_DONE_IDLE)
      {
        e_return = E_RF_CCA_RESULT_IDLE;
      }/* if */
    }/* if */
    #endif /* USE_TI_RTOS */

    /* Set the tranaceiver in rx mode */
    sf_rf_reset();
  }/* if */
  else
  {
    e_return = E_RF_CCA_RESULT_INVALID_RF_STATE;
  }/* if...else */

  return e_return;
}/* sf_rf_6lowpan_cca() */


#ifdef __cplusplus
}
#endif
