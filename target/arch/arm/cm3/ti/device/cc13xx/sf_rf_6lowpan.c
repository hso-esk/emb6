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

/* Stack specific include */
#include "packetbuf.h"
#include "evproc.h"
#include "phy_framer_802154.h"

#define  LOGGER_ENABLE        LOGGER_RADIO
#include "logger.h"

#if USE_TI_RTOS
#include <ti/drivers/rf/RF.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_prop_cmd.h>
#include <driverlib/rf_prop_mailbox.h>
#endif /* USE_TI_RTOS */

#include "rflib/rf_cc1350.h"

#include "sf_rf.h"
#include "rf_settings/sf_cc13xx_802_15_4_ch26.h"
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
  uint16_t i_phyPktLen = 0U;

  /* Check if a syncword of a telegram was received */
  if(i_len == RF_NEW_TLG)
  {
    /* Reset variables. */
    gi_bytesToReceive = PHY_HEADER_LEN;;
    gi_bytesReceived = 0x00U;

    /* Initialize reception. */
    sf_rf_rxInit(&gc_rxRssi, 1U);

    /* Store start address of the received tlg. */
    gpc_rxBuf = sf_rf_getRxStartAddr();

    if(gpc_rxBuf != NULL)
    {
      /* Get the packet length. */
      i_phyPktLen = phy_framer802154_getPktLen(gpc_rxBuf, PHY_HEADER_LEN);

      if(i_phyPktLen)
      {
        /* Update the amount of bytes to receive. */
        gi_bytesToReceive += i_phyPktLen;
      }
      else
      {
        /* Invalid packet length returned from phy framer. Abort reception. */
        gi_bytesToReceive = CC13XX_MAX_TELEGRAM_LEN;
      }/* if ..else */

      if((gi_bytesToReceive < CC13XX_MAX_TELEGRAM_LEN))
      {
        /* Using NULL here means that the internal buffer of the rf module
           will be used */
        sf_rf_rxData(NULL, gi_bytesToReceive);
      }
      else
      {
        /* Restart the RX process. */
         sf_rf_rxFinish(E_RF_MODE_RUN);
      }/* if() .. else */
    }/* if() */
  }/* if */
  else
  {
    /* Update the number of received bytes. */
    gi_bytesReceived += i_len;

    /* Check if all bytes have been received. */
    if((gi_bytesReceived >= gi_bytesToReceive) &&
            (gi_bytesReceived > PHY_HEADER_LEN))
    {
      /* Signal complete reception interrupt. */
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
 * @brief  Event handler of the CC13xx used by the emb6 stack.
 */
/*============================================================================*/
void cc13xx_eventHandler(c_event_t c_event, p_data_t p_data)
{
  /* set the error code to default */
     e_nsErr_t err = NETSTK_ERR_NONE;

  /* finalize reception process */
  if(c_event == NETSTK_RF_EVENT)
  {
    if((gi_bytesToReceive == gi_bytesReceived) &&
    (gi_bytesReceived > PHY_HEADER_LEN))
    {
      ps_rf_netstk->phy->recv(gpc_rxBuf, gi_bytesReceived, &err);
      gi_bytesReceived = 0;

      /* The transceiver shall be ready for TX request before
       * signaling upper layer of the received frame */
      sf_rf_rxFinish(E_RF_MODE_RUN);
    }
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
E_RF_6LOWPAN_CCA_RESULT_t sf_rf_6lowpan_cca(uint8_t c_numOfRssiMeas)
{
  E_RF_6LOWPAN_CCA_RESULT_t e_ccaResult = E_RF_6LOWPAN_CCA_RESULT_INVALID_RF_STATE;

  /* Map the return values. */
  switch(sf_rf_cca(c_numOfRssiMeas))
  {
    case E_RF_CCA_RESULT_IDLE:
      e_ccaResult = E_RF_6LOWPAN_CCA_RESULT_IDLE;
      break;
    case E_RF_CCA_RESULT_BUSY:
      e_ccaResult = E_RF_6LOWPAN_CCA_RESULT_BUSY;
      break;
    case E_RF_CCA_RESULT_INVALID_RF_STATE:
    default:
      e_ccaResult = E_RF_6LOWPAN_CCA_RESULT_INVALID_RF_STATE;
      break;
  }/* switch() */

  return e_ccaResult;
}/* sf_rf_6lowpan_cca() */


#ifdef __cplusplus
}
#endif
