
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
#include <cc13xx_cfg.h>
#include "sf_rf_6lowpan.h"
#include "bsp/ti_cc13xx/board_conf.h"


/*==============================================================================
                            DEFINES
==============================================================================*/
#ifndef CC13XX_MAX_TELEGRAM_LEN
#error Please define the maximum length needed to send or receive one telegram
#endif /* CC13XX_MAX_TELEGRAM_LEN */


/*==============================================================================
                            VARIABLES
==============================================================================*/
static s_ns_t *ps_rf_netstk;

/*============================================================================*/
/**
 * @brief  Eventhandler of the CC13xx used by the emb6 stack.
 */
/*============================================================================*/
void cc13xx_eventHandler(c_event_t c_event, p_data_t p_data)
{
  /* set the error code to default */
  e_nsErr_t err = NETSTK_ERR_NONE;

  /* check if it is the right event */
  if(c_event == NETSTK_RF_EVENT )
  {
    ps_rf_netstk->phy->recv(sf_rf_get_p_lastPkt(), sf_rf_get_LenLastPkt(), &err);

    /* The transceiver shall be ready for TX request before
     * signaling upper layer of the received frame */
    // TODO to check this
  }
}

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/** sf_rf_6lowpan_init() */
/*============================================================================*/
bool sf_rf_6lowpan_init(void *p_netstk)
{
  /* store pointer to global netstack structure */
  ps_rf_netstk = (s_ns_t *)p_netstk;

  /* call the routine with init state to initialize the driver */
  if(sf_rf_switchState(RF_STATUS_INIT) != ROUTINE_DONE )
    return false;
  if(sf_rf_switchState(RF_STATUS_RX_LISTEN)!= ROUTINE_DONE)
    return false;
  /* initialize local variables */
  RF_SEM_WAIT(NETSTK_RF_EVENT);

  return true;
}

/*============================================================================*/
/** sf_rf_6lowpan_sendBlocking() */
/*============================================================================*/
uint8_t sf_rf_6lowpan_sendBlocking(uint8_t *pc_data, uint16_t  i_len)
{
    uint8_t status;
  /* init tx cmd */
  if (!sf_rf_init_tx(pc_data,i_len))
  {
    bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
    return 0;
  }
  /* call the routine with tx state to send the packet*/
  status = sf_rf_switchState(RF_STATUS_TX);
  if(status != ROUTINE_DONE && status != ROUTINE_ERROR_TX_NOACK )
  {
      bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
      return 0;
  }
  else
  {
        #if CC13XX_TX_LED_ENABLED
          bsp_led( E_BSP_LED_2, E_BSP_LED_TOGGLE );
        #endif
        /* Set the transceiver in rx mode */
          sf_rf_switchState(RF_STATUS_RX_LISTEN);
        if(status== ROUTINE_DONE)
            return 1;
        else
            return 2;
  }
}

/*============================================================================*/
/** sf_rf_6lowpan_sleep() */
/*============================================================================*/
void sf_rf_6lowpan_sleep(void)
{
  sf_rf_sleep();
}

/*============================================================================*/
/** sf_rf_6lowpan_startRx() */
/*============================================================================*/
bool sf_rf_6lowpan_startRx(void) //  called by the radio : cc13xx_On (e_nsErr_t *p_err)
{
  /* TODO check if we have to initialize/wake */
  if(sf_rf_switchState(RF_STATUS_INIT)!= ROUTINE_DONE)
        return false;
  /* Turn radio to RX mode */
  if(sf_rf_switchState(RF_STATUS_RX_LISTEN)!= ROUTINE_DONE)
      return false;

  return true;
}

/*============================================================================*/
/** sf_rf_6lowpan_getTxPower() */
/*============================================================================*/
uint8_t sf_rf_6lowpan_getTxPower(void)
{
  return sf_rf_getTxPower();
}

/*============================================================================*/
/** sf_rf_6lowpan_setTxPower() */
/*============================================================================*/
bool sf_rf_6lowpan_setTxPower(uint8_t c_txPower)
{
  return sf_rf_setTxPower(c_txPower);
}

/*============================================================================*/
/** sf_rf_6lowpan_getRssi() */
/*============================================================================*/
uint8_t sf_rf_6lowpan_getRssi(void)
{
  return sf_rf_getRssi();
}

/*============================================================================*/
/** sf_rf_6lowpan_cca() */
/*============================================================================*/
E_RF_CCA_RESULT_t sf_rf_6lowpan_cca(uint8_t c_numOfRssiMeas)
{
  E_RF_CCA_RESULT_t e_return;
  uint8_t returnValue;

  /* set number of RSSI measurement */
  sf_rf_set_numOfRssiMeas(c_numOfRssiMeas);
  /* Check if we are in the expected state: TODO check if this needed */
    returnValue=sf_rf_switchState(RF_STATUS_CCA);
    if(returnValue == ROUTINE_CCA_RESULT_IDLE)
      e_return = E_RF_CCA_RESULT_IDLE;
    else if (returnValue == ROUTINE_CCA_RESULT_BUSY )
      e_return = E_RF_CCA_RESULT_BUSY;
    else
      e_return = E_RF_CCA_RESULT_INVALID_RF_STATE;
  /* Set the transceiver in RX mode */
  sf_rf_switchState(RF_STATUS_RX_LISTEN);
  return e_return;
}


bool sf_rf_6lowpan_chanNumSet(uint8_t chan_num)
{
    uint8_t retVal;
    uint16_t Cent_freq;
    uint16_t Frac_Freq;
    uint16_t Delta;
    uint8_t OpMode=1;
    /* TODO Check all case (sleepy mode)
     * either wake up and configure ans sleep again
     * or
     * update struct and wake up fct with setup the radio with new freq
     * */
    if( (sf_rf_get_Status()== RF_STATUS_RX_LISTEN) && chan_num <= 33 )
    {

        /* check the operation mode */
        switch(OpMode)
        {
        case 1 :
            Cent_freq=0x035F;
            Frac_Freq=0x2000;
            Delta=0x3333;
            break;
        default:
            return false;
        }

        /* calculate the frequency parameters for desired channel */
        Cent_freq = Cent_freq + (chan_num / 5);
        Frac_Freq = Frac_Freq + (chan_num % 5) * Delta;
        /* update frequency */
        if(sf_rf_update_frequency(Cent_freq, Frac_Freq))
          retVal= true;
        else
          retVal= false;
        /* Set the transceiver in RX mode */
        sf_rf_switchState(RF_STATUS_RX_LISTEN);
    }
    else
        retVal= false;
return retVal;
}

#ifdef __cplusplus
}
#endif
