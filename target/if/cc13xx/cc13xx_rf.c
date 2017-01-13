#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       cc13xx_rf.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      This is the 6lowpan-stack driver for the rf module.
 */

/*! @defgroup emb6_if emb6 stack if driver
    This group is the if driver for the emb6 stack.
  @{  */

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include "emb6.h"
#include "hwinit.h"
#include "phy_framer_802154.h"

#include "packetbuf.h"
#include "evproc.h"

#include "sf_rf_6lowpan.h"

/*! Enable or disable logging. */
#define     LOGGER_ENABLE        LOGGER_RADIO
#include    "logger.h"

/*============================================================================*/
/*                               LOCAL MACROS                                 */
/*============================================================================*/
/* This is the minimum value which can be disbplayed using a signed 8 bit
 * integer =-127dBm */
#define CC13xx_MIN_RSSI            128U
#define CC13xx_MIN_RSSI_SIGNED     -128
/* Number of RSSI checks that must be lower than the defined threshold */
#define CC13xx_NUM_OFF_RSSI_CHECKS 5U
/*============================================================================*/
/*                           LOCAL VARIABLES                                  */
/*============================================================================*/

/*============================================================================*/
/*                       LOCAL FUNCTIONS DECLARATION                          */
/*============================================================================*/
static void cc13xx_Init (void *p_netstk, e_nsErr_t *p_err);
static void cc13xx_On (e_nsErr_t *p_err);
static void cc13xx_Off (e_nsErr_t *p_err);
static void cc13xx_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void cc13xx_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err);
static void cc13xx_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/
/*! Offset used by the CC13xx transceiver to display the tx power */
#define CC13XX_TX_POWER_OFFSET     130U
/*! Maximum TX-output power in dBm */
#define CC13XX_TX_MAX_POWER        14
/*! Minimum TX-output power in dBm */
#define CC13XX_TX_MIN_POWER        -10
/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/

/*============================================================================*/
/*                                  CONSTANTS                                 */
/*============================================================================*/

/*============================================================================*/
/*                                LOCAL VARIABLES                             */
/*============================================================================*/

/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/
static void loc_cc13xx_txPowerGet(int8_t *pc_txPower, e_nsErr_t *p_err);
static void loc_cc13xx_txPowerSet(int8_t *pc_txPower, e_nsErr_t *p_err);
static void loc_cc13xx_getRssi(int8_t *pc_rssi, e_nsErr_t *p_err);
static void loc_cc13xx_cca(e_nsErr_t *p_err);
/*============================================================================*/
/*                                ISR PROTOTYPES                              */
/*============================================================================*/

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/**
 * @brief   Request the tx power used by the transceiver.
 *
 * @param   p_err       Pointer to result enum.
 *
 * @param   pc_txPower Pointer to store the tx power in.
 */
static void loc_cc13xx_txPowerGet(int8_t *pc_txPower, e_nsErr_t *p_err)
{
  /* Requested power value from the transceiver */
  uint8_t c_txPower;

  /* Request the power value used by the transceiver */
  c_txPower = sf_rf_6lowpan_getTxPower();

  /* Convert the transceiver value for the emb6-stack.
   * The transceiver resolution: from -130dBm (0x0) to 125dBm (0xFE). */
  *pc_txPower = c_txPower - CC13XX_TX_POWER_OFFSET;

  *p_err = NETSTK_ERR_NONE;
}/* loc_cc13xx_txPowerGet() */

/**
 * @brief   Set the tx power used by the transceiver.
 *
 * @param   p_err       Pointer to result enum.
 *
 * @param   pc_txPower Pointer to the tx-power to set.
 */
static void loc_cc13xx_txPowerSet(int8_t *pc_txPower, e_nsErr_t *p_err)
{
  /* Power value to be set */
  uint8_t c_txPower;
  bool b_return = false;

  if((*pc_txPower <= CC13XX_TX_MAX_POWER) &&
     (*pc_txPower >= CC13XX_TX_MIN_POWER))
  {
    /* Convert the stack value in to a value used by the rf module */
    c_txPower = *pc_txPower + CC13XX_TX_POWER_OFFSET;
    b_return = sf_rf_6lowpan_setTxPower(c_txPower);
  }
  /* Set the tx power value */
  if(b_return)
  {
    *p_err = NETSTK_ERR_NONE;
  }
  else
  {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
  }
}/* loc_cc13xx_txPowerSet() */

/**
 * @brief   Get the last received RSSI value
 *
 * @param   p_err       Pointer to result enum.
 *
 * @param   pc_rssi Pointer to store the RSSI value in.
 */
static void loc_cc13xx_getRssi(int8_t *pc_rssi, e_nsErr_t *p_err)
{
  uint8_t c_rssi;

  /* The resoloution of the rf is 0dbm to -255dbm */
  c_rssi = sf_rf_6lowpan_getRssi();

  /* Convert value in to a signed 8bit for the 6lowpan stack. */
  if(c_rssi >= CC13xx_MIN_RSSI)
  {
    *pc_rssi = CC13xx_MIN_RSSI_SIGNED;
  }/* if */
  else
  {
    *pc_rssi = (~c_rssi) + 1;
  }/* if */

  *p_err = NETSTK_ERR_NONE;
}/* loc_cc13xx_getRssi() */

/**
 * @brief   Preform a clear channel assesment and return @ref NETSTK_ERR_NONE if
 *          no signal is detected and @ref NETSTK_ERR_CHANNEL_ACESS_FAILURE if a
 *          signal was detected
 *
 * @param   p_err       Pointer to result enum.
 */
static void loc_cc13xx_cca(e_nsErr_t *p_err)
{
  switch(sf_rf_6lowpan_cca(CC13xx_NUM_OFF_RSSI_CHECKS))
  {
    case E_RF_CCA_RESULT_IDLE:
      /* No signal detected */
      *p_err = NETSTK_ERR_NONE;
      break;
    case E_RF_CCA_RESULT_BUSY:
      /* Signal detected */
      *p_err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
      break;
    case E_RF_CCA_RESULT_INVALID_RF_STATE:
      /* The device is busy during other opperations */
      *p_err = NETSTK_ERR_BUSY;
      break;
  }/* switch */
}

/*============================================================================*/
/*                           API FUNCTION DEFINITIONS                         */
/*============================================================================*/

/*!
 * @brief   This function initializes the radio driver.
 *
 * @param   p_netstk Pointer to the netstack structure.
 * @param   p_err    Pointer to result enum.
 */
static void cc13xx_Init (void *p_netstk, e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }

  if (p_netstk == NULL)
  {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
  #endif /* NETSTK_CFG_ARG_CHK_EN */

  if(sf_rf_6lowpan_init(p_netstk))
  {
    *p_err = NETSTK_ERR_NONE;
  }/* if */
  else
  {
    *p_err = NETSTK_ERR_INIT;
  }/* else */
}


/*!
 * @brief   This function turns the radio transceiver on.
 *
 * @param   p_err  Pointer to result enum.
 */
static void cc13xx_On (e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }
  #endif

  *p_err = NETSTK_ERR_NONE;
  if(!sf_rf_wake())
    *p_err = NETSTK_ERR_INIT;
}


/*!
 * @brief   This function turns the radio transceiver off.
 *
 * @param   p_err   Pointer to result enum.
 */
static void cc13xx_Off (e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }
  #endif

  *p_err = NETSTK_ERR_NONE;
   sf_rf_6lowpan_sleep();
}


/*!
 * @brief   This function transmits data.
 *
 * @param   p_data      Point to buffer storing data to send.
 * @param   len         Length of data to send.
 * @param   p_err       Pointer to result enum.
 */
static void cc13xx_Send (uint8_t      *p_data,
                         uint16_t     len,
                         e_nsErr_t    *p_err)
{
    uint8_t retVal;
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }

  if ((p_data == NULL) || (len == 0))
  {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
  #endif

  if (len > CC13XX_MAX_TELEGRAM_LEN)
  {
    /* packet length is out of range, and therefore transmission is
     * refused */
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
  retVal=sf_rf_6lowpan_sendBlocking(p_data, len);
  if(retVal==1)
  {
    *p_err = NETSTK_ERR_NONE;
  }
  else if(retVal==2)
  {
    *p_err = NETSTK_ERR_TX_NOACK;
  }
  else
  {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
  }
}

/*!
 * @brief   This function receives data.
 *
 * @param   p_buf   Point to buffer storing data to send.
 * @param   len     Length of received to send.
 * @param   p_err   Pointer to result enum.
 */
static void cc13xx_Recv (uint8_t    *p_buf,
                         uint16_t   len,
                         e_nsErr_t  *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }/* if */
  #endif

}

/*!
 * @brief   Input/Output control
 *
 * @param cmd         IOCTL Command.
 * @param p_val       If wanted to assign value to a information base attribute,
 *                    p_val points to value to set.
 *                    If wanted to read value of a information base attribute,
 *                    p_val points to storing buffer.
 * @param p_err       Pointer to result enum.
 */
static void cc13xx_Ioctl (e_nsIocCmd_t    cmd,
                          void            *p_val,
                          e_nsErr_t       *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }
  #endif

  *p_err = NETSTK_ERR_NONE;
  switch (cmd)
  {
    case NETSTK_CMD_RF_TXPOWER_SET:
      loc_cc13xx_txPowerSet((int8_t*)p_val, p_err);
      break;
    case NETSTK_CMD_RF_TXPOWER_GET:
      loc_cc13xx_txPowerGet((int8_t*)p_val, p_err);
      break;

    case NETSTK_CMD_RF_CCA_GET:
      loc_cc13xx_cca(p_err);
      break;
    case NETSTK_CMD_RF_RSSI_GET:
      loc_cc13xx_getRssi((int8_t*)p_val, p_err);
      break;

    case NETSTK_CMD_RX_BUF_READ:
    /*
     * Signal upper layer if a packet has arrived by the time this
     * command is issued.
     * Trigger event-process manually
     */
    cc13xx_eventHandler(NETSTK_RF_EVENT, NULL);
    break;


    case NETSTK_CMD_RF_RF_SWITCH_SET:
    case NETSTK_CMD_RF_ANT_DIV_SET:
    case NETSTK_CMD_RF_SENS_SET:
    case NETSTK_CMD_RF_SENS_GET:
    case NETSTK_CMD_RF_IS_RX_BUSY:
    case NETSTK_CMD_RF_CHAN_NUM_SET:
        /* set the desired channel */
        if(sf_rf_6lowpan_chanNumSet(*(uint8_t*)p_val))
            *p_err = NETSTK_ERR_NONE;
        else
            *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    case NETSTK_CMD_RF_WOR_EN:
    default:
      /* unsupported commands are treated in same way */
      *p_err = NETSTK_ERR_CMD_UNSUPPORTED;
      break;
    }/* switch */
}

/*============================================================================*/
/*                             DRIVER DEFINITION                              */
/*============================================================================*/
/*! Definition of the if driver. The emb6 stack uses this structure to access
    the if module. */
const s_nsRF_t rf_driver_ticc13xx =
{
    "CC13xx",
    cc13xx_Init,
    cc13xx_On,
    cc13xx_Off,
    cc13xx_Send,
    cc13xx_Recv,
    cc13xx_Ioctl,
};

/*! @} 6lowpan_if */

#ifdef __cplusplus
}
#endif
