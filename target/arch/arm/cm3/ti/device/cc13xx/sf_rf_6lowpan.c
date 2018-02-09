
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
#include "target_conf.h"

/* Stack specific include */
#include "rt_tmr.h"
#include "packetbuf.h"
#include "evproc.h"

#define  LOGGER_ENABLE        LOGGER_RADIO
#include "logger.h"

#include "cc13xx_cfg.h"
#include "sf_rf_6lowpan.h"
#include "bsp/ti_cc13xx/board_conf.h"
/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>
#include "rflib/rf_dbell.h"
#include "driverlib/rf_mailbox.h"
#include "bsp.h"
#include "framer_802154_ll.h"
#include "phy_framer_802154.h"

#if USE_TI_RTOS
#include <ti/drivers/rf/RF.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_prop_cmd.h>
#include <driverlib/rf_prop_mailbox.h>
#include <xdc/runtime/Error.h>
#include <rf_patches/rf_patch_cpe_genfsk.h>
#include <rf_patches/rf_patch_rfe_genfsk.h>
#include "emb6_semaphore.h"
#include "sf_mcu_sleep.h"
#else
/* BoardSupportPacket */
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"
#include "rtimer.h"
#endif
/*==============================================================================
                            DEFINES
==============================================================================*/
#ifndef CC13XX_TX_ENABLED
#error Please define if the device support TX
#endif /* CC13XX_TX_ENABLED */

#ifndef CC13XX_RX_ENABLED
#error Please define if the device support RTX
#endif /* CC13XX_RX_ENABLED */

#ifndef CC13XX_MAX_TELEGRAM_LEN
#error Please define the maximum length needed to send or receive one telegram
#endif /* CC13XX_MAX_TELEGRAM_LEN */


/*==============================================================================
                            VARIABLES
==============================================================================*/
static s_ns_t *ps_rf_netstk;

st_cc1310_t cc1310;
uint8_t RxBuff[RX_BUFF_SIZE];
framer802154ll_attr_t frame;
uint8_t ack[10];
uint8_t ack_length;

#if USE_TI_RTOS
/* RF parameter struct */
static RF_Params gs_rfParams;
static RF_Object gs_rfObject;
static RF_Handle gps_rfHandle;
RF_CmdHandle gps_rf_cmdHandle;
#endif

/* Threshold value for the rx process. */
#define RF_RX_THRESHOLD        0x05U

/* Defines the default TX-power of the device: Here 0dBm is used */
#define RF_DEFAULT_TX_POWER    130U


/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/

static bool rf_driver_init(void);
static uint8_t sf_rf_switchState(e_rf_status_t state);
static e_rf_status_t sf_rf_get_Status(void);
static uint8_t sf_rf_getRssi(void);
static uint8_t sf_rf_init_tx(uint8_t *pc_data, uint16_t  i_len);
static uint8_t* sf_rf_get_p_lastPkt(void);
static uint16_t sf_rf_get_LenLastPkt(void);
static bool sf_rf_set_numOfRssiMeas(uint8_t num);
static bool sf_rf_update_run_frequency(uint16_t frequency, uint16_t fractFreq);
static bool sf_rf_update_frequency(uint16_t frequency, uint16_t fractFreq);
static void rx_call_back(uint32_t l_flag);

static void sf_rf_init_cca_cmd(void);
#if USE_TI_RTOS
static void Rtos_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
#else
static void loc_switchToHfXtalOsc(void);
#endif


/*==============================================================================
                           API FUNCTIONS
==============================================================================*/

void cc13xx_eventHandler(c_event_t c_event, p_data_t p_data)
{
  /* set the error code to default */
  e_nsErr_t err = NETSTK_ERR_NONE;

  /* check if it is the right event */
  if(c_event == EVENT_TYPE_RF )
  {
    ps_rf_netstk->phy->recv(sf_rf_get_p_lastPkt(), sf_rf_get_LenLastPkt(), &err);
  }
}


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
  RF_SEM_WAIT(EVENT_TYPE_RF);

  return true;
}

uint8_t sf_rf_6lowpan_prepare_pkt(uint8_t *payload, uint16_t payload_len)
{
	  if (payload!=NULL && payload_len > 1)
	  {
	    cc1310.tx.p_cmdPropTxAdv->pPkt = payload;
	    cc1310.tx.p_cmdPropTxAdv->pktLen = payload_len ;
	  }
	  else
		return 0;
  cc1310.tx.p_cmdPropTxAdv->pktLen += PHY_HEADER_LEN;
  cc1310.tx.p_cmdPropTxAdv->pPkt -= PHY_HEADER_LEN;
  sf_rf_init_tx(cc1310.tx.p_cmdPropTxAdv->pPkt, cc1310.tx.p_cmdPropTxAdv->pktLen);
  return 1;
}
#if !USE_TI_RTOS
uint8_t tx_cmd_done = 0;
static void tx_cmd_done_cb(uint32_t l_flag)
{
  if(IRQ_LAST_COMMAND_DONE == (l_flag & IRQ_LAST_COMMAND_DONE))
  {
	  tx_cmd_done = 1;
  }
}
#endif

uint8_t sf_rf_transmit(void)
{
#if USE_TI_RTOS
  RF_EventMask result ;
#else
  static rfc_cmdStatus_t cmdStatus;
#endif

  /* check if transceiver is not is sleepy mode */
  if( sf_rf_get_Status() == RF_STATUS_SLEEP )
    return 0;
#if !USE_TI_RTOS
  tx_cmd_done = 0;
#endif
  /* reset the rx flag */
  cc1310.rx.is_receiving = 0;

#if USE_TI_RTOS

      /* Stop last cmd (usually it is Rx cmd ) */
      RF_flushCmd(gps_rfHandle, gps_rf_cmdHandle, 1);
      /* Send tx command  */
      result = RF_runCmd(gps_rfHandle, (RF_Op*)cc1310.tx.p_cmdPropTxAdv, RF_PriorityNormal, NULL, 0);
      if (!(result & RF_EventLastCmdDone))
      {
        return 0;
      }
#else
      /* Stop last cmd (usually it is Rx cmd ) */
      RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
      /* Send tx command  */
      // cmdStatus = RFC_sendRadioOp((rfc_radioOp_t*)cc1310.tx.p_cmdPropTxAdv);
      cmdStatus = RFC_sendRadioOp_nb((rfc_radioOp_t*)cc1310.tx.p_cmdPropTxAdv, tx_cmd_done_cb);

      while(!tx_cmd_done);

      /* Since we sent a blocking cmd then we check the state */
      if(cmdStatus != RFC_CMDSTATUS_DONE)
      {
        return 0 ;
      }
#endif
      return 1;
}

void set_polling_mode(void)
{
  cc1310.poll_mode=1;
  /* activate timestamp*/
  cc1310.rx.p_cmdPropRxAdv->rxConf.bAppendTimestamp=1;
}

uint8_t is_polling_mode(void)
{
  /* return is polling flag  */
  return cc1310.poll_mode;
}

uint8_t receiving_packet(void)
{
 /* return is receiving flag  */
 return cc1310.rx.is_receiving ;
}

 uint8_t pending_packet(void)
{
  uint8_t rv = 0;
  volatile rfc_dataEntry_t *entry = (rfc_dataEntry_t *)cc1310.rx.p_currentDataEntry;

  /* Go through all RX buffers and check their status */
  do {
    if(entry->status == DATA_ENTRY_FINISHED) {
      rv += 1;
    }
    entry = (rfc_dataEntry_t *)entry->pNextEntry;
  } while(entry != (rfc_dataEntry_t *)cc1310.rx.p_currentDataEntry);

  /* If we didn't find an entry at status finished, no frames are pending */
  return rv;
}

 uint8_t sf_rf_set_rx_buff_length(uint16_t i_len)
 {
   if (i_len > 1)
   {
     cc1310.rx.max_len_appBuff = i_len ;
     return 1;
   }
   return 0;
 }

 uint8_t sf_rf_set_p_RX_buff(uint8_t *pc_data)
 {
   if (pc_data!=NULL)
   {
     cc1310.rx.p_appBuff = pc_data;
     return 1;
   }
   return 0;
 }

 uint16_t sf_rf_read_frame()
{
	 volatile rfc_dataEntryGeneral_t *entry = cc1310.rx.p_currentDataEntry;
	 uint8_t finish = 0;
	 while(!finish && entry->status != DATA_ENTRY_FINISHED)
	 {
		 entry = (rfc_dataEntryGeneral_t *)entry->pNextEntry;
		 if(entry == (rfc_dataEntryGeneral_t *)cc1310.rx.p_currentDataEntry)
			 finish=1;
	 }
	 if(entry->status == DATA_ENTRY_FINISHED) {
	    /* when we receive data */
	#if NETSTK_CFG_IEEE_802154G_EN
		uint8_t *p_data = (uint8_t*)(&cc1310.rx.p_currentDataEntry->data);
		uint8_t tempData;

	    /* FIXME flip PHR back */
		tempData = p_data[0];
		p_data[0] = p_data[1];
		p_data[1] = tempData;

	    cc1310.rx.LenLastPkt = phy_framer802154_getPktLen(p_data, PHY_HEADER_LEN) + PHY_HEADER_LEN;
	    cc1310.rx.p_lastPkt = p_data;
	#else
	     /* FIXME this is not correct if the packet is lager 127 */
	    cc1310.rx.LenLastPkt = *(uint8_t*)(&entry->data) + PHY_HEADER_LEN;
	    cc1310.rx.p_lastPkt = (uint8_t*)(&entry->data);
	#endif


	    if(cc1310.rx.LenLastPkt > 0) {
	      if(cc1310.rx.LenLastPkt <= cc1310.rx.max_len_appBuff + PHY_HEADER_LEN ) {
	        memcpy(cc1310.rx.p_appBuff, cc1310.rx.p_lastPkt + PHY_HEADER_LEN , cc1310.rx.LenLastPkt - PHY_HEADER_LEN );
	      }

		   /* Get RSSI value from the queue : The RSSI is given on signed form in dBm.
		    * If no RSSI is available, this is signaled with a special value of the RSSI (-128, or 0x80) */
		  cc1310.rx.c_rssiValue=(uint8_t) cc1310.rx.p_lastPkt[cc1310.rx.LenLastPkt];
	      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, (int8_t)cc1310.rx.p_lastPkt[cc1310.rx.LenLastPkt]);
	      packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, 0x7F);
	    }

	    entry->status=DATA_ENTRY_PENDING;
	   /* switch to the next entry */
	   RFQueue_nextEntry();
	   /* Get the current entry and it to pending state */
	   cc1310.rx.p_currentDataEntry = (rfc_dataEntryGeneral_t*)RFQueue_getDataEntry();
	   cc1310.rx.p_currentDataEntry->status=DATA_ENTRY_PENDING;
	   return cc1310.rx.LenLastPkt - 2 - PHY_HEADER_LEN ; /* fixme: 2 bytes crc */
	  }
	  return 0;
}

uint8_t sf_rf_6lowpan_sendBlocking(uint8_t *pc_data, uint16_t  i_len)
{
    uint8_t status;
  /* check if transceiver is not is sleepy mode */
  if( sf_rf_get_Status() == RF_STATUS_SLEEP )
    return 0;
  /* init tx cmd */
  if (!sf_rf_init_tx(pc_data,i_len))
  {
    return 0;
  }
  /* call the routine with tx state to send the packet*/
  status = sf_rf_switchState(RF_STATUS_TX);
  if(status != ROUTINE_DONE && status != ROUTINE_ERROR_TX_NOACK )
  {
      return 0;
  }
  else
  {
#if CC13XX_TX_LED_ENABLED
         bsp_led( HAL_LED2, EN_BSP_LED_OP_BLINK );
#endif /* #if CC13XX_TX_LED_ENABLED */

        /* Set the transceiver in rx mode */
        sf_rf_switchState(RF_STATUS_RX_LISTEN);

        if(status== ROUTINE_DONE)
            return 1;
        else
            return 2;
  }
}


void sf_rf_6lowpan_sleep(void)
{
    /* check if the radio is already in sleep mode */
    if( sf_rf_get_Status() == RF_STATUS_SLEEP )
        return;
    /* Turn OFF radio core and disable interrupt */
    sf_rf_switchState(RF_STATUS_SLEEP);
}


bool sf_rf_wake(void) //  called by the radio : cc13xx_On (e_nsErr_t *p_err)
{
  /* Init the driver */
  if(sf_rf_switchState(RF_STATUS_INIT)!= ROUTINE_DONE)
        return false;
  /* Turn radio to RX mode */
  if(sf_rf_switchState(RF_STATUS_RX_LISTEN)!= ROUTINE_DONE)
      return false;

  return true;
}


uint8_t sf_rf_6lowpan_getTxPower(void)
{
    return cc1310.tx.signalStrength;
}


/* TX Power dBm lookup table - values from SmartRF Studio.
 * copied from the file:
 * C:\ti\tirtos_cc13xx_cc26xx_2_15_00_17\packages\examples\source\rf\easylink\EasyLink.c */
typedef struct S_OUTPUT_CONFIG {
  int8_t sc_dbm;
  uint16_t i_txPower; /* Value for the PROP_DIV_RADIO_SETUP.txPower field */
} s_outputConfig_t;

/* TX Power dBm lookup table - values from SmartRF Studio.
 * copied from the file:
 * C:\ti\tirtos_cc13xx_cc26xx_2_15_00_17\packages\examples\source\rf\easylink\EasyLink.c */
#define RF_OUTPUT_POWER_TBL_SIZE   16U
static const s_outputConfig_t s_outputPower[RF_OUTPUT_POWER_TBL_SIZE] = {
    {  0, 0x0041 },
    {  1, 0x10c3 },
    {  2, 0x1042 },
    {  3, 0x14c4 },
    {  4, 0x18c5 },
    {  5, 0x18c6 },
    {  6, 0x1cc7 },
    {  7, 0x20c9 },
    {  8, 0x24cb },
    {  9, 0x2ccd },
    { 10, 0x38d3 },
    { 11, 0x50da },
    { 12, 0xb818 },
    { 13, 0xa73f }, /* 12.5 */
    { 14, 0xa73f },
    {-10, 0x08c0 },
};


bool sf_rf_6lowpan_setTxPower(uint8_t c_signal)
{
    bool b_return = true;

#if USE_TI_RTOS
#if 0
    /** Variable to store the PA power during shutdown (only register lost) */
    static uint8_t gc_txPowerIdx;

    rfc_CMD_SET_TX_POWER_t s_cmdSetPower = {0};
    rfc_CMD_SCH_IMM_t s_immOpCmd = {0};
    rfc_CMD_PROP_RADIO_DIV_SETUP_t *ps_divSetup;
    RF_CmdHandle s_cmd;
    RF_EventMask e_result;

    s_cmdSetPower.commandNo = CMD_SET_TX_POWER;
    s_immOpCmd.commandNo = CMD_SCH_IMM;
    s_immOpCmd.startTrigger.triggerType = TRIG_NOW;
    s_immOpCmd.startTrigger.pastTrig = 1;
    s_immOpCmd.startTime = 0;

    /* Convert the input parameter to a index*/
    if(c_signal < 130U)
    {
      gc_txPowerIdx = 15;
    }
    else if (c_signal > 144)
    {
      gc_txPowerIdx = 14;
    }
    else
    {
      gc_txPowerIdx = c_signal-130;
    }

    /* If 14dBm power is requested then the CCFG_FORCE_VDDR_HH must be set in
     * the ccfg */
  #if (CCFG_FORCE_VDDR_HH != 0x1)
    if(gc_txPowerIdx == 14)
    {
      /* Decrement to a supported value */
      gc_txPowerIdx--;
    }
  #endif

    ps_divSetup = (rfc_CMD_PROP_RADIO_DIV_SETUP_t*)cc1310.conf.ps_cmdPropRadioDivSetup;
    /* Set the correct TX power */
    ps_divSetup->txPower = s_outputPower[gc_txPowerIdx].i_txPower;

    if(sf_rf_get_Status() != RF_STATUS_SLEEP)
    {
      /* CMD_SET_TX_POWER txPower is currently a bit filed in a struct, but will
       * change to a uint16 in future releases. Hence do a memcpy to cater for
       * both */
      memcpy(&(s_cmdSetPower.txPower), &(s_outputPower[gc_txPowerIdx].i_txPower),
              sizeof(uint16_t));

      /* point the Operational Command to the immediate set power command */
      s_immOpCmd.cmdrVal = (uint32_t) &s_cmdSetPower;

      if(gps_rfHandle != NULL)
      {
        /* Send command */
        s_cmd = RF_postCmd(gps_rfHandle,(RF_Op*)&s_immOpCmd, RF_PriorityNormal, NULL, 0);
        e_result = RF_pendCmd(gps_rfHandle, s_cmd, (RF_EventLastCmdDone |RF_EventCmdError));
        if ((e_result & RF_EventLastCmdDone) == false)
        {
          b_return = false;
        }/* if */
      }/* if */
    }/* if */
#endif
#else
    /** Command for setting the tx power */
    rfc_CMD_SET_TX_POWER_t s_cmdTxPower;
    s_cmdTxPower.commandNo = CMD_SET_TX_POWER;


    /* check if the radio is in sleepy mode */
    if( sf_rf_get_Status() == RF_STATUS_SLEEP)
        return false;
    /* Values from smart rf studio v2.1.0 */
    /* Set the minimum */
    if(c_signal <= 124U) /* -10dbm */
    {
      s_cmdTxPower.txPower.IB =        0x00U;
      s_cmdTxPower.txPower.GC =        0x03U;
      s_cmdTxPower.txPower.boost =     0x00U;
      s_cmdTxPower.txPower.tempCoeff = 0x00U;
      cc1310.tx.signalStrength = 120U;
    }
    /* Set the maximum */
    else if(c_signal >= 144) /*  14dbm */
    {
      s_cmdTxPower.txPower.IB =        0x3FU;
      s_cmdTxPower.txPower.GC =        0x00U;
      s_cmdTxPower.txPower.boost =     0x01U;
      s_cmdTxPower.txPower.tempCoeff = 0x00U;
      cc1310.tx.signalStrength = 144U;
    }
    /* Calculate the register settings */
    else
    {
      /* Check which setting should be used. */
      switch(c_signal)
      {
      case 143U:                                                     /* 13dBm */
        s_cmdTxPower.txPower.IB =        0x3FU;
        s_cmdTxPower.txPower.GC =        0x00U;
        s_cmdTxPower.txPower.boost =     0x00U;
        s_cmdTxPower.txPower.tempCoeff = 0x00U;
        break;

      case 142U:                                                     /* 12dBm */
        s_cmdTxPower.txPower.IB =        0x14U;
        s_cmdTxPower.txPower.GC =        0x00U;
        s_cmdTxPower.txPower.boost =     0x00U;
        s_cmdTxPower.txPower.tempCoeff = 0x00U;
        break;

      case 141U: case 140U: case 139U: case 138U:                    /* 10dBm */
        s_cmdTxPower.txPower.IB =        0x11U;
        s_cmdTxPower.txPower.GC =        0x03U;
        s_cmdTxPower.txPower.boost =     0x00U;
        s_cmdTxPower.txPower.tempCoeff = 0x00U;
        break;

      case 137U: case 136U: case 135U: case 134U: case 133U:         /*  5dBm */
        s_cmdTxPower.txPower.IB =        0x05U;
        s_cmdTxPower.txPower.GC =        0x03U;
        s_cmdTxPower.txPower.boost =     0x00U;
        s_cmdTxPower.txPower.tempCoeff = 0x00U;
        break;

      case 132U: case 131U: case 130U: case 129U: case 128U: case 127U:
      case 126U: case 125U:                                          /*  0dBm */
        s_cmdTxPower.txPower.IB =        0x02U;
        s_cmdTxPower.txPower.GC =        0x03U;
        s_cmdTxPower.txPower.boost =     0x00U;
        s_cmdTxPower.txPower.tempCoeff = 0x00U;
        break;
      }
      cc1310.tx.signalStrength = c_signal;
    }

      /* Transmit the tx power setting to the rf-core */
      MB_SendCommand((uint32_t) &s_cmdTxPower);
#endif
    return b_return;
}


uint8_t sf_rf_6lowpan_getRssi(void)
{
  return sf_rf_getRssi();
}

uint32_t sf_rf_6lowpan_getTimeStamp(void)
{
	return cc1310.rx.timeStamp;
}

E_RF_CCA_RESULT_t sf_rf_6lowpan_cca(uint8_t c_numOfRssiMeas)
{
  E_RF_CCA_RESULT_t e_return;
  uint8_t returnValue;

  if( sf_rf_get_Status() == RF_STATUS_SLEEP )
      return E_RF_CCA_RESULT_INVALID_RF_STATE;
  /* set number of RSSI measurement */
  sf_rf_set_numOfRssiMeas(c_numOfRssiMeas);
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
  /* verify if the channel is supported */
  if(chan_num > 33)
      return false;
  /* check the operation mode */
  switch(OpMode)
  {
  case 1 :
    Cent_freq=0x035F;
    Frac_Freq=0x2000;
    Delta=0x3333;
    /* calculate the frequency parameters for desired channel */
    Cent_freq = Cent_freq + (chan_num / 5);
    Frac_Freq = Frac_Freq + (chan_num % 5) * Delta;
    break;
  default:
    return false;
  }
  /* check if the radio is in sleepy mode */
  if( (sf_rf_get_Status()== RF_STATUS_SLEEP))
  {
    /* update frequency without running command: it will be executed when the radio wake up */
    if(sf_rf_update_frequency(Cent_freq, Frac_Freq))
      return true;
    else
      return false;
  }
  /* check if the radio is in RX mode */
  if( (sf_rf_get_Status()== RF_STATUS_RX_LISTEN) )
  {
      /* update frequency */
      if(sf_rf_update_run_frequency(Cent_freq, Frac_Freq))
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



/*==============================================================================
                             FUNCTION
==============================================================================*/

 /* TI RTOS variables */
 #if USE_TI_RTOS
 /* TI-RTOS RF Mode Object */
 RF_Mode RF_prop =
 {
     .rfMode = RF_MODE_PROPRIETARY_SUB_1,
     .cpePatchFxn = &rf_patch_cpe_genfsk,
     .mcePatchFxn = 0,
     .rfePatchFxn = &rf_patch_rfe_genfsk,
 };

#define RF_RTOS_EVENT_MASK    ( RF_EventLastCmdDone | RF_EventRxOk | \
		RF_EventRxNOk | \
		RF_EventInternalError | \
		RF_EventRxBufFull | RF_EventRxIgnored | RF_EventRxAborted )


static uint8_t flag_cca = 0;

static void cca_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
	flag_cca = 1 ;
}

static void Rtos_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxOk)
    {
      rx_call_back(IRQ_RX_OK );
    }
    else if (e & RF_EventRxNOk)
	{
      rx_call_back(IRQ_RX_NOK );
    }
    else if (e & RF_EventInternalError)
    {
      rx_call_back(IRQ_INTERNAL_ERROR);
    }
    else if (e & (RF_EventRxBufFull | RF_EventRxIgnored | RF_EventRxAborted))
   {
      rx_call_back(IRQ_RX_BUF_FULL);
   }
}
#endif /* USE_TI_RTOS */


static bool rf_driver_init(void)
{
#if USE_TI_RTOS

  RF_Params_init(&gs_rfParams);
  if(gps_rfHandle == NULL)
    gps_rfHandle = RF_open(&gs_rfObject, &RF_prop, (RF_RadioSetup*) &RF_802_15_4_cmdPropRadioDivSetup , &gs_rfParams);
  if(gps_rfHandle != NULL)
    return true;
  else
	return false;

#else
  rfc_returnValue_t returnValue;

  returnValue = RFC_selectRadioMode(RFC_GFSK);
  if(returnValue == RFC_OK)
  {
    /* 1.2. Enable RF Core */
    returnValue = RFC_enableRadio();
    if(returnValue == RFC_OK)
    {
      /* 1.3. Switch to HF XTAL. Needs to be done before running the RFC setup. */
      loc_switchToHfXtalOsc();
      return true;
    }
  }
  return false;
#endif
}

#if !USE_TI_RTOS
#define IRQN_HW_SYN_WORD           5
#define IRQ_HW_SYN_WORD           (1U << IRQN_HW_SYN_WORD)
uint32_t temp_timestamp;
void RFC_hw_Isr(void)
{
  /* Read interrupt flags */
  uint32_t interruptFlags = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG);

  /*  Clear interrupt */
  do{
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = ~interruptFlags;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) & interruptFlags);

  /* check the flag interrupt */
  if(interruptFlags & IRQ_HW_SYN_WORD)
  {
	  cc1310.rx.is_receiving = 1;
	  temp_timestamp = (uint32_t) RTIMER_NOW() - (uint32_t) bsp_us_to_rtimerTiscks(1500);
  }


}
#endif

static void rx_call_back(uint32_t l_flag)
{
  /* reset the rx flag */
  cc1310.rx.is_receiving = 0;

  if(IRQ_RX_OK == (l_flag & IRQ_RX_OK))
  {
#if !USE_TI_RTOS
	  cc1310.rx.timeStamp = temp_timestamp;
#endif
#if CC13XX_RX_LED_ENABLED
         bsp_led( HAL_LED3, EN_BSP_LED_OP_BLINK );
#endif /* #if CC13XX_RX_LED_ENABLED */

      if (cc1310.tx.Tx_waiting_Ack)
      {
          cc1310.rx.p_lastPkt = (uint8_t*)(&cc1310.rx.p_currentDataEntry->data) ;
          if ((cc1310.rx.p_lastPkt[PHY_HEADER_LEN] == 0x02) &&
                  (cc1310.rx.p_lastPkt[PHY_HEADER_LEN + 2] == cc1310.tx.expSeqNo))
          {
              cc1310.tx.Ack_received=1;
          }
          else
          {
              cc1310.tx.Ack_received=0;
          }
          cc1310.rx.p_currentDataEntry->status=DATA_ENTRY_PENDING;
      }
      else
      {
        if(!cc1310.poll_mode)
        {
        /* signal complete reception interrupt */
        en_evprocResCode_t err = RF_SEM_POST(EVENT_TYPE_RF);
        /* handle the received packet */
        sf_rf_switchState(RF_STATUS_RX);
        }
      }
      /* set RX mode again */
      sf_rf_switchState(RF_STATUS_RX_LISTEN);
  }
  else if(IRQ_RX_NOK == (l_flag & IRQ_RX_NOK))
  {
    /* set RX mode again */
    sf_rf_switchState(RF_STATUS_RX_LISTEN);
  }
  else if(IRQ_INTERNAL_ERROR == (l_flag & IRQ_INTERNAL_ERROR))
  {
      /* Initialize the driver again */
      if(sf_rf_switchState(RF_STATUS_INIT) != ROUTINE_DONE )
          sf_rf_switchState(RF_STATUS_ERROR);
      /* Send RX command to the radio */
      if(sf_rf_switchState(RF_STATUS_RX_LISTEN)!= ROUTINE_DONE)
          sf_rf_switchState(RF_STATUS_ERROR);
  }
  else if((IRQ_RX_BUF_FULL == (l_flag & IRQ_RX_BUF_FULL)) || (IRQ_RX_IGNORED == (l_flag & IRQ_RX_IGNORED))
          || (IRQ_RX_ABORTED == (l_flag & IRQ_RX_ABORTED)) )
  {
    /* set RX mode again */
    sf_rf_switchState(RF_STATUS_RX_LISTEN);
  }
}


static uint8_t sf_rf_switchState(e_rf_status_t state)
{
#if USE_TI_RTOS
	RF_EventMask result ;
#else
  /* Return value of lib functions */
  static rfc_returnValue_t returnValue;
  /* State of the receiver */
  static rfc_cmdStatus_t cmdStatus;
#endif
  volatile uint16_t i = 0x00U;
  e_nsErr_t err;


  /* check wheter we have to update the state or not */
  if(state!=RF_STATUS_NONE)
    cc1310.state=state;

  switch(cc1310.state)
  {
  case RF_STATUS_INIT :
    if(rf_driver_init())
    {
      /* initiate command pointer */
      cc1310.conf.ps_cmdPropRadioDivSetup=(rfc_radioOp_t*)&RF_802_15_4_cmdPropRadioDivSetup;
      cc1310.conf.ps_cmdFs=(rfc_radioOp_t*)&RF_802_15_4_cmdFs;
      cc1310.rx.p_cmdPropRxAdv=&RF_802_15_4_cmdPropRxAdv;
      cc1310.tx.p_cmdPropTxAdv=&RF_802_15_4_cmdPropTxAdv;

      /*initiate CCA command */
      sf_rf_set_numOfRssiMeas(CC13xx_NUM_OFF_RSSI_CHECKS_DEFAULT);
      sf_rf_init_cca_cmd();

#if USE_TI_RTOS
      /* Set the frequency */
      gps_rf_cmdHandle = RF_postCmd(gps_rfHandle, (RF_Op*)cc1310.conf.ps_cmdFs, RF_PriorityNormal, NULL, 0);
      if(gps_rf_cmdHandle < 0 )
       {
          return ROUTINE_ERROR_FS_CMD ;
       }
#else
      /* disable  interrupt  */
      RFC_registerCpe0Isr(NULL);
      RFC_registerCpe1Isr(NULL);
      RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));

      /* enable radio again */
      if (RFC_enableRadio() != RFC_OK)
      {
        return ROUTINE_ERROR_ENABLE_RADIO;
      }

      returnValue = RFC_setupRadio(cc1310.conf.ps_cmdPropRadioDivSetup);
      if(returnValue == RFC_OK)
      {
        /* Send CMD_FS command to RF Core */
        for(i = 0; i<0x7FFF;i++);
        cmdStatus = RFC_sendRadioOp(cc1310.conf.ps_cmdFs);
        if(cmdStatus != RFC_CMDSTATUS_DONE)
        {
          return ROUTINE_ERROR_FS_CMD ;
        }
      }
      else
          return ROUTINE_ERROR_SETUP_RADIO;

      /* Initialize interrupts */
      RFC_registerCpe0Isr(rx_call_back);
      RFC_enableCpe0Interrupt(IRQ_RX_OK);
      RFC_enableCpe0Interrupt(IRQ_RX_NOK);
      RFC_enableCpe0Interrupt(IRQ_RX_ABORTED);
      RFC_enableCpe0Interrupt(IRQ_INTERNAL_ERROR);
      RFC_enableCpe0Interrupt(IRQ_RX_BUF_FULL);
      RFC_enableCpe0Interrupt(IRQ_RX_IGNORED);

      /* initialize HW int */
      IntRegister(INT_RFC_HW_COMB, &RFC_hw_Isr);
      IntEnable(INT_RFC_HW_COMB);
      MB_EnableHWInts(IRQ_HW_SYN_WORD);

#endif

      /* initialize RX queue */
      if(RFQueue_defineQueue(&cc1310.rx.dataQueue, RxBuff, RX_BUFF_SIZE , 2 , 150 + 3 ) != 0x00U)
      {
        /* Failed to allocate space for all data entries */
        return ROUTINE_ERROR_INIT_QUEUE ;
      }

      /* reset the rx flag */
      cc1310.rx.is_receiving = 0;
      /* reset the waiting for ACK flag */
      cc1310.tx.Tx_waiting_Ack=0;
      /* Initialize global variable. Set the lowest RSSI value as default -255dBm */
      cc1310.rx.c_rssiValue=0xFFU;
      /* set the default TX power */
      cc1310.tx.signalStrength=RF_DEFAULT_TX_POWER;
      return ROUTINE_DONE ;
    }
    else
    {
      return ROUTINE_ERROR_INIT_RF ;
    }
  case RF_STATUS_IDLE :
    break;
  case RF_STATUS_SLEEP :
#if USE_TI_RTOS
	  return ROUTINE_DONE;
#else
      /* disable  interrupt  */
      RFC_registerCpe0Isr(NULL);

      /* Disable radio core */
      if (RFC_disableRadio() != RFC_OK)
      {
          return ROUTINE_ERROR_DISABLE_RADIO;
      }
      return ROUTINE_DONE;
#endif
  case RF_STATUS_TX :
	  /* reset the rx flag */
	  cc1310.rx.is_receiving = 0;
      /* find out if ACK is required */
      cc1310.tx.Tx_waiting_Ack = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);
      if(cc1310.tx.Tx_waiting_Ack)
      {
      cc1310.tx.Ack_received=0;
      cc1310.tx.expSeqNo = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);

      /* Get the ACK wait from buffer */
      cc1310.tx.waitForAckTimeout = (uint16_t) packetbuf_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION);
      }
#if USE_TI_RTOS
      /* Stop last cmd (usually it is Rx cmd ) */
      RF_flushCmd(gps_rfHandle, gps_rf_cmdHandle, 1);
      /* Send tx command  */
      result = RF_runCmd(gps_rfHandle, (RF_Op*)cc1310.tx.p_cmdPropTxAdv, RF_PriorityNormal, NULL, 0);
      if (!(result & RF_EventLastCmdDone))
      {
        return ROUTINE_ERROR_TX_CMD;
      }
      else
      {
        /* if the packet doesn't require ACK then we finish with success once the packet is send */
         if(cc1310.tx.Tx_waiting_Ack==0)
           return ROUTINE_DONE ;

         /* Stop last cmd (usually it is Rx cmd ) */
        // RF_cancelCmd(gps_rfHandle, gps_rf_cmdHandle, 1);

         cc1310.rx.p_currentDataEntry=(rfc_dataEntryGeneral_t*)cc1310.rx.dataQueue.pCurrEntry;
         cc1310.rx.p_currentDataEntry->status=DATA_ENTRY_PENDING;

         /* Post CMD_PROP_RX command to RF Core (non-blocking function )*/
         gps_rf_cmdHandle = RF_postCmd(gps_rfHandle, (RF_Op*)cc1310.rx.p_cmdPropRxAdv, RF_PriorityHighest, &Rtos_callback, RF_RTOS_EVENT_MASK);

         /* FIXME wait for ACK */
         //bsp_delay_us(cc1310.tx.waitForAckTimeout);
         for(i; i<0x2FFD;i++);
         /* stop waiting for ACK */
         cc1310.tx.Tx_waiting_Ack=0;
         if(cc1310.tx.Ack_received)
           return ROUTINE_DONE;
         else
           return ROUTINE_ERROR_TX_NOACK;
      }
#else
      /* Stop last cmd (usually it is Rx cmd ) */
      RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
      /* Send tx command  */
      cmdStatus = RFC_sendRadioOp((rfc_radioOp_t*)cc1310.tx.p_cmdPropTxAdv);

      /* Since we sent a blocking cmd then we check the state */
      if(cmdStatus != RFC_CMDSTATUS_DONE)
      {
        return ROUTINE_ERROR_TX_CMD ;
      }
      else
      {
          /* if the packet doesn't require ACK then we finish with success once the packet is send */
          if(cc1310.tx.Tx_waiting_Ack==0)
              return ROUTINE_DONE ;
          /* Send CMD_PROP_RX command to RF Core */
          RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
          cc1310.rx.p_currentDataEntry=(rfc_dataEntryGeneral_t*)cc1310.rx.dataQueue.pCurrEntry;
          cc1310.rx.p_currentDataEntry->status=DATA_ENTRY_PENDING;
          /* Send Rx command to receive the ACK */
          cmdStatus=RFC_sendRadioOp_nb((rfc_radioOp_t*)cc1310.rx.p_cmdPropRxAdv , NULL);
          /* FIXME wait for ACK */
          //bsp_delay_us(cc1310.tx.waitForAckTimeout);
          for(i= 0; i<0x4FFD;i++)
          {
            if( cc1310.tx.Ack_received )
              break;
          }
          /* stop waiting for ACK */
          cc1310.tx.Tx_waiting_Ack=0;
          if(cc1310.tx.Ack_received)
              return ROUTINE_DONE;
          else
              return ROUTINE_ERROR_TX_NOACK;
      }
#endif
  case RF_STATUS_RX_LISTEN :
    cc1310.rx.p_currentDataEntry=(rfc_dataEntryGeneral_t*)cc1310.rx.dataQueue.pCurrEntry;
    cc1310.rx.p_currentDataEntry->status=DATA_ENTRY_PENDING;
    /* Set the Data Entity queue for received data */
    cc1310.rx.p_cmdPropRxAdv->pQueue = &cc1310.rx.dataQueue;
    /* Set rx statistics output struct */
    cc1310.rx.p_cmdPropRxAdv->pOutput= (uint8_t*)&cc1310.rx.rxStatistics;

#if USE_TI_RTOS
    /* Stop last cmd (usually it is Rx cmd ) */
    RF_flushCmd(gps_rfHandle, gps_rf_cmdHandle, 1);
    /* Post CMD_PROP_RX command to RF Core (non-blocking function )*/
    gps_rf_cmdHandle = RF_postCmd(gps_rfHandle, (RF_Op*)cc1310.rx.p_cmdPropRxAdv, RF_PriorityNormal, &Rtos_callback, RF_RTOS_EVENT_MASK);
    /*  check the return value */
    if(gps_rf_cmdHandle < 0 )
    {
      return ROUTINE_ERROR_RX_CMD ;
    }
    return ROUTINE_DONE ;
#else
    /* Stop last command */
    RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
    /* Send CMD_PROP_RX command to RF Core */
    cmdStatus=RFC_sendRadioOp_nb((rfc_radioOp_t*)cc1310.rx.p_cmdPropRxAdv , NULL);
    if(cmdStatus != RFC_CMDSTATUS_DONE)
    {
      return ROUTINE_ERROR_RX_CMD ;
    }
    return ROUTINE_DONE ;
#endif

  case RF_STATUS_RX :
  {
    /* when we receive data */
#if NETSTK_CFG_IEEE_802154G_EN
	uint8_t *p_data = (uint8_t*)(&cc1310.rx.p_currentDataEntry->data);
	uint8_t tempData;

    /* FIXME flip PHR back */
	tempData = p_data[0];
	p_data[0] = p_data[1];
	p_data[1] = tempData;

    cc1310.rx.LenLastPkt = phy_framer802154_getPktLen(p_data, PHY_HEADER_LEN) + PHY_HEADER_LEN;
    cc1310.rx.p_lastPkt = p_data;
#else
     /* FIXME this is not correct if the packet is lager 127 */
    cc1310.rx.LenLastPkt = *(uint8_t*)(&cc1310.rx.p_currentDataEntry->data) + PHY_HEADER_LEN;
    cc1310.rx.p_lastPkt = (uint8_t*)(&cc1310.rx.p_currentDataEntry->data);
#endif
    /* parse the received packet to check whether it requires ACK or not */
    framer802154ll_parse(&frame, cc1310.rx.p_lastPkt , cc1310.rx.LenLastPkt );

   if(frame.is_ack_required)
   {
    /* create the ACK  */
    ack_length=framer802154ll_createAck(&frame,ack, sizeof(ack));
#if !NETSTK_CFG_IEEE_802154G_EN
    /* configure ACK length and ptr */
    cc1310.tx.p_cmdPropTxAdv->pktLen = ack_length - frame.crc_len;
    cc1310.tx.p_cmdPropTxAdv->pPkt = ack;
#else
    /* configure ACK length and ptr */
    cc1310.tx.p_cmdPropTxAdv->pPkt = ack;
    cc1310.tx.p_cmdPropTxAdv->pktLen = ack_length - frame.crc_len  ;
    /* FIXME overwrite PHR */
    uint16_t transmitLen = cc1310.tx.p_cmdPropTxAdv->pktLen - PHY_HEADER_LEN;
    uint16_t totalLen = transmitLen + frame.crc_len;
    /* FIXME only support CRC-32 */
	cc1310.tx.p_cmdPropTxAdv->pPkt[0] = totalLen & 0xFF;
	/* check whether the 32-bits and 16-bits CRC is used // 0x10: 16-bits CRC and 0x00 : 32-bits CRC */
	if(frame.crc_len == 2)
      cc1310.tx.p_cmdPropTxAdv->pPkt[1] = (totalLen >> 8) + 0x10  ;
	else
	  cc1310.tx.p_cmdPropTxAdv->pPkt[1] = (totalLen >> 8) + 0x00  ;
#endif
#if USE_TI_RTOS

    /* Send tx command  */
    RF_postCmd(gps_rfHandle, (RF_Op*)cc1310.tx.p_cmdPropTxAdv, RF_PriorityHighest, NULL , 0 );
    /* wait the command until it finish */
    for(i; i<0x0FFD;i++);

#else
    /* Send Tx command to send the ACK */
    RFC_sendRadioOp_nb((rfc_radioOp_t*)cc1310.tx.p_cmdPropTxAdv,NULL);
    /* wait for the command until finish */
    uint16_t wdog = 0xFFFF;
    while(cc1310.tx.p_cmdPropTxAdv->status != PROP_DONE_OK && wdog)
    {
      wdog--;
    }
#endif
    }
   /* Get RSSI value from the queue : The RSSI is given on signed form in dBm.
    * If no RSSI is available, this is signaled with a special value of the RSSI (-128, or 0x80) */
   cc1310.rx.c_rssiValue=(uint8_t) cc1310.rx.p_lastPkt[cc1310.rx.LenLastPkt];
   /* switch to the next entry */
   RFQueue_nextEntry();
   /* Get the current entry and it to pending state */
   cc1310.rx.p_currentDataEntry = (rfc_dataEntryGeneral_t*)RFQueue_getDataEntry();
   cc1310.rx.p_currentDataEntry->status=DATA_ENTRY_PENDING;
    return ROUTINE_DONE ;
  }
  case RF_STATUS_CCA :
#if USE_TI_RTOS
	    /* Stop last cmd (usually it is Rx cmd ) */
	    RF_flushCmd(gps_rfHandle, gps_rf_cmdHandle, 1);
	    /* update command */
	       cc1310.cca.s_cmdCS.numRssiBusy =cc1310.cca.c_numOfRssiMeas;
	       cc1310.cca.s_cmdCS.numRssiIdle =cc1310.cca.c_numOfRssiMeas;
	    /* Start carrier sense */
	    gps_rf_cmdHandle = RF_postCmd(gps_rfHandle, (RF_Op*)&cc1310.cca.s_cmdCS, RF_PriorityHighest, &cca_callback , (RF_EventLastCmdDone | RF_EventCmdError) );
	    /* TODO add watch dog */
	    while(!flag_cca);
	    flag_cca = 0 ;
        /* Check the status of the CS command */
        if(cc1310.cca.s_cmdCS.status == PROP_DONE_IDLE)
        {
          return ROUTINE_CCA_RESULT_IDLE;
        }
        else
          return ROUTINE_CCA_RESULT_BUSY;
#else
    /* Stop last cmd (usually it is Rx cmd ) */
    RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
    /* update command */
    cc1310.cca.s_cmdCS.numRssiBusy =cc1310.cca.c_numOfRssiMeas;
    cc1310.cca.s_cmdCS.numRssiIdle =cc1310.cca.c_numOfRssiMeas;
    /* Start carrier sense */
    cmdStatus = RFC_sendRadioOp((rfc_radioOp_t*)&cc1310.cca.s_cmdCS);
    /* Check if the command was processed correctly */
    if(cmdStatus == RFC_CMDSTATUS_DONE)
    {
      /* Check the status of the CS command */
      if(cc1310.cca.s_cmdCS.status == PROP_DONE_IDLE)
      {
        return ROUTINE_CCA_RESULT_IDLE;
      }
      else
        return ROUTINE_CCA_RESULT_BUSY;
    }
    return ROUTINE_CCA_ERROR_STATE;
#endif

  case RF_STATUS_ERROR :
      /* Call emb6 error handler */
      err = NETSTK_ERR_FATAL;
      emb6_errorHandler(&err);
      return ROUTINE_FATAL_ERROR;
  };
  return ROUTINE_DONE ;
}

static e_rf_status_t sf_rf_get_Status(void)
{
  return cc1310.state;
}

static uint8_t* sf_rf_get_p_lastPkt(void)
{
  return cc1310.rx.p_lastPkt;
}

static uint16_t sf_rf_get_LenLastPkt(void)
{
  return cc1310.rx.LenLastPkt;
}

static uint8_t sf_rf_getRssi(void)
{
  return cc1310.rx.c_rssiValue;
}

static uint8_t sf_rf_init_tx(uint8_t *pc_data, uint16_t  i_len)
{
  if (pc_data!=NULL && i_len > 1)
  {
    cc1310.tx.p_cmdPropTxAdv->pktLen = i_len ;
    cc1310.tx.p_cmdPropTxAdv->pPkt = pc_data;
#if NETSTK_CFG_IEEE_802154G_EN
    /* FIXME overwrite PHR */
    uint16_t transmitLen = i_len - PHY_HEADER_LEN;
    uint16_t totalLen = transmitLen + packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);
    /* FIXME only support CRC-32 */
	cc1310.tx.p_cmdPropTxAdv->pPkt[0] = totalLen & 0xFF;
	/* check whether the 32-bits and 16-bits CRC is used // 0x10: 16-bits CRC and 0x00 : 32-bits CRC */
	if(packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN) == 2)
      cc1310.tx.p_cmdPropTxAdv->pPkt[1] = (totalLen >> 8) + 0x10  ;
	else
	  cc1310.tx.p_cmdPropTxAdv->pPkt[1] = (totalLen >> 8) + 0x00  ;
#else
    cc1310.tx.p_cmdPropTxAdv->pPkt[0] = i_len - PHY_HEADER_LEN + 2 ;
#endif
    return 1;
  }
  return 0;
}


static bool sf_rf_set_numOfRssiMeas(uint8_t num)
{
  if( num )
    cc1310.cca.c_numOfRssiMeas=num;
  else
    return false;
  return true;
}

static void sf_rf_init_cca_cmd(void)
{
  /* Fill the carrier sense command */
  /* The command ID number 0x3805 */
  cc1310.cca.s_cmdCS.commandNo = CMD_PROP_CS;
  /* An integer telling the status of the command.
   * This value is updated by the radio CPU during operation and may be read by the system
   * CPU at any time. */
  cc1310.cca.s_cmdCS.status = 0U;
  /* Pointer to the next operation to run after this operation is done */
  cc1310.cca.s_cmdCS.pNextOp = NULL;
  /* Absolute or relative start time (depending on the value of startTrigger) */
  cc1310.cca.s_cmdCS.startTime = 0U;
  /* The type of trigger. */
  cc1310.cca.s_cmdCS.startTrigger.triggerType = TRIG_NOW;
  /* 0: No alternative trigger command */
  cc1310.cca.s_cmdCS.startTrigger.bEnaCmd = 0U;
  /* The trigger number of the CMD_TRIGGER command that triggers this action */
  cc1310.cca.s_cmdCS.startTrigger.triggerNo = 0U;
  /* 0: A trigger in the past is never triggered, or for start of commands, give an error */
  cc1310.cca.s_cmdCS.startTrigger.pastTrig = 0U;
  /* Condition for running next command: Rule for how to proceed */
  cc1310.cca.s_cmdCS.condition.rule = 1U;
  /* Number of skips if the rule involves skipping */
  cc1310.cca.s_cmdCS.condition.nSkip = 0U;
  /* 0: Keep synth running if command ends with channel Idle */
  cc1310.cca.s_cmdCS.csFsConf.bFsOffIdle = 0U;
  /* 0: Keep synth running if command ends with channel Busy */
  cc1310.cca.s_cmdCS.csFsConf.bFsOffBusy = 0U;
  /* If 1, enable RSSI as a criterion. */
  cc1310.cca.s_cmdCS.csConf.bEnaRssi = 1U;
  /* If 0, disable correlation as a criterion. */
  cc1310.cca.s_cmdCS.csConf.bEnaCorr = 0U;
  /* 0: Busy if either RSSI or correlation indicates Busy */
  cc1310.cca.s_cmdCS.csConf.operation = 0U;
  /* 1: End carrier sense on channel Busy */
  cc1310.cca.s_cmdCS.csConf.busyOp = 1U;
  /* 1: End on channel Idle */
  cc1310.cca.s_cmdCS.csConf.idleOp = 1U;
  /* 0: Timeout with channel state Invalid treated as Busy */
  cc1310.cca.s_cmdCS.csConf.timeoutRes = 0U;
  /* Set the RSSI threshold value for a busy channel */
  cc1310.cca.s_cmdCS.rssiThr = RF_CCA_RSSI_THR;
  /* Number of consecutive RSSI measurements below the threshold needed before
   * the channel is declared Idle.  */
  cc1310.cca.s_cmdCS.numRssiIdle = cc1310.cca.c_numOfRssiMeas;
  /* Number of consecutive RSSI measurements above the threshold needed before
   * the channel is declared Busy */
  cc1310.cca.s_cmdCS.numRssiBusy = cc1310.cca.c_numOfRssiMeas;
  /* The type of trigger */
  cc1310.cca.s_cmdCS.csEndTrigger.triggerType = TRIG_NEVER;
  /* 0: No alternative trigger command */
  cc1310.cca.s_cmdCS.csEndTrigger.bEnaCmd = 0U;
  /* The trigger number of the CMD_TRIGGER command that triggers this action */
  cc1310.cca.s_cmdCS.csEndTrigger.triggerNo = 0U;
  /* 0: A trigger in the past is never triggered, or for start of commands, give an error */
  cc1310.cca.s_cmdCS.csEndTrigger.pastTrig = 0U;
  /*  Time used together with csEndTrigger for ending the operation. */
  cc1310.cca.s_cmdCS.csEndTime = 0U;
}



static bool sf_rf_update_run_frequency(uint16_t frequency, uint16_t fractFreq)
{
    /* check the struct is already initialized */
    if(cc1310.conf.ps_cmdFs== NULL)
        return false;
#if USE_TI_RTOS

    /* Cancel the previous command  */
    RF_cancelCmd(gps_rfHandle, gps_rf_cmdHandle, 1);
    /* check if the center frequency is changed then we have to setup the radio again */
    if(((rfc_CMD_PROP_RADIO_DIV_SETUP_t*)cc1310.conf.ps_cmdPropRadioDivSetup)->centerFreq != frequency)
    {
        /* set the new center Frequency */
        ((rfc_CMD_PROP_RADIO_DIV_SETUP_t*)cc1310.conf.ps_cmdPropRadioDivSetup)->centerFreq=frequency;
        /* send Setup command again */
        gps_rf_cmdHandle = RF_postCmd(gps_rfHandle, (RF_Op*)cc1310.conf.ps_cmdPropRadioDivSetup, RF_PriorityNormal, NULL, 0);
        if(gps_rf_cmdHandle < 0 )
         {
            return false ;
         }
    }
    /* update FS command */
    ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->frequency=frequency;
    ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->fractFreq=fractFreq;

    gps_rf_cmdHandle = RF_postCmd(gps_rfHandle, (RF_Op*)cc1310.conf.ps_cmdFs , RF_PriorityNormal, NULL, 0);
    if(gps_rf_cmdHandle < 0 )
     {
        return false ;
     }
#else
    /* Stop the last command : it should be RX command (May be no command in sleepy mode) */
    RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
    /* check if the center frequency is changed then we have to setup the radio again */
    if(((rfc_CMD_PROP_RADIO_DIV_SETUP_t*)cc1310.conf.ps_cmdPropRadioDivSetup)->centerFreq != frequency)
    {
        /* set the new center Frequency */
        ((rfc_CMD_PROP_RADIO_DIV_SETUP_t*)cc1310.conf.ps_cmdPropRadioDivSetup)->centerFreq=frequency;
        /* send Setup command again */
        if(RFC_setupRadio(cc1310.conf.ps_cmdPropRadioDivSetup) != RFC_OK)
            return false ;
    }
    /* update FS command */
    ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->frequency=frequency;
    ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->fractFreq=fractFreq;
    /* Send CMD_FS command to RF Core */
    if(RFC_sendRadioOp(cc1310.conf.ps_cmdFs) != RFC_CMDSTATUS_DONE)
    {
      return false ;
    }
#endif
    return true;
}


static bool sf_rf_update_frequency(uint16_t frequency, uint16_t fractFreq)
{
   /* check the struct is already initialized */
  if(cc1310.conf.ps_cmdFs == NULL || cc1310.conf.ps_cmdPropRadioDivSetup == NULL)
    return false;
  /* update center Frequency */
  ((rfc_CMD_PROP_RADIO_DIV_SETUP_t*)cc1310.conf.ps_cmdPropRadioDivSetup)->centerFreq=frequency;
  /* update FS command */
  ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->frequency=frequency;
  ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->fractFreq=fractFreq;
  return true;
}

/*==============================================================================
                         LOCAL FUNCTIONS
==============================================================================*/

/*******************************************************************************
 * @brief   Switch to use HF XTAL
 *
 * @param   None
 *
 * @return  None
 *******************************************************************************/
#if !USE_TI_RTOS
static void loc_switchToHfXtalOsc(void)
{
  /* Enable OSC DIG interface to change clock sources */
  OSCInterfaceEnable();

  /* Check to not enable XOSC if already enabled */
  if(OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF)
  {
    /* Disable autostart of XOSC state machine */
    DDI32BitsSet(AUX_DDI0_OSC_BASE,
        DDI_0_OSC_O_AMPCOMPCTL,
        DDI_0_OSC_AMPCOMPCTL_AMPCOMP_SW_CTRL);

    /* Request to switch to the crystal to enable radio operation. */
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_XOSC_HF);

    /* Wait for a while */
    CPUdelay(48000000/3*0.002);

    /* Enable autostart, will start the state machine */
    DDI32BitsClear(AUX_DDI0_OSC_BASE,
        DDI_0_OSC_O_AMPCOMPCTL,
        DDI_0_OSC_AMPCOMPCTL_AMPCOMP_SW_CTRL);

    /* Switch the HF source - Done via ROM API*/
    OSCHfSourceSwitch();
  }

  /* Disable OSC DIG interface */
  OSCInterfaceDisable();
}
#endif

#ifdef __cplusplus
}
#endif
