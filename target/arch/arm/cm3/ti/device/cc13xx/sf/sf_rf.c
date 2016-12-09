
/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_rf.c
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      Implementation of general rf function for the CC13xx rf module.
*/
#ifdef __cplusplus
extern "C" {
#endif

#define __DECL_SF_RF_API_H__
#define __DECL_SF_RF_H__

#include "hwinit.h"

#ifndef USE_TI_RTOS
#error Please define if TI-RTOS is in use ore not
#endif /* USE_TI_RTOS */

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>

/**************************** CC13xx includes *********************************/
/* RF dataqueue headers */
#include "rflib/rf_cc1350.h"
#include "rflib/rf_dbell.h"
#include "rflib/rf_queue.h"
#include "driverlib/rf_mailbox.h"
#include "bsp.h"
#include <cc13xx_cfg.h>
#include "sf_rf_6lowpan.h"

#include "llframer.h"
#include "packetbuf.h"


#if USE_TI_RTOS
#include <ti/drivers/rf/RF.h>
#include "rfLib/patch/apply_wmbus_tmode_mce_patch.h"
#else
/* BoardSupportPacket */
#include "bsp/srf06eb_cc26xx/drivers/source/bsp.h"
#endif /* USE_TI_RTOS */
#include "sf_rf.h"
/*==============================================================================
                            DEFINES
==============================================================================*/
#ifndef CC13XX_TX_ENABLED
#error Please define if the device support TX
#endif /* CC13XX_TX_ENABLED */

#ifndef CC13XX_RX_ENABLED
#error Please define if the device support RTX
#endif /* CC13XX_RX_ENABLED */

#ifndef CC13XX_IS_POLLED_RADIO
#error Please define if the radio will be polled by the run function ore not
#endif /* CC13XX_IS_POLLED_RADIO */


#ifndef CC13XX_MAX_TELEGRAM_LEN
#error Please define the maximum length needed to send or receive one telegram
#else
/* For some strange reasons the rf needs 3 additional bytes to receice telegrams */
#define RF_RX_ARRAY_SIZE   (CC13XX_MAX_TELEGRAM_LEN + 3U)
#endif /* CC13XX_MAX_TELEGRAM_LEN */



#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define NUM_DATA_ENTRIES       1  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * - the length indicator (if RF_cmdPropRx.rxConf.bIncludeHdr = 0)
                                   *   - Not included
                                   * - the payload
                                   *   - Included by default, does not affect appended bytes
                                   * - CRC (if RF_cmdPropRx.rxConf.bIncludeCrc = 1)
                                   *   - NOT included
                                   * - RSSI (if RF_cmdPropRx.rxConf.bAppendRssi = 1)
                                   *   - Included
                                   * - Timestamp (if RF_cmdPropRx.rxConf.bAppendTimestamp = 1)
                                   *   - NOT included
                                   * - Status (if RF_cmdPropRx.rxConf.bAppendStatus = 1
                                   *   - Included
                                   *--------------------------------------------
                                   * Total default appended bytes: 2 */

/* Threshold value for the rx process. */
#define RF_RX_THRESHOLD        0x05U

/* Defines the default TX-power of the device: Here 0dBm is used */
#define RF_DEFAULT_TX_POWER    130U
/*==============================================================================
                            TYPEDEF ENUMS
==============================================================================*/

/*==============================================================================
                            FUNCTION PROTOTYPES
==============================================================================*/

static void loc_switchToHfXtalOsc(void);
static void sf_rf_init_cca_cmd(void);

/*==============================================================================
                            API FUNCTION
==============================================================================*/
st_cc1310_t cc1310;
#define RX_BUFF_SIZE 400
uint8_t RxBuff[RX_BUFF_SIZE];
llframe_attr_t frame;
uint8_t ack[10];
uint8_t ack_length;

static bool rf_driver_init(void)
{
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
}

static void rx_call_back(uint32_t l_flag)
{
  if(IRQ_RX_OK == (l_flag & IRQ_RX_OK))
  {
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
        /* handle the received packet */
        rf_driver_routine(RF_STATUS_RX);
        /* signal complete reception interrupt */
        RF_SEM_POST(NETSTK_RF_EVENT);
      }
      /* set RX mode again */
      rf_driver_routine(RF_STATUS_RX_LISTEN);
      bsp_led( E_BSP_LED_1, E_BSP_LED_OFF );
      bsp_led( E_BSP_LED_0, E_BSP_LED_ON );
      bsp_led( E_BSP_LED_3, E_BSP_LED_TOGGLE );
  }
  else if(IRQ_RX_NOK == (l_flag & IRQ_RX_NOK))
  {
    /* set RX mode again */
    rf_driver_routine(RF_STATUS_RX_LISTEN);
    bsp_led( E_BSP_LED_1, E_BSP_LED_ON );
    bsp_led( E_BSP_LED_0, E_BSP_LED_OFF );
    bsp_led( E_BSP_LED_3, E_BSP_LED_TOGGLE );
  }
  else if(IRQ_INTERNAL_ERROR == (l_flag & IRQ_INTERNAL_ERROR))
  {
      /* Initialize the driver again */
      if(rf_driver_routine(RF_STATUS_INIT) != ROUTINE_DONE )
          rf_driver_routine(RF_STATUS_ERROR);
      /* Send RX command to the radio */
      if(rf_driver_routine(RF_STATUS_RX_LISTEN)!= ROUTINE_DONE)
          rf_driver_routine(RF_STATUS_ERROR);
    bsp_led( E_BSP_LED_1, E_BSP_LED_ON );
    bsp_led( E_BSP_LED_0, E_BSP_LED_ON );
  }
  else if((IRQ_RX_BUF_FULL == (l_flag & IRQ_RX_BUF_FULL)) || (IRQ_RX_IGNORED == (l_flag & IRQ_RX_IGNORED))
          || (IRQ_RX_ABORTED == (l_flag & IRQ_RX_ABORTED)) )
  {
    /* set RX mode again */
    rf_driver_routine(RF_STATUS_RX_LISTEN);
    bsp_led( E_BSP_LED_1, E_BSP_LED_ON );
    bsp_led( E_BSP_LED_0, E_BSP_LED_ON );
  }
}




uint8_t rf_driver_routine(e_rf_status_t state)
{
  /* Return value of lib functions */
  static rfc_returnValue_t returnValue;
  /* State of the receiver */
  static rfc_cmdStatus_t cmdStatus;
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

      /* disable  interrupt  */
      RFC_registerCpe0Isr(NULL);
      RFC_registerCpe1Isr(NULL);
      RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));

      /* initialize RX queue */
      if(RFQueue_defineQueue(&cc1310.rx.dataQueue, RxBuff, RX_BUFF_SIZE , 2 , 150 + 3 ) != 0x00U)
      {
        /* Failed to allocate space for all data entries */
        bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
        return ROUTINE_ERROR_INIT_QUEUE ;
      }

      if (RFC_enableRadio() != RFC_OK)
      {
        bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
        return ROUTINE_ERROR_ENABLE_RADIO;
      }

      returnValue = RFC_setupRadio(cc1310.conf.ps_cmdPropRadioDivSetup);
      if(returnValue == RFC_OK)
      {
        /* Send CMD_FS command to RF Core */
        for(i; i<0x7FFF;i++);
        cmdStatus = RFC_sendRadioOp(cc1310.conf.ps_cmdFs);
        if(cmdStatus != RFC_CMDSTATUS_DONE)
        {
          bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
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

      /* reset the waiting for ACK flag */
      cc1310.tx.Tx_waiting_Ack=0;
      /* Initialize global variable. Set the lowest RSSI value as default -255dBm */
      cc1310.rx.c_rssiValue=0xFFU;
      return ROUTINE_DONE ;
    }
    else
    {
      bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
      return ROUTINE_ERROR_INIT_RF ;
    }

  case RF_STATUS_IDLE :
    break;
  case RF_STATUS_SLEEP :
      /* disable  interrupt  */
      RFC_registerCpe0Isr(NULL);
      /* Disable radio core */
      if (RFC_disableRadio() != RFC_OK)
      {
          return ROUTINE_ERROR_DISABLE_RADIO;
      }
      return ROUTINE_DONE;
  case RF_STATUS_TX :
      /* TODO add check whether the radio is on & setup correctly etc */

      /* find out if ACK is required */
      cc1310.tx.Tx_waiting_Ack = packetbuf_attr(PACKETBUF_ATTR_MAC_ACK);
      if(cc1310.tx.Tx_waiting_Ack)
      {
      cc1310.tx.Ack_received=0;
      cc1310.tx.expSeqNo = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);

      /* TODO : carry about the use of long preamble because the wait ack will be longer */
      cc1310.tx.waitForAckTimeout = (uint16_t) packetbuf_attr(PACKETBUF_ATTR_MAC_ACK_WAIT_DURATION);
      }

      /* Stop last cmd (usually it is Rx cmd ) */
      RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
      /* Send tx command  */
      cmdStatus = RFC_sendRadioOp((rfc_radioOp_t*)cc1310.tx.p_cmdPropTxAdv);
      /* Since we sent a blocking cmd then we check the state */
      if(cmdStatus != RFC_CMDSTATUS_DONE)
      {
        bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
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
          for(i; i<0x2FFD;i++); // 3 ms
          /* stop waiting for ACK */
          cc1310.tx.Tx_waiting_Ack=0;
          if(cc1310.tx.Ack_received)
              return ROUTINE_DONE ;
          else
              return ROUTINE_ERROR_TX_NOACK ;
      }

  case RF_STATUS_RX_LISTEN :
    /* TODO add check whether the radio is on & setup correctly etc */

    RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
    cc1310.rx.p_currentDataEntry=(rfc_dataEntryGeneral_t*)cc1310.rx.dataQueue.pCurrEntry;
    /* Set the Data Entity queue for received data */
    cc1310.rx.p_cmdPropRxAdv->pQueue = &cc1310.rx.dataQueue;
    /* Set rx statistics output struct */
    cc1310.rx.p_cmdPropRxAdv->pOutput= (uint8_t*)&cc1310.rx.rxStatistics;

    /* Send CMD_PROP_RX command to RF Core */
    cmdStatus=RFC_sendRadioOp_nb((rfc_radioOp_t*)cc1310.rx.p_cmdPropRxAdv , NULL);
    if(cmdStatus != RFC_CMDSTATUS_DONE)
    {
      bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
      return ROUTINE_ERROR_RX_CMD ;
    }
    return ROUTINE_DONE ;

  case RF_STATUS_RX :

    /* when we receive data */
    cc1310.rx.LenLastPkt = *(uint8_t*)(&cc1310.rx.p_currentDataEntry->data) + 1;
    cc1310.rx.p_lastPkt = (uint8_t*)(&cc1310.rx.p_currentDataEntry->data) ;

    /* parse the received packet to check whether it requires ACK or not */
    llframe_parse(&frame, cc1310.rx.p_lastPkt , cc1310.rx.LenLastPkt );

   if(frame.is_ack_required)
   {

    ack_length=llframe_createAck(&frame,ack, sizeof(ack));

    /* configure ACK length and ptr */
    cc1310.tx.p_cmdPropTxAdv->pktLen = ack_length-2  ;
    cc1310.tx.p_cmdPropTxAdv->pPkt = ack;
    /* Send Tx command to send the ACK */
    RFC_sendRadioOp_nb((rfc_radioOp_t*)cc1310.tx.p_cmdPropTxAdv,NULL);
    /* wait for the command until finish : TODO check if this is blocking in case of error */
    while(cc1310.tx.p_cmdPropTxAdv->status != PROP_DONE_OK);
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

  case RF_STATUS_CCA :

    /* TODO add check whether the radio is on & setup correctly etc */

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

  case RF_STATUS_ERROR :
      /* Call emb6 error handler */
      err = NETSTK_ERR_FATAL;
      emb6_errorHandler(&err);
      return ROUTINE_FATAL_ERROR;
  };

  return ROUTINE_DONE ;
}

e_rf_status_t sf_rf_get_Status(void)
{
  return cc1310.state;
}

uint8_t* sf_rf_get_p_lastPkt(void)
{
  return cc1310.rx.p_lastPkt;
}

uint16_t sf_rf_get_LenLastPkt(void)
{
  return cc1310.rx.LenLastPkt;
}

uint8_t sf_rf_getRssi(void)
{
  return cc1310.rx.c_rssiValue;
}

uint8_t sf_rf_init_tx(uint8_t *pc_data, uint16_t  i_len)
{
  if (pc_data!=NULL && i_len > 1)
  {
    cc1310.tx.p_cmdPropTxAdv->pktLen = i_len ;
    cc1310.tx.p_cmdPropTxAdv->pPkt = pc_data;
    cc1310.tx.p_cmdPropTxAdv->pPkt[0] = i_len - 1 + 2 ;
    return 1;
  }
  return 0;
}


bool sf_rf_set_numOfRssiMeas(uint8_t num)
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



bool sf_rf_update_frequency(uint16_t frequency, uint16_t fractFreq)
{
    /* check the struct is already initialized */
    if(cc1310.conf.ps_cmdFs== NULL)
        return false;
    ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->frequency=frequency;
    ((rfc_CMD_FS_t*)cc1310.conf.ps_cmdFs)->fractFreq=fractFreq;

    /* Stop the last command : it should be RX command (May be no command in sleepy mode) */
    RFC_sendDirectCmd(CMDR_DIR_CMD(CMD_ABORT));
    /* TODO check if we have to setup the radio again */

    /* Send CMD_FS command to RF Core */
    if(RFC_sendRadioOp(cc1310.conf.ps_cmdFs) != RFC_CMDSTATUS_DONE)
    {
      bsp_led( E_BSP_LED_3, E_BSP_LED_ON );
      return false ;
    }

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
}/* loc_switchToHfXtalOsc() */



/*============================================================================*/
/** sf_rf_setRfChannel() */
/*============================================================================*/
bool sf_rf_setRfChannel(rfc_radioOp_t* ps_cmdPropRadioDivSetup, rfc_radioOp_t* ps_cmdFs)
{
return true;
}

/*============================================================================*/
/** sf_rf_sleep() */
/*============================================================================*/
void sf_rf_sleep(void)
{
 /* TODO check the state before */

 /* Turn OFF radio core and disable interrupt */
 rf_driver_routine(RF_STATUS_SLEEP);

}

/*============================================================================*/
/** sf_rf_wake() */
/*============================================================================*/
void sf_rf_wake(void)
{
    /* TODO  again */
    rf_driver_routine(RF_STATUS_INIT);
  return;
} /* sf_rf_wake() */

/*============================================================================*/
/** sf_rf_setSignalStrength() */
/*============================================================================*/
bool sf_rf_setSignalStrength(uint8_t c_signal)
{
  bool b_return = true;
  /** Command for setting the tx power */
  rfc_CMD_SET_TX_POWER_t s_cmdTxPower;
  s_cmdTxPower.commandNo = CMD_SET_TX_POWER;

  /* Values from smart rf studio v2.1.0 ***************************************/
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

    /* TODO make sure that the radio is not in sleepy mode */

    /* Transmit the tx power setting to the rf-core */
    MB_SendCommand((uint32_t) &s_cmdTxPower);
  return b_return;
}

/*============================================================================*/
/** sf_rf_getSignalStrength() */
/*============================================================================*/
uint8_t sf_rf_getSignalStrength(void)
{
  return cc1310.tx.signalStrength;
}


#ifdef __cplusplus
}
#endif
