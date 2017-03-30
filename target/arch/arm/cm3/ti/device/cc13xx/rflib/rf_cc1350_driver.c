/******************************************************************************
*  Filename:       rf_cc1350_driver.c
*  Revised:        $ $
*  Revision:       $ $
*
*  Description:    CC1350 API
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

//*****************************************************************************
//
//! \addtogroup rf_cc1350_driver_api
//! @{
//
//*****************************************************************************

/*******************************************************************************
* INCLUDES
*/
#include <inc/hw_chip_def.h>
#include <inc/hw_prcm.h>
#include <driverlib/rfc.h>

#include "rf_cc1350.h"
#include "rf_dbell.h"

#include "rf_config.h"

/* Patches */

#ifdef RFC_INCLUDE_GFSK
#endif

#ifdef RFC_INCLUDE_BLE
#endif

#ifdef RFC_INCLUDE_802_15_4
#endif

#ifdef RFC_INCLUDE_OOK
#endif

#ifdef RFC_INCLUDE_LRM
#endif

#ifdef RFC_INCLUDE_T_MODE
#include <rflib/patch/rf_patch_mce_wmbus_ctmode.h>
#endif


/*******************************************************************************
* MACROS
*/

/*******************************************************************************
* CONSTANTS
*/

/*******************************************************************************
* TYPEDEFS
*/

/*******************************************************************************
* LOCAL VARIABLES
*/

/* Internal library state */
RFC_STATIC rfc_config_t rfc_config =
{
  .rpid                 = RFC_RPID_UNDEF,
  .currentCmd           = NULL,
  .bTx                  = 3u
};


void (*rfc_nbIsr)(uint32_t);
void (*rfc_cpe0Isr)(uint32_t);
void (*rfc_cpe1Isr)(uint32_t);


/*******************************************************************************
* GLOBAL VARIABLES
*/
volatile uint8_t rfc_flag;
union rfc_cmds rfc_tmp_cmd;


/*******************************************************************************
* PROTOTYPES
*/
RFC_STATIC void RFC_cpe0Isr(void);
RFC_STATIC void RFC_cpe1Isr(void);
RFC_STATIC_INLINE void RFC_trapCmd(rfc_radioOp_t *cmd);
RFC_STATIC rfc_returnValue_t RFC_patchCpe(void);
RFC_STATIC_INLINE rfc_returnValue_t RFC_initRadio_nb(rfc_radioOp_t *cmd);

/******************************************************************************/


//*****************************************************************************
//
//! \brief Powerup of RFCORE, including CPE patch if needed and switching to XTAL
//!
//! Step 1. Check if radio mode has been configured \n
//! Step 2. Enable the RF Core Power Domain \n
//! Step 3. Enable clocks inside RF Core \n
//! Step 4. Init mailbox interface and callbacks
//! Step 5: Setup interrupts
//! Step 6: Patch CPE
//! Step 7. Start RAT timer
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
rfc_returnValue_t
RFC_enableRadio()
{  
  /* Step 1. Check if radio mode has been configured */
  if(rfc_config.rpid == NULL)
  {
    return(RFC_ERROR_RPID);
  }
  
  /* Step 2. Enable the RF Core Power Domain */
  PRCMPowerDomainOn(PRCM_DOMAIN_RFCORE);
  while(PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) != PRCM_DOMAIN_POWER_ON);
  
  /* Step 3. Enable clocks inside RF Core */
  RFCClockEnable(); 
  
  /* Step 4. Init mailbox interface and callbacks */
  MB_Init();
 
  /* Step 5: Setup interrupts */
  /* 5.1 Multiplex all RF Core interrupts to CPE0 IRQ and enable the following interrupts: 
  *   -  IRQ_LAST_COMMAND_DONE - last command in chain done */
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL) = 0;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) = IRQ_LAST_COMMAND_DONE;
  
  /* 5.2 Clear any pending interrupts, register interrupts handler and enable interrupts. */
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0x0;
  IntPendClear(INT_RFC_CPE_0 | INT_RFC_CPE_1 | INT_RFC_CMD_ACK);
  IntPendClear(INT_RFC_CPE_0 | INT_RFC_CPE_1);
  
  MB_RegisterIsrCback(RF_CPE_0_INTERRUPT, &RFC_cpe0Isr);
  MB_RegisterIsrCback(RF_CPE_1_INTERRUPT, &RFC_cpe1Isr);
  
  IntEnable(INT_RFC_CPE_0);
  
  /* Step 6: Patch CPE */
  if(RFC_patchCpe())
  {
    return(RFC_ERROR_PATCH);
  }
  
  /* Step 7. Start RAT timer */
  uint8_t res = MB_SendCommand(CMDR_DIR_CMD(CMD_START_RAT));
  if(res == CMDSTA_Done)
  {
    ; // OK or already running
  }
  else if(res == CMDSTA_ContextError)
  {
    MB_SendCommand(CMDR_DIR_CMD(CMD_ABORT));
  }
  else
  {
    return(RFC_ERROR);
  }
  
  /* Reset state */
  rfc_config.bTx = 3;
  
  return(RFC_OK);
}

//*****************************************************************************
//
//! \brief Powerdown of rfcore
//!
//! Step 1. Check if RF core already off
//! Step 2. Send ABORT and power off the synth \n
//! Step 3. Switch off the digital part
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
rfc_returnValue_t 
RFC_disableRadio()
{
  /* The proper thing to do here is to avoid switching off the digital part of 
  * Radio without switching off the analog parts first */
  
  /* Check if RF core already off */
  if(((HWREG( PRCM_BASE + PRCM_O_PDCTL0) & PRCM_PDCTL0_RFC_ON) == 0x0) ||
     ((HWREG( PRCM_BASE + PRCM_O_PDCTL1) & PRCM_PDCTL1_RFC_ON) == 0x0))
  {
    return(RFC_ERROR);
  }
  
  /* Step 2.1 Send ABORT, then wait until the command has completed */
  if(HWREG(0x210000EC) != NULL)
  {
    MB_SendCommand(CMDR_DIR_CMD(CMD_ABORT));
    
    /* Wait until command is done executing */
    while(rfc_flag)
    {
      PRCMSleep();
    }
  }
  
  /* Step 2.2 Configure powerdown command */
  // TODO. Check here if the synth needs power down or not
  rfc_tmp_cmd.cmdFsPowerDown.commandNo                = CMD_FS_POWERDOWN;
  rfc_tmp_cmd.cmdFsPowerDown.status                   = IDLE;
  rfc_tmp_cmd.cmdFsPowerDown.pNextOp                  = 0;
  rfc_tmp_cmd.cmdFsPowerDown.startTime                = 0;
  rfc_tmp_cmd.cmdFsPowerDown.startTrigger.triggerType = TRIG_NOW;
  rfc_tmp_cmd.cmdFsPowerDown.startTrigger.bEnaCmd     = 0;
  rfc_tmp_cmd.cmdFsPowerDown.startTrigger.triggerNo   = 0;
  rfc_tmp_cmd.cmdFsPowerDown.startTrigger.pastTrig    = 0;
  rfc_tmp_cmd.cmdFsPowerDown.condition.rule           = COND_NEVER;
  rfc_tmp_cmd.cmdFsPowerDown.condition.nSkip          = 0;
  
  /* 2.3 Turn of synth */
  RFC_sendRadioOp((rfc_radioOp_t *)&rfc_tmp_cmd.cmdFsPowerDown);
  
  if(rfc_tmp_cmd.cmdFsPowerDown.status != DONE_OK)
  {
    return(RFC_ERROR);
  }
  
  /* Step 3 Switch off the digital part */
  PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE);
  
  /* Reset state */
  rfc_config.bTx = 3;
  
  return(RFC_OK);
}


//*****************************************************************************
//
//! \brief Setup radio with patches, overrides and hooks for given RPID
//!
//! \param cmd Pointer to the setup command to use.
//!
//! Step 1: MCE/RFE Patches \n
//! Step 2: Submit setup command \n
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
inline rfc_returnValue_t
RFC_initRadio_nb(rfc_radioOp_t *cmd)
{
  /* Make sure really setup command */
  //  ASSERT(cmd->commandNo == CMD_PROP_RADIO_SETUP ||
  //         cmd->commandNo == CMD_RADIO_SETUP )
  
  /* Step 1. MCE/RFE Patches (MCE/RFE) */
  switch(rfc_config.rpid)
  {
    
#ifdef RFC_INCLUDE_GFSK
  case RFC_GFSK:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_BLE
  case RFC_BLE:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_802_15_4
  case RFC_802_15_4:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_OOK
  case RFC_OOK:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_LRM
  case RFC_LRM:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_T_MODE
  case RFC_T_MODE:
    
    /* Patch the RF Core */
    /* Force-enable the MCE and RFE RAM, used for patching */
    RFC_sendDirectCmd(CMDR_DIR_CMD_2BYTE(0x0607, RFC_PWR_PWMCLKEN_MDMRAM));
  
    /* Patch MCE */
    rf_patch_mce_wmbus_ctmode();
    
    /* Remove force-enable of the MCE and RFE RAM clocks */
    RFC_sendDirectCmd(CMDR_DIR_CMD_2BYTE(0x0607, 0));
    
    /* Force bus  */
    RFC_sendDirectCmd(CMDR_DIR_CMD_1BYTE(0x040E, 1));
    break;
#endif
    
  default:
    return(RFC_ERROR_PATCH);
  }
  
  /* Step 2. Radio_Setup */
  rfc_flag = 1;
  
  /* Keep track of command execution */
  rfc_config.currentCmd = cmd;
  
  /* Set current command to IDLE state */
  cmd->status = IDLE;
  
  /* Send radio setup command */
  MB_SendCommand((uint32_t) cmd);
  
  return(RFC_OK);
}


//*****************************************************************************
//
//! \brief Setup radio with correct patches
//!
//! \param cmd is a pointer to a setup command \n
//! Must be one of either:
//! - \ref rfc_CMD_RADIO_SETUP_t
//! - \ref rfc_CMD_PROP_RADIO_SETUP_t
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
rfc_returnValue_t
RFC_setupRadio(rfc_radioOp_t *cmd)
{ 
  /* Make sure really rfc_radioOp_t */
  ASSERT((((cmd->commandNo) & 0x0800) == 0x0800));
  
  OSCInterfaceEnable();
  
  /* 1. Check that XOSC HF is used */
  if(OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF)
  {
    return (RFC_ERROR);
  }
  
  OSCInterfaceDisable();
  
  /* 2. MCE/RFE Patches and submit setup command */
  rfc_returnValue_t retval = RFC_initRadio_nb(cmd);
  
  if(retval)
  {
    return(retval);
  }
  
  /* Step 2.1 Wait until command is done executing */
  while(rfc_flag);
  
  if((cmd->status & 0xF00) != DONE_OK)
  {
    return(RFC_ERROR_SETUP);
  } 
  
  return(retval);
}

//*****************************************************************************
//
//! \brief Setup radio with correct patches
//!
//! \param cmd is a pointer to a setup command \n
//! Must be one of either:
//! - \ref rfc_CMD_RADIO_SETUP_t
//! - \ref rfc_CMD_PROP_RADIO_SETUP_t
//! - \ref rfc_CMD_PROP_RADIO_DIV_SETUP_t
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
rfc_returnValue_t
RFC_setupRadio_nb(rfc_radioOp_t *cmd)
{ 
  /* Make sure really rfc_radioOp_t */
  ASSERT((((cmd->commandNo) & 0x0800) == 0x0800));
  
  /* Only possible to send this if synth is not supposed to be powered up */
  if(((rfc_CMD_PROP_RADIO_SETUP_t *)cmd)->config.bNoFsPowerUp != 0x1)
  {
    return (RFC_ERROR);
  }
  
  /* MCE/RFE Patches and submit setup command */
  rfc_returnValue_t retval = RFC_initRadio_nb(cmd);
  
  return (retval);
}

//*****************************************************************************
//
//! \brief Send a radio operation to the radio
//!
//! \param cmd Pointer to command structure
//!
//! Step 1. Pre configure for this command \n
//! Step 2. Send command to RF Core \n
//! Step 3. Sleep until command is done executing
//!
//! \return rfc_cmdStatus_t
//
//*****************************************************************************
rfc_cmdStatus_t 
RFC_sendRadioOp(rfc_radioOp_t *cmd)
{
  rfc_cmdStatus_t retval;
  
  /* Step 1. Run pre configuration for this command */
  RFC_trapCmd(cmd);
  
  /* Keep track of command execution */
  rfc_flag = 1;
  
  /* Keep track of command execution */
  rfc_config.currentCmd = cmd;
  
  /* Set current command to IDLE state */
  cmd->status = IDLE;
  
  /* Step 2. Send command to RF Core */
  retval = MB_SendCommand((uint32_t) cmd);
  
  /* Step 3. Wait until command is done executing */
  while(rfc_flag)
  {
    PRCMSleep();
  }
  
  /* Return status*/
  return(retval);
}


//*****************************************************************************
//
//! \brief Issues a non-blocking radio operation
//!
//! \param cmd is a pointer to a command structure
//! \param isr is a pointer to the isr for operation completed,
//!  can be NULL for non-blocking operation with no callback.
//!
//! Step 1. Pre configure for this command \n
//! Step 2. Send command to RF Core \n
//!
//! \return rfc_cmdStatus_t
//
//*****************************************************************************
rfc_cmdStatus_t 
RFC_sendRadioOp_nb(rfc_radioOp_t *cmd, void (*isr)(uint32_t))
{
  /* Make sure really rfc_radioOp_t */
  ASSERT((((cmd->commandNo) & 0x0800) == 0x0800));
  
  /* Step 1. Run pre configuration for this command */
  RFC_trapCmd(cmd);
  
  if(isr == NULL)
  {
    /* Keep track of command execution */
    rfc_flag = 1;
  }
  else
  {
    /* Register ISR */
    rfc_nbIsr = isr;
  }
  
  /* Keep track of command execution */
  rfc_config.currentCmd = cmd;
  
  /* Set current command to IDLE state */
  cmd->status = IDLE;
  
  /* Step 2. Send command to RF Core */
  return(MB_SendCommand((uint32_t) cmd));
}

//*****************************************************************************
//
//! \brief Sends a direct operation to the radio
//!
//! \param cmd is a direct command
//!
//! \return rfc_cmdStatus_t
//
//*****************************************************************************
rfc_cmdStatus_t 
RFC_sendDirectCmd(uint32_t cmd)
{ 
  /* Send command to RF Core */
  return(MB_SendCommand(cmd));
}


//*****************************************************************************
//
//! \brief Check if radio operation has returned
//!
//! \return 1 if completed, 0 othervise
//
//*****************************************************************************
inline uint8_t
RFC_isRadioOpCompleted()
{
  return(!rfc_flag);
}

//*****************************************************************************
//
//! \brief Select RF Mode
//!
//! \param rpid
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
rfc_returnValue_t
RFC_selectRadioMode(rfc_RPID_t rpid)
{ 
  
  if(rpid != NULL)
  {
    /* Calculate mode */
    uint8_t mode = 0xFF & RFC_GET_OTHERS(rpid);
    
    /* Try to set mode */
    HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = mode;
    
    /* Check if mode was set correctly */
    if(HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) == mode)
    {
      /* Update RPID and state*/
      rfc_config.rpid = rpid;
      rfc_config.bTx = 3;
      return(RFC_OK);
    }
    return(RFC_ERROR_MODE);
  }
  return(RFC_ERROR_RPID);
}

/* API change */
rfc_returnValue_t
RFC_initRadio(rfc_RPID_t rpid)
{
  return(RFC_selectRadioMode(rpid));
}

//*****************************************************************************
//
//! \brief Function to help with pre and post dependencies of cmds
//!
//! \param cmd is the cmd that will be intercepted
//!
//! \return None
//
//*****************************************************************************
RFC_STATIC_INLINE void
RFC_trapCmd(rfc_radioOp_t *cmd)
{
  uint16_t cmdNo = cmd->commandNo;
  
  /* Make sure really rfc_radioOp_t */
  ASSERT((((cmdNo) & 0x0800) == 0x0800));
  
  switch(cmdNo)
  {
  case CMD_PROP_TX:
  case CMD_PROP_TX_ADV:
  case CMD_TX_TEST:
    /* Tx fix */
    rfc_config.bTx = 1;
    break;
  case CMD_PROP_RX:
  case CMD_PROP_RX_ADV:
  case CMD_RX_TEST:
    /* Rx fix */
    rfc_config.bTx = 0;
    break;
  default:
    /* Do nothing */
    break;
  }
}

//*****************************************************************************
//
//! \brief Patches CPE
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
rfc_returnValue_t 
RFC_patchCpe()
{
  /* CPE Patch */
  switch(rfc_config.rpid)
  {
    
#ifdef RFC_INCLUDE_GFSK
  case RFC_GFSK:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_BLE
  case RFC_BLE:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_802_15_4
  case RFC_802_15_4:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_OOK
  case RFC_OOK:
    /* No patch */
    break;
#endif
    
#ifdef RFC_INCLUDE_LRM
  case RFC_LRM:
    /* No patch */
    break;
#endif

#ifdef RFC_INCLUDE_T_MODE
  case RFC_T_MODE:
    /* No patch */
    break;
#endif    
    
  default:
    return(RFC_ERROR_PATCH);
  }
  
  return(RFC_OK);
}



//*****************************************************************************
//
//! Enable CPE0 interrupt 
//!
//! \param mask Mask for enabling interrupt
//!
//! \return None
//
//*****************************************************************************
void RFC_enableCpe0Interrupt(uint32_t mask)
{
  /* Multiplex RF Core interrupts to CPE0 IRQ and enable the masked interrupts */
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL ) &= ~mask;
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN ) |= mask;
  
  /* Clear any pending interrupts and enable interrupts. */
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG ) = 0x0;
}

//*****************************************************************************
//
//! Enable CPE1 interrupt 
//!
//! \param mask Mask for enabling interrupt
//!
//! \return None
//
//*****************************************************************************
void RFC_enableCpe1Interrupt(uint32_t mask)
{
  /* Multiplex RF Core interrupts to CPE1 IRQ and enable the masked interrupts */
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL ) |= mask;
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN ) |= mask;
  
  /* Clear any pending interrupts and enable interrupts. */
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG ) = 0x0;
}


//*****************************************************************************
//
//! Disable CPE0 interrupt 
//!
//! \param mask Mask for disabling interrupt
//!
//! \return None
//
//*****************************************************************************
void RFC_disableCpe0Interrupt(uint32_t mask)
{
  if ((mask & IRQ_LAST_COMMAND_DONE) == 0)
  {
    /* Disable the masked interrupts */
    HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN ) &= ~mask;
    
    /* Clear any pending interrupts and enable interrupts. */
    HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG ) = 0x0;
  }
}

//*****************************************************************************
//
//! Disable CPE1 interrupt 
//!
//! \param mask Mask for disabling interrupt
//!
//! \return None
//
//*****************************************************************************
void RFC_disableCpe1Interrupt(uint32_t mask)
{
  /* Disable the masked interrupts */
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) &= ~mask;
  
  /* Clear any pending interrupts and enable interrupts. */
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;
}

//*****************************************************************************
//
//! Clear interrupt flags
//!
//! \param interruptFlags Mask of interrupts to clear
//!
//! \return None
//
//*****************************************************************************
void RFC_clearCpeInterrupts(uint32_t interruptFlags)
{
  /*  Clear interrupt */
  do{
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~interruptFlags;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & interruptFlags);
}



//*****************************************************************************
//
//! Register Radio Library ISR
//!
//! \param isr Pointer to ISR Callback
//!
//! \return None
//
//*****************************************************************************
inline void RFC_registerCpe0Isr(void (*isr)(uint32_t))
{
  rfc_cpe0Isr = isr;
}

//*****************************************************************************
//
//! Register Radio Library ISR
//!
//! \param isr Pointer to ISR Callback
//!
//! \return None
//
//*****************************************************************************
inline void RFC_registerCpe1Isr(void (*isr)(uint32_t))
{
  rfc_cpe1Isr = isr;
}


//*****************************************************************************
//
//! Radio Library ISR
//
//*****************************************************************************
void RFC_cpe0Isr(void)
{
  /* Read interrupt flags */
  uint32_t interruptFlags = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG);
  
  /* Handle interrupt */
  if(interruptFlags & IRQ_LAST_COMMAND_DONE)
  {
    rfc_flag = 0;
    
    /* Command callback if there is one*/
    if(rfc_nbIsr != NULL)
    {
      rfc_nbIsr(interruptFlags);
      rfc_nbIsr = NULL;
    }
  }
  
  /*  Clear interrupt */
  do{
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~interruptFlags;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & interruptFlags);
  
  if(rfc_cpe0Isr != NULL)
  {
    rfc_cpe0Isr(interruptFlags);
  }else
  {
    ;
  }
}

//*****************************************************************************
//
//! Radio Library ISR
//
//*****************************************************************************
void RFC_cpe1Isr(void)
{
  /* Read interrupt flags */
  uint32_t interruptFlags = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG);
  
  /*  Clear interrupt */
  do{
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~interruptFlags;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & interruptFlags);
  
  if(rfc_cpe1Isr != NULL)
  {
    rfc_cpe1Isr(interruptFlags);
  }else
  {
    ;
  }
}

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
