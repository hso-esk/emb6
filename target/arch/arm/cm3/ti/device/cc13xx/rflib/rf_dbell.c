/******************************************************************************
*  Filename:       rf_dbell.c
*  Revised:        $ $
*  Revision:       $ $
*
*  Description:    This file contains the data structures and APIs for the doorbell.
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
//! \addtogroup rflib_api
//! @{
//
//*****************************************************************************

/*******************************************************************************
 * INCLUDES
 */

#include <driverlib/rfc.h>
#include <driverlib/interrupt.h>        // with IntRegister not else

#include <rflib/rf_dbell.h>

/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8_t mbFree;                       // True: MB is free, False: MB busy.
  uint8_t taskID;                       // Task ID to notify (if used).
  uint32_t eventID;                      // Task Event ID of notification (if used).
} mbState_t;

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
#if defined(__IAR_SYSTEMS_ICC__)
__no_init mbIntCback_t mbIntCbackTable[NUM_MAILBOX_INTERRUPTS];
#elif defined(__TI_COMPILER_VERSION__)
mbIntCback_t mbIntCbackTable[NUM_MAILBOX_INTERRUPTS];
#else
mbIntCback_t mbIntCbackTable[NUM_MAILBOX_INTERRUPTS];
#endif

/*******************************************************************************
 * PROTOTYPES
 */

void MB_DisableInts(void);
void mbCpe0Isr(void);
void mbCpe1Isr(void);


/*******************************************************************************
 * @fn          MB_Init
 *
 * @brief       This function is used to initialize the Mailbox interface. It
 *              masks and clears all interrupts, clears the CPE and HW jump
 *              table, and disables the mailbox command acknowlege interrupt.
 *
 *              Note: If the ACK interrupt is needed, it can be enabled and
 *                    a user ISR can be registered for processing.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void MB_Init(void)
{
  uint8_t i;

  // disable and clear all interrupts
  MB_DisableInts();

  // clear the Mailbox interrupt callback table
  for(i=0; i<NUM_MAILBOX_INTERRUPTS; i++)
  {
    // clear the entry
    mbIntCbackTable[i] = NULL;
  }

  IntRegister(INT_RFC_CPE_0, &mbCpe0Isr);
  IntRegister(INT_RFC_CPE_1, &mbCpe1Isr);
  
  return;
}


/*******************************************************************************
 * @fn          MB_EnableInts
 *
 * @brief       This function is used to map CPE 0 and 1, and HW interrupts,
 *              and clears/unmasks them. These interrupts are then enabled.
 *
 * input parameters
 *
 * @param       cpe0Ints - Bit map of CPE 0 interrupts.
 * @param       cpe1Ints - Bit map of CPE 1 interrupts.
 * @param       hwInts   - Bit map of HW interrupts.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void MB_EnableInts(uint32_t cpe0Ints, uint32_t cpe1Ints, uint32_t hwInts)
{
  // disable all Mailbox interrupts
  MB_DisableInts();

  // map bit map of all corresponding interrupts to CPE 0
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL) &= ~cpe0Ints;

  // map bit map of all corresponding interrupts to CPE 1
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL) |= cpe1Ints;

  // unmask all corresponding interrupts for either CPE 0 or 1
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) |= (cpe0Ints | cpe1Ints);

  // unmask all corresponding interrupts for HW
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN) |= hwInts;

  return;
}


/*******************************************************************************
 * @fn          MB_DisableInts
 *
 * @brief       This function is used to disable and clear all CPE 0 and 1,
 *              and HW interrupts.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void MB_DisableInts(void)
{
  // mask RF CPE interrupts
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) = 0;

  // mask RF HW interrupts
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN) = 0;

  // clear RF CPE interrupts
  do
  {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != 0);

  // clear RF HW interrupts
  do
  {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) != 0);

  return;
}


/*******************************************************************************
 * @fn          MB_ClearInts
 *
 * @brief       This function is used to clear all unmasked CPE/HW interrupts.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void MB_ClearInts(void)
{
  // clear RF CPE interrupts
  do
  {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != 0);

  // clear RF HW interrupts
  do
  {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) != 0);

  return;
}
/*******************************************************************************
 * @fn          MB_EnableHWInts
 *
 * @brief       This function is used to map only HW interrupts, and
 *              clears/unmasks them. These interrupts are then enabled.
 *
 * input parameters
 *
 * @param       hwInts - Bit map of HW interrupts.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void MB_EnableHWInts(uint32_t hwInts)
{
  // mask RF HW interrupts
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN) = 0;

  // clear RF HW interrupts
  do
  {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) != 0);

  // clear RF HW interrupts
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN) |= hwInts;

  return;
}


/*******************************************************************************
 * @fn          MB_SendCommand
 *
 * @brief       This function is used to send a command to the Mailbox
 *              interface, then poll for the the command status to change
 *              indicating that the command has completed (in the case of an
 *              Immediate or Direct command), or the command has been accepted
 *              for execution (in the case of a Radio command).
 *
 *              Note that this routine only returns the command status result.
 *              If the command status return bytes are required, they can be
 *              obtained using MB_ReadMailboxStatus.
 *
 *              Note: This command can safely execute from a critical section.
 *
 * input parameters
 *
 * @param       radioCmd - Immediate, Direct, or Radio command.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      A CMDSTA value, or CMDSTA_MAILBOX_BUSY.
 */
uint8_t MB_SendCommand(uint32_t radioCmd)
{
  // check that the Mailbox is not busy
  if(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) != CMDSTA_Pending)
  {
    return(CMDSTA_MAILBOX_BUSY);
  }
  
  // issue the command to the Mailbox interface
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = radioCmd;

  // wait for the the command status value to change by using the ACK interrupt
  while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG));
  do
  {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) != 0);

  return(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA) & 0xFF);
}


/*******************************************************************************
 * @fn          MB_SendCommandSynch
 *
 * @brief       This function can be used to issue an RFHAL command via the
 *              Mailbox interface. This function allows the user to issue a
 *              RFHAL command completely synchronously, without concern for
 *              interrupt processing. This function therefore assumes, in the
 *              case of Radio Commands, that no other radio command is pending.
 *
 *              Before the command is issued, all unmasked CPE/HW interrupts
 *              are saved, and all interrupts are masked. After the command is
 *              issued, the command status is polled until a status change has
 *              occurred, indicating that the command has either completed in
 *              the case of an Immediate or Direct command, or the command has
 *              been accepted for execution in the case of a Radio command.
 *              If the command is a Radio Command, and the Mailbox command
 *              status completed successfully, the Last Command Done interrupt
 *              is polled, and the Last Command and Last Command Done interrupts
 *              are cleared (whether or not they were unmasked by the user).
 *              In all cases, all unmasked interrupts are cleared, the user's
 *              mask register is restored, and the Mailbox command status is
 *              returned.
 *
 *              WARNING: Radio commands MUST have an end trigger or timeout
 *                       trigger that will eventually end or this command will
 *                       hang waiting for a Command Done or Last Command Done
 *                       interrupt that never occurs.
 *
 *              WARNING: This routine assumes there is no other radio command
 *                       pending.
 *
 *              Note: This routine only returns the command status result.
 *                    If the command status return bytes are required, they
 *                    can be obtained using MB_ReadMailboxStatus.
 *
 *              Note: This command can safely execute from a critical section.
 *
 * input parameters
 *
 * @param       rfCmd - Immediate, Direct, or Radio command.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      A CMDSTA value, or CMDSTA_MAILBOX_BUSY.
 */
uint8_t MB_SendCommandSynch(uint32_t rfCmd)
{
  uint32_t cpeIntMask;
  uint32_t hwIntMask;

  // check that the Mailbox is not busy
  if(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) != CMDSTA_Pending)
  {
    return(CMDSTA_MAILBOX_BUSY);
  }

  // get the currently unmasked CPE and HW interrupts
  cpeIntMask = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN);
  hwIntMask  = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN);

  // mask the interrupts
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) = 0;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN)  = 0;

  // issue the command to the Mailbox interface
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = rfCmd;

  // wait for the the command status value to change by using the ACK interrupt
  while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG));
  do
  {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) != 0);

  if(!(rfCmd & 0x01) && (*((uint16_t *)rfCmd) & 0x0800))
  {
    // ensure that the Mailbox command completed successfully
    if(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA) == CMDSTA_Done)
    {
      // poll last command done interrupt
      while(!(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & IRQ_LAST_COMMAND_DONE));

      // clear Last Command and Last Command Done interrupts
      HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~(IRQ_COMMAND_DONE | IRQ_LAST_COMMAND_DONE);
      HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~(IRQ_COMMAND_DONE | IRQ_LAST_COMMAND_DONE);
    }
  }

  // finished, so clear all unmasked interrupts
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~cpeIntMask;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~cpeIntMask;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG)  = ~hwIntMask;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG)  = ~hwIntMask;

  // restore the interrupt mask
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) = cpeIntMask;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN)  = hwIntMask;

  return(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA) & 0xFF);
}


/*******************************************************************************
 * @fn          MB_RegisterIsrCback
 *
 * @brief       This routine registers a Mailbox Interrupt Callback for one
 *              of the Mailbox interrupts.
 *
 * input parameters
 *
 * @param       mbIntType  - Interrupt for associated Callback function.
 * @param       mbIntCback - Interrupt Callback function.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void MB_RegisterIsrCback(uint8_t mbIntType, mbIntCback_t mbIntCback)
{
  // check the Mailbox interrupt type
  mbIntCbackTable[mbIntType] = mbIntCback;

  return;
}



/*
** Mailbox Interrupt Service Routines
*/


/*******************************************************************************
 * @fn          Mailbox Command Processor Engine 0 ISR
 *
 * @brief       This ISR handles the Mailbox Comamnd Processing Engine 0
 *              interrupt, or transfers control to a registered Callback to
 *              handle it.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void mbCpe0Isr(void)
{
  // check if there's a registered handler for this interrupt
  if (mbIntCbackTable[RF_CPE_0_INTERRUPT] == NULL)
  {
    uint32_t        intClear;

    // build mask to clear CPE0 interrupts
    intClear  =  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN);
    intClear &= ~HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL);

    // clear any originally unmasked CPE0 interrupts that may now be pending
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~intClear;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~intClear;
  }
  else // there's a registered handler
  {
    // so use it
    (mbIntCbackTable[RF_CPE_0_INTERRUPT])();
  }

  return;
}


/*******************************************************************************
 * @fn          Mailbox Command Processor Engine 1 ISR
 *
 * @brief       This ISR handles the Mailbox Comamnd Processing Engine 1
 *              interrupt, or transfers control to a registered Callback to
 *              handle it.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void mbCpe1Isr(void)
{
  // check if there's a registered handler for this interrupt
  if(mbIntCbackTable[RF_CPE_1_INTERRUPT] == NULL)
  {
    uint32_t intClear;
    
    // build mask to clear CPE0 interrupts
    intClear  = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN);
    intClear &= HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL);

    // clear any originally unmasked CPE1 interrupts that may now be pending
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~intClear;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~intClear;
  }
  else // there's a registered handler
  {
    // so use it
    (mbIntCbackTable[RF_CPE_1_INTERRUPT])();
  }

  return;
}


/*******************************************************************************
 * @fn          Mailbox Hardware ISR
 *
 * @brief       This ISR handles the Mailbox Hardware interrupt, or transfers
 *              control to a registered Callback to handle it.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void mbHwIsr(void)
{
  // check if there's a registered handler for this interrupt
  if(mbIntCbackTable[RF_HW_INTERRUPT] == NULL)
  {
    uint32_t        intClear;
    
    // build mask to clear CPE0 interrupts
    intClear = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN);

    // clear any originally unmasked HW interrupts that may now be pending
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = ~intClear;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = ~intClear;
  }
  else // there's a registered handler
  {
    // so use it
    (mbIntCbackTable[RF_HW_INTERRUPT])();
  }

  return;
}


/*******************************************************************************
 * @fn          Mailbox Command Acknowledge ISR
 *
 * @brief       This ISR handles the Mailbox Comamnd Acknowledge interrupt, or
 *              transfers control to a registered Callback to handle it.
 *
 *              Note: The Mailbox Command ACK interrupt can not be masked.
 *              Note: There may be additional radio interrupts that may result.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void mbCmdAckIsr(void)
{
  // always clear the interrupt
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0;
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0;

  // check if there's a registered handler for this interrupt
  if(mbIntCbackTable[RF_CMD_ACK_INTERRUPT] == NULL)
  {
    // nothing to clear, nothing to do
    return;
  }
  else // there's a registered handler
  {
    // so use it
    (mbIntCbackTable[RF_CMD_ACK_INTERRUPT])();
  }

  return;
}
/*******************************************************************************
THE FUNCTIONS HERE ARE FROM RF_CCTAG_LO.C */

//*****************************************************************************
//
//! \brief Applies rfe/mce patches
//!
//! \param rfe is a pointer to the function that patches rfe
//! \param mce is a pointer to the function that patches mce
//!
//! \return rfc_returnValue_t
//
//*****************************************************************************
rfc_returnValue_t
RFC_patch(void (*rfe)(void),void (*mce)(void))
{
  
  switch((uint32_t)*rfe)
  {
  case 0:
    switch((uint32_t)*mce)
    {
    case 0:
      break;
    default:
      
      /* Force-enable the MCE RAM, used for patching */
      /* Wait until received, parsed and running on RF Core */
      /*  Clear interrupt */
      MB_SendCommand(CMDR_DIR_CMD_2BYTE(0x0607, RFC_PWR_PWMCLKEN_MDMRAM));
      
      /* Patch MCE */
      mce();
    }
    break;
  default:
    switch((uint32_t)*mce)
    {
    case 0:
      /* Force-enable the RFE RAM, used for patching */
      /* Wait until received, parsed and running on RF Core */
      /*  Clear interrupt */
      MB_SendCommand(CMDR_DIR_CMD_2BYTE(0x0607, RFC_PWR_PWMCLKEN_RFERAM));
      
      /* Patch RFE */
      rfe();
      break;
    default:
      /* Force-enable the MCE and RFE RAM, used for patching */
      /* Wait until received, parsed and running on RF Core */
      /*  Clear interrupt */
      MB_SendCommand(CMDR_DIR_CMD_2BYTE(0x0607, RFC_PWR_PWMCLKEN_MDMRAM | RFC_PWR_PWMCLKEN_RFERAM));
      
      /* Patch RFE */
      rfe();
      
      /* Patch MCE */
      mce();
    }
  }
  
  /* Remove force-enable of the MCE and RFE RAM clocks */
  /* Wait until received, parsed and running on RF Core */
  /*  Clear interrupt */
  MB_SendCommand(CMDR_DIR_CMD_2BYTE(0x0607, 0));
  
  /* Force bus enabled */
  /* Wait until received, parsed and running on RF Core */
  /*  Clear interrupt */
  MB_SendCommand(0x11280003);
  
  return(RFC_OK);
}

/*******************************************************************************
 */

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
