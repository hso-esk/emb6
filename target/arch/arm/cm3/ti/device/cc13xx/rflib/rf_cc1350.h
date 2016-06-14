/******************************************************************************
*  Filename:       rf_cc1350.h
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
//! \addtogroup rf_cc1350_driver_api
//! @{
//
//*****************************************************************************

#ifndef RF_CC1350_H
#define RF_CC1350_H

#include <stdlib.h>

#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>
#include <driverlib/pwr_ctrl.h>

#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_mailbox.h>

#include <driverlib/rf_prop_cmd.h>
#include <driverlib/rf_prop_mailbox.h>

#include <rflib/rf_def.h>

/*******************************************************************************
* Function prototypes
*/
extern rfc_returnValue_t RFC_setupRadio(rfc_radioOp_t *cmd);
extern rfc_returnValue_t RFC_setupRadio_nb(rfc_radioOp_t *cmd);

extern rfc_returnValue_t RFC_enableRadio(void);
extern rfc_returnValue_t RFC_disableRadio(void);
extern rfc_returnValue_t RFC_selectRadioMode(rfc_RPID_t rpid);

extern rfc_cmdStatus_t RFC_sendRadioOp(rfc_radioOp_t *cmd);
extern rfc_cmdStatus_t RFC_sendRadioOp_nb(rfc_radioOp_t *cmd, void (*isr)(uint32_t));
extern rfc_cmdStatus_t RFC_sendDirectCmd(uint32_t cmd);
extern uint8_t RFC_isRadioOpCompleted(void);

extern void RFC_registerCpe0Isr(void (*isr)(uint32_t));
extern void RFC_registerCpe1Isr(void (*isr)(uint32_t));

/*******************************************************************************
* Interrupt functions
*/
extern void RFC_enableCpe0Interrupt(uint32_t mask);
extern void RFC_enableCpe1Interrupt(uint32_t mask);
extern void RFC_disableCpe0Interrupt(uint32_t mask);
extern void RFC_disableCpe1Interrupt(uint32_t mask);
extern void RFC_clearCpeInterrupts(uint32_t ifg);

// development
extern rfc_returnValue_t RFC_setupGFSK_nb(rfc_radioOp_t *cmd);

#endif

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
