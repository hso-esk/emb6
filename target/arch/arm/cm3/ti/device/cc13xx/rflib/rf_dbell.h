/******************************************************************************
*  Filename:       rf_dbell.h
*  Revised:        $ $
*  Revision:       $ $
*
*  Description:    This file contains the defines and declarations for rf_lib.c
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


#ifndef RF_DBELL_H
#define RF_DBELL_H

#include <rflib/rf_def.h>
#include <driverlib/rf_mailbox.h>
#include <inc/hw_rfc_dbell.h>

// Mailbox Command Status Result (CMDSTA)
#define CMDSTA_MAILBOX_BUSY            0xFF  // Software only status.

// Mailbox RF Interrupt Types
#define NUM_MAILBOX_INTERRUPTS         4

// Miscellaneous
#define INVALID_TASK_ID                0xFF

// Mailbox Interrupt Types
#define RF_CMD_ACK_INTERRUPT           0
#define RF_CPE_0_INTERRUPT             1
#define RF_CPE_1_INTERRUPT             2
#define RF_HW_INTERRUPT                3

typedef void (*mbIntCback_t)(void);

extern mbIntCback_t mbIntCbackTable[];

/*******************************************************************************
* Function prototypes
*/

extern void MB_Init( void );
extern void MB_EnableInts( uint32_t, uint32_t, uint32_t );
extern void MB_DisableInts( void );
extern void MB_ClearInts( void );
extern void MB_EnableHWInts( uint32_t );
extern uint8_t MB_SendCommand( uint32_t );
extern uint8_t MB_SendCommandSynch( uint32_t );
extern void MB_RegisterIsrCback( uint8_t, mbIntCback_t );

// Mailbox ISRs
extern void   mbCmdAckIsr(void);
extern void   mbCpe0Isr( void );
extern void   mbCpe1Isr( void );
extern void   mbHwIsr( void );


extern rfc_returnValue_t RFC_patch(void (*rfe)(void),void (*mce)(void));

#endif


//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
