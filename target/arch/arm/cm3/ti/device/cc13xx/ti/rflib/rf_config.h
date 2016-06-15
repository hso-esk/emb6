/******************************************************************************
*  Filename:       rf_config.h
*  Revised:        $ $
*  Revision:       $ $
*
*  Description:    Build configuration file
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
//! \addtogroup rfconfig
//! @{
//
//*****************************************************************************

#ifndef RF_CONFIG_H
#define RF_CONFIG_H

//*****************************************************************************
//! Supported frontends, include this to include the hooks for this frontend. 
//! The selection is mutual exclusive.
//! - RFC_DIFF_XB
//*****************************************************************************
#define RFC_INCLUDE_DIFF_XB        // frontend

//*****************************************************************************
//! Supported protocols, include these as needed.
//! The selection is NOT mutual exclusive.
//! These defines decide what patches are to be included as well as run time 
//! fixes. Library size will increase greatly if all are included.
//! - RFC_INCLUDE_GFSK
//! - RFC_INCLUDE_BLE
//! - RFC_INCLUDE_802_15_4
//! - RFC_INCLUDE_OOK
//! - RFC_INCLUDE_LRM
//*****************************************************************************
//#define RFC_INCLUDE_GFSK           // protocol
//#define RFC_INCLUDE_BLE            // protocol
//#define RFC_INCLUDE_802_15_4       // protocol
//#define RFC_INCLUDE_OOK            // protocol
//#define RFC_INCLUDE_LRM            // protocol


#endif

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
