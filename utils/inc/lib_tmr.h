/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*============================================================================*/

/**
 * @file    lib_tmr.h
 * @author  PN
 * @brief   Timer management module
 */

#ifndef LIB_TMR_PRESENT
#define LIB_TMR_PRESENT

#ifdef LIB_TMR_MODULE
#define LIB_TMR_EXT
#else
#define LIB_TMR_EXT     extern
#endif

#include "lib_port.h"

/*
********************************************************************************
*                           DATA TYPES DECLARATION
********************************************************************************
*/
typedef uint32_t LIB_TMR_TICK;
typedef uint8_t  LIB_TMR_TYPE;
typedef uint8_t  LIB_TMR_QTY;
typedef uint8_t  LIB_TMR_STATE;

typedef void    (*FNCT_VOID) ( void* );
typedef struct  lib_tmr LIB_TMR;


struct lib_tmr {
    LIB_TMR         *Next;
    LIB_TMR         *Prev;
    LIB_TMR_TYPE     Type;
    LIB_TMR_TICK     Period;
    LIB_TMR_TICK     Counter;
    LIB_TMR_STATE    State;
    FNCT_VOID        CbFnct;
    void            *CbArg;
};


/*
********************************************************************************
*                                   DEFINES
********************************************************************************
*/
#define LIB_TMR_DEBUG_EN                                    ( 1u )

#define LIB_TMR_TICK_FREQ_IN_HZ             (uint16_t     ) ( 1000u )

#define LIB_TMR_TYPE_ONE_SHOT               (LIB_TMR_TYPE ) ( 1u )
#define LIB_TMR_TYPE_PERIODIC               (LIB_TMR_TYPE ) ( 2u )

#define LIB_TMR_STATE_INIT                  (LIB_TMR_STATE) ( 0u )
#define LIB_TMR_STATE_CREATED               (LIB_TMR_STATE) ( 1u )
#define LIB_TMR_STATE_RUNNING               (LIB_TMR_STATE) ( 2u )
#define LIB_TMR_STATE_STOPPED               (LIB_TMR_STATE) ( 3u )
#define LIB_TMR_STATE_FINISHED              (LIB_TMR_STATE) ( 4u )


/*
********************************************************************************
*                           GLOBAL VARIABLES DECLARATION
********************************************************************************
*/
extern LIB_TMR         *TmrListHead;
extern LIB_TMR         *TmrListTail;
extern LIB_TMR_TICK     TmrCurTick;
extern LIB_TMR_QTY      TmrListQty;

#if LIB_TMR_DEBUG_EN
extern LIB_TMR         *TmrLatestUnlinked;
#endif

/*
********************************************************************************
*                           API FUNCTIONS DECLARATIONS
********************************************************************************
*/
void Tmr_Init       (void);

void Tmr_Create     (LIB_TMR      *p_tmr,
                     LIB_TMR_TYPE  type,
                     LIB_TMR_TICK  period,
                     FNCT_VOID      fnct,
                     void          *p_cb);

void Tmr_Stop       (LIB_TMR *p_tmr);
void Tmr_Start      (LIB_TMR *p_tmr);
void Tmr_Delay      (LIB_TMR_TICK  delay);
void Tmr_Update     (void);

LIB_TMR_STATE  Tmr_StateGet    (LIB_TMR *p_tmr);
LIB_TMR_TICK   Tmr_RemainGet   (LIB_TMR *p_tmr);

#endif /* LIB_TMR_PRESENT */
