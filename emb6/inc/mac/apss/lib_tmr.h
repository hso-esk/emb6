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

void Tmr_Start      (LIB_TMR *p_tmr);
void Tmr_Stop       (LIB_TMR *p_tmr);
void Tmr_Update     (void);
void Tmr_Delay      (LIB_TMR_TICK  delay);

LIB_TMR_STATE  Tmr_StateGet    (LIB_TMR *p_tmr);
LIB_TMR_TICK   Tmr_RemainGet   (LIB_TMR *p_tmr);

#endif /* LIB_TMR_PRESENT */
