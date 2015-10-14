/**
 * @file    lib_tmr.c
 * @author  PN
 * @brief   Timer management module
 */

#include <stdint.h>

#define LIB_TMR_MODULE
#include <lib_tmr.h>

#ifdef IAR_COMPILER
#include "emb6_conf.h"
#endif

LIB_TMR        *TmrListHead;
LIB_TMR        *TmrListTail;
LIB_TMR_QTY     TmrListQty;
LIB_TMR_TICK    TmrCurTick;

#if LIB_TMR_DEBUG_EN
LIB_TMR        *TmrLatestUnlinked;
#endif


static void Tmr_Link (LIB_TMR *p_tmr);
static void Tmr_Unlink (LIB_TMR *p_tmr);


/**
 * @brief   Add a timer to the double-linked list of timers.
 * @param   p_tmr   Point to timer to add
 */
static void Tmr_Link (LIB_TMR *p_tmr)
{
    LIB_TMR *p_temptmr;


    if (TmrListHead != (LIB_TMR*)0) {
        p_temptmr = TmrListHead;

        while (p_temptmr != (LIB_TMR *)0) {
            if (p_temptmr->Counter < p_tmr->Counter) {
                p_temptmr = p_temptmr->Next;
            } else {
                if (p_temptmr == TmrListHead) {
                    TmrListHead = p_tmr;
                } else {
                    (p_temptmr->Prev)->Next = p_tmr;        // p_temptmr is not at head of timer list
                    p_tmr->Prev = p_temptmr->Prev;
                }
                
                p_tmr->Next = p_temptmr;
                p_temptmr->Prev = p_tmr;
                break;
            }
        }

        if (p_temptmr == (LIB_TMR *)0) {
            TmrListTail->Next = p_tmr;
            p_tmr->Prev = TmrListTail;
            TmrListTail = p_tmr;
        }
    } else {
        TmrListHead = p_tmr;
        TmrListTail = p_tmr;
    }


    TmrListQty++;
}


/**
 * @brief   Remove a timer from the double-linked list of timers
 * @param   p_tmr   Point to timer to remove
 */
static void Tmr_Unlink (LIB_TMR *p_tmr)
{
    if (TmrListHead == p_tmr) {
        if (TmrListQty > 1) {
            if (p_tmr->Next == (LIB_TMR *)0) {
                LED_ERROR();
                while (1) {
                }
            }

            TmrListHead = p_tmr->Next;
            (p_tmr->Next)->Prev = (LIB_TMR *)0;
        } else {
            TmrListHead = (LIB_TMR *)0;
            TmrListTail = (LIB_TMR *)0;
        }
    } else if (TmrListTail == p_tmr) {
        TmrListTail = p_tmr->Prev;
        (p_tmr->Prev)->Next = (LIB_TMR *)0;
    } else {
        (p_tmr->Prev)->Next = p_tmr->Next;
        (p_tmr->Next)->Prev = p_tmr->Prev;

        if (TmrListQty < 3) {
            LED_ERROR();
            while (1) {
            }
        }
    }

    p_tmr->Next = (LIB_TMR *)0;
    p_tmr->Prev = (LIB_TMR *)0;
    TmrListQty--;
}


/**
 * @brief Initialize timer management module
 */
void Tmr_Init (void)
{
    TmrListHead = (LIB_TMR *)0;
    TmrListTail = (LIB_TMR *)0;
    TmrCurTick = 0;
    TmrListQty = 0;
}


/**
 * @brief   Create a timer. The timer to be created should be set to 0 before
 *          calling Tmr_Create function.
 * @param   p_tmr   Point to timer to create
 * @param   type    Type of timer: periodic or one_shot
 * @param   period  Timer counter
 * @param   fnct    Callback function upon the timer interrupt presence
 * @param   p_cbarg Callback function argument
 */
void Tmr_Create (LIB_TMR        *p_tmr,
                 LIB_TMR_TYPE    type,
                 LIB_TMR_TICK    period,
                 FNCT_VOID       fnct,
                 void           *p_cbarg)
{

    if ((p_tmr == (LIB_TMR *) 0) || (period == (LIB_TMR_TICK) 0)) {
        return;
    }

    if (p_tmr->State == LIB_TMR_STATE_RUNNING) {
        return;
    }


    memset(p_tmr, 0, sizeof(LIB_TMR));

    p_tmr->Type = type;
    p_tmr->Period = period;
    p_tmr->Counter = 0;
    p_tmr->State = LIB_TMR_STATE_CREATED;
    p_tmr->CbFnct = fnct;
    p_tmr->CbArg = p_cbarg;
    p_tmr->Next = (LIB_TMR *)0;
    p_tmr->Prev = (LIB_TMR *)0;
}


/**
 * @brief   Start a timer. Afterwards the time is added to the double-linked
 *          list of timer.
 *
 * @param   p_tmr
 */
void Tmr_Start (LIB_TMR *p_tmr)
{
    CPU_ENTER_CRITICAL();
    if ((p_tmr->State == LIB_TMR_STATE_CREATED) ||
        (p_tmr->State == LIB_TMR_STATE_FINISHED)) {
        p_tmr->State = LIB_TMR_STATE_RUNNING;
        p_tmr->Counter = TmrCurTick + p_tmr->Period;
        Tmr_Link (p_tmr);
    }
    CPU_EXIT_CRITICAL();
}


/**
 * @brief   Stop a timer. Afterwards the timer is removed from the double-linked
 *          list of timer.
 *
 * @param   p_tmr
 */
void Tmr_Stop (LIB_TMR *p_tmr)
{
    CPU_ENTER_CRITICAL();
    if (p_tmr->State == LIB_TMR_STATE_CREATED) {
        p_tmr->State = LIB_TMR_STATE_FINISHED;
    }

    if (p_tmr->State == LIB_TMR_STATE_RUNNING) {
        p_tmr->State = LIB_TMR_STATE_FINISHED;
        Tmr_Unlink (p_tmr);
    }
    CPU_EXIT_CRITICAL();
}


/**
 * @brief
 * @param   delay
 */
void Tmr_Delay(LIB_TMR_TICK delay)
{
    LIB_TMR_TICK tick_end;


    if (delay) {
        tick_end = TmrCurTick + delay;

        while (tick_end > TmrCurTick) {
            /* Do nothing */
        }
    }
}


/**
 * @brief   Achieve remaining ticks of a timer until it interrupts.
 * @param   p_tmr   Point to the timer
 * @return
 */
LIB_TMR_TICK Tmr_RemainGet (LIB_TMR *p_tmr)
{
    return (p_tmr->Counter - p_tmr->Period);
}


/**
 * @brief   Achieve current operating state of a timer.
 * @param   p_tmr   Point to the timer
 */
LIB_TMR_STATE Tmr_StateGet (LIB_TMR *p_tmr)
{
    return p_tmr->State;
}


/**
 * @brief   Update the double-linked list of timers upon system clock interrupt
 *          event. This function should be called at rate of
 *          UTIL_TMR_TICK_FREQ_IN_HZ
 */
void Tmr_Update (void)
{
    LIB_TMR *p_tmr;


    TmrCurTick++;
    while (TmrListHead != (LIB_TMR *)0) {
        p_tmr = TmrListHead;        
        if (p_tmr->Counter > TmrCurTick) {
            break;
        }

        if (p_tmr->CbFnct != (FNCT_VOID)0) {
            p_tmr->CbFnct (p_tmr->CbArg);
        }

#if LIB_TMR_DEBUG_EN
        TmrLatestUnlinked = p_tmr;
#endif

        Tmr_Stop(p_tmr);
        if (p_tmr->Type == LIB_TMR_TYPE_PERIODIC) {
            Tmr_Start(p_tmr);
        }
    }
}
