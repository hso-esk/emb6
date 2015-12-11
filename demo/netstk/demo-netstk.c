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
 * @file    demo-netstk.c
 * @author  Phuong Nguyen
 * @brief   Demo application for emb6 netstack
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "bsp.h"
#include "board_conf.h"
#include "evproc.h"
#include "packetbuf.h"
#include "lib_tmr.h"
//#include "demo.h"

/*
********************************************************************************
*                               LOCAL DATA TYPES
********************************************************************************
*/
typedef enum app_state
{
    APP_STATE_IDLE = 0U,
    APP_STATE_TX_STARTED,
    APP_STATE_TX_BUSY,
    APP_STATE_TX_DONE,
    APP_STATE_RX_DONE,

}APP_STATE;


/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define APP_PAYLOAD_REQ             0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,    \
                                    0x60, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,    \
                                    0x70, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59

#define APP_PAYLOAD_RSP             0xa0, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,    \
                                    0xb0, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,    \
                                    0xc0, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59

#define APP_TMR_PERIOD_TX_PAYLOAD           (uint32_t )( 100u )


#define APP_SURFIX_MACADDR          0xFF, 0x00, 0x00, 0x19, 0x88, 0x12, 0x01,

#if     defined(LPR_TX)
#define APP_SRC_MACADDR             0x11, APP_SURFIX_MACADDR
#define APP_DST_MACADDR             0x22, APP_SURFIX_MACADDR
#define APP_CFG_UART_EN             ( 1U )

#elif   defined(LPR_RX)
#define APP_SRC_MACADDR             0x22, APP_SURFIX_MACADDR
#define APP_DST_MACADDR             0x11, APP_SURFIX_MACADDR
#define APP_CFG_UART_EN             ( 0U )

#else
#error  "Either LPR_TX or LPR_RX was not selected"
#endif


#define APP_EVENT_PEND(_event_)       evproc_regCallback((_event_), App_CbEvproc)
#define APP_EVENT_POST(_event_)       evproc_putEvent(E_EVPROC_HEAD, (_event_), NULL)

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static  s_ns_t              App_Netstk;
static  uint32_t            App_TxQty;
static  uint32_t            App_TxOk;
static  e_nsErr_t           App_LastErr;
static  uint32_t            App_RxOk;
static  APP_STATE           App_State;
static  LIB_TMR             App_TmrTxPayload;

static  uint8_t App_PayloadReq[] = { APP_PAYLOAD_REQ };
static  uint8_t App_SrcMACAddr[] = { APP_SRC_MACADDR };
static  uint8_t App_DstMACAddr[] = { APP_DST_MACADDR };


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
e_nsErr_t NetstkLastErr;


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
/**
 * @brief   Data reception handler
 */
static void App_IsrRx(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    int is_dif;

    is_dif = strncmp((const char *)App_PayloadReq,
                     (const char *)p_data,
                      len);
    if (!is_dif) {
        App_RxOk++;
        App_State = APP_STATE_RX_DONE;
        APP_EVENT_POST(NETSTK_APP_EVENT_TX);
    }
}


/**
 * @brief   Data transmission timer interrupt handler
 * @param p_arg
 */
static void App_IsrTx(void *p_arg)
{
    if (App_State == APP_STATE_IDLE) {
        App_State = APP_STATE_TX_STARTED;
        APP_EVENT_POST(NETSTK_APP_EVENT_TX);
    }
}


/**
 * @brief   Data confirmation handler
 * @param   p_stk
 * @param   status
 * @param   transmissions
 */
static void App_CbTx(void *p_arg, e_nsErr_t *p_err)
{
    if (App_State == APP_STATE_TX_BUSY) {
        App_State = APP_STATE_TX_DONE;
        App_LastErr = *p_err;
        APP_EVENT_POST(NETSTK_APP_EVENT_TX);
    }
}


static void App_DataReq(e_nsErr_t *p_err)
{
    /*
     * Write packet to send into common packet buffer
     */
    packetbuf_clear();

    packetbuf_set_addr(PACKETBUF_ADDR_SENDER,
                       (linkaddr_t *)App_SrcMACAddr);

    packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER,
                       (linkaddr_t *)App_DstMACAddr);

    packetbuf_set_attr(PACKETBUF_ATTR_RELIABLE, 1);

    packetbuf_set_datalen(sizeof(App_PayloadReq));
    memcpy(packetbuf_dataptr(), App_PayloadReq, sizeof(App_PayloadReq));

    /*
     * set TX callback function and argument
     */
    App_Netstk.dllc->ioctrl(NETSTK_CMD_TX_CBFNCT_SET,
                           (void *)App_CbTx,
                           p_err);

    App_Netstk.dllc->ioctrl(NETSTK_CMD_TX_CBARG_SET,
                           NULL,
                           p_err);

    /*
     * Issue next lower layer to transmit the prepared packet
     */
    App_Netstk.dllc->send(packetbuf_hdrptr(),
                         packetbuf_totlen(),
                         p_err);
}


/**
 * @brief   Event-process handler of the application
 * @param   c_event     Event that has been raised
 * @param   p_data      Point to callback argument that was registered along
 *                      with event registration
 */
static void App_CbEvproc(c_event_t c_event, p_data_t p_data)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    switch (App_State) {
        case APP_STATE_IDLE:
            break;

        case APP_STATE_TX_STARTED:
            App_DataReq(&err);
            if (err == NETSTK_ERR_NONE) {
                App_TxQty++;
                App_State = APP_STATE_TX_BUSY;
            } else {
                App_State = APP_STATE_IDLE;
            }
            break;

        case APP_STATE_TX_BUSY:
            break;

        case APP_STATE_TX_DONE:
            if (App_LastErr == NETSTK_ERR_NONE) {
                App_TxOk++;
            }
            App_State = APP_STATE_IDLE;

#if APP_CFG_UART_EN
            printf("TX: %5lu/ %5lu/ %5d \r\n", App_TxOk, App_TxQty, App_LastErr);
#endif
            break;

        case APP_STATE_RX_DONE:
            App_State = APP_STATE_IDLE;

#if APP_CFG_UART_EN
            printf("RX: %5lu/ %5d\r\n", App_RxOk, NetstkLastErr);
#endif
            break;

        default:
            break;
    }
}


/**
 * @brief   Initialization of stack to be used by the application
 * @param   p_stk
 */
static void App_StkInit (s_ns_t *p_stk)
{
    e_nsErr_t err = NETSTK_ERR_NONE;


    /*
     * Configure stack address
     */
    linkaddr_set_node_addr((linkaddr_t *)App_SrcMACAddr);
    memcpy(mac_phy_config.mac_address, App_SrcMACAddr, sizeof(App_SrcMACAddr));
    mac_phy_config.pan_id = 0xabcd;

    /*
     * Initialize stack submodules
     */
    App_Netstk.rf->init(p_stk, &err);
    App_Netstk.phy->init(p_stk, &err);
    App_Netstk.mac->init(p_stk, &err);
    App_Netstk.dllc->init(p_stk, &err);

    /*
     * Set LLC RX callback function
     */
    App_Netstk.dllc->ioctrl(NETSTK_CMD_RX_CBFNT_SET,
                           (void *)App_IsrRx,
                           &err);
    
    /*
     * In normal operation mode, netstack drivers are always powered on.
     */
    App_Netstk.dllc->on(&err);
}


/**
 * @brief   Initialization of the application
 */
static void App_Init (void)
{
    Tmr_Init();
    bsp_init(&App_Netstk);
    //board_conf(&App_Netstk);

    App_TxQty   = 0;
    App_TxOk    = 0;
    App_RxOk    = 0;
    App_State   = APP_STATE_IDLE;
    App_LastErr = NETSTK_ERR_NONE;
    App_StkInit(&App_Netstk);


#if 0
    memset(&App_TmrTxPayload, 0, sizeof(App_TmrTxPayload));
    Tmr_Create(&App_TmrTxPayload,
                LIB_TMR_TYPE_PERIODIC,
                APP_TMR_PERIOD_TX_PAYLOAD,
                App_IsrTx,
                NULL);
    Tmr_Start (&App_TmrTxPayload);
#else
#if defined(LPR_TX)
    memset(&App_TmrTxPayload, 0, sizeof(App_TmrTxPayload));
    Tmr_Create(&App_TmrTxPayload,
                LIB_TMR_TYPE_PERIODIC,
                APP_TMR_PERIOD_TX_PAYLOAD,
                App_IsrTx,
                NULL);
    Tmr_Start (&App_TmrTxPayload);
#endif
#endif

    APP_EVENT_PEND(NETSTK_APP_EVENT_TX);
}


/**
 * @brief   Main function
 * @return
 */
int main (void)
{
    e_nsErr_t stk_err;


    /*
     * Initialize APP
     */
    App_Init();


    /*
     * APP's endless loop
     */
    stk_err = NETSTK_ERR_NONE;
    while (TRUE) {
        /*
         * Prevent unreachable warning
         */
        if (stk_err != NETSTK_ERR_NONE) {
            break;
        }

        /*
         * Event-processing handler
         */
        evproc_nextEvent();
    }


    /*
     * Terminate APP
     */
    return -1;
}

/**
 * @brief   Emb6 stack error handler
 * @param   p_err
 */
void emb6_errorHandler(e_nsErr_t *p_err)
{
    LED_ERROR();
    while (1) {

    }
}
