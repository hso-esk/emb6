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
/**
 * \addtogroup tcpip_radio
 * @{
 */
/*============================================================================*/
/*! \file   tcpip.c

    \author Artem Yushev artem.yushev@hs-offenburg.de

    \brief  Fake radio transceiver based on UDP loopback.

   \version 1.1
*/
/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#include "emb6.h"
#include "emb6_conf.h"
#include "bsp.h"
#include "packetbuf.h"
#include "tcpip.h"
#include "etimer.h"
#include <errno.h>
#include <sys/time.h>
/* Ligthweight Communication and Marshalling (LCM) is a library to perform
communication between processes  */
#include <lcm/lcm.h>
/*==============================================================================
                                     MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_RADIO
#include    "logger.h"
#define     CHANNEL_NAME  "EMB6_NATIVE"
/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                          VARIABLE DECLARATIONS
==============================================================================*/
static  struct  etimer          tmr;
/* Pointer to the lmac structure */
static  s_nsLowMac_t*           p_lmac = NULL;
extern  uip_lladdr_t uip_lladdr;
static  lcm_t*                  pgs_lcm;
/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/
/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
==============================================================================*/
/* Radio transceiver local functions */
static    void      _printAndExit(const char* rpc_reason);
static    int8_t    _tcpip_on(void);
static    int8_t    _tcpip_off(void);
static    int8_t    _tcpip_init(s_ns_t* p_netStack);
static    int8_t    _tcpip_send(const void *pr_payload, uint8_t c_len);
static    void      _tcpip_read(const lcm_recv_buf_t *rbuf, const char * channel,
                                void * p_macAddr);
static    void      _tcpip_handler(c_event_t c_event, p_data_t p_data);
/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/
const s_nsIf_t  tcpip_driver = {
        "tcpip_driver",
        _tcpip_init,
        _tcpip_send,
        _tcpip_on,
        _tcpip_off,
};
/*==============================================================================
                                LOCAL FUNCTIONS
==============================================================================*/
/*----------------------------------------------------------------------------*/
/** \brief  This function reports the error and exits back to the shell.
 *
 *  \param  rpc_reason  Error to show
 *  \return Node
 */
/*----------------------------------------------------------------------------*/
static void _printAndExit(const char* rpc_reason)
{
    fputs(strerror(errno),stderr);
    fputs(": ",stderr);
    fputs(rpc_reason, stderr);
    fputc('\n',stderr);
    exit(1);
}

#ifdef __TCPIPROLE_BRSERVER__
/*----------------------------------------------------------------------------*/
/** \brief  TCPIP initialization for broadcasting on loopback
 *
 *  \param  p_netStack    Pointer to s network stack.
 *  \return int8_t        Status code.
 */
/*----------------------------------------------------------------------------*/
static int8_t _tcpip_init(s_ns_t* p_netStack)
{
    linkaddr_t      un_addr;
    uint8_t         l_error;
    /* Please refer to lcm_create() reference. Deafult parameter is taken 
       from there */
    pgs_lcm = lcm_create(NULL);

    if(!pgs_lcm)
      return 1;
    
    LOG_INFO("%s\n\r","tcpip driver was initialized");

    if (mac_phy_config.mac_address == NULL) {
        _printAndExit("MAC address is NULL");
    }

    lcm_subscribe (pgs_lcm, CHANNEL_NAME, _tcpip_read,
                   &mac_phy_config.mac_address);

    /* Initialise global lladdr structure with a given mac */
    memcpy((void *)&un_addr.u8,  &mac_phy_config.mac_address, 8);
    memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
    linkaddr_set_node_addr(&un_addr);

    LOG_INFO("MAC address %x:%x:%x:%x:%x:%x:%x:%x",    \
            un_addr.u8[0],un_addr.u8[1],\
            un_addr.u8[2],un_addr.u8[3],\
	    un_addr.u8[4],un_addr.u8[5],	\
            un_addr.u8[6],un_addr.u8[7]);

    if (p_netStack->lmac != NULL) {
        p_lmac = p_netStack->lmac;
        l_error = 1;
    } else {
        _printAndExit("Bad lmac pointer");
    }

    /* Start the packet receive process */
    etimer_set(&tmr, 10, _tcpip_handler);
    LOG_INFO("set %p for %p callback\n\r",&tmr, &_tcpip_handler);
    
    return l_error;
} /* _tcpip_init() */
#else
/*----------------------------------------------------------------------------*/
/** \brief  TCPIP initialization for broadcasting on loopback
 *
 *  \param  p_netStack    Pointer to s network stack.
 *  \return int8_t        Status code.
 */
/*----------------------------------------------------------------------------*/
static int8_t _tcpip_init(s_ns_t* p_netStack)
{
    linkaddr_t      un_addr;
    uint8_t         l_error;
    LOG_INFO("Try to initialize Broadcasting Client for tcpip radio driver");

    /* Please refer to lcm_create() reference. Deafult parameter is taken
       from there */
    pgs_lcm = lcm_create(NULL);

    if(!pgs_lcm)
      return 1;

    lcm_subscribe (pgs_lcm, CHANNEL_NAME, _tcpip_read, NULL);

    LOG_INFO("%s\n\r","tcpip driver was initialized");

    if (mac_phy_config.mac_address == NULL) {
        _printAndExit("MAC address is NULL");
    }

    /* Initialise global lladdr structure with a given mac */
    memcpy((void *)&un_addr.u8,  &mac_phy_config.mac_address, 8);
    memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
    linkaddr_set_node_addr(&un_addr);

    LOG_INFO("MAC address %x:%x:%x:%x:%x:%x:%x:%x",    \
            un_addr.u8[0],un_addr.u8[1],\
            un_addr.u8[2],un_addr.u8[3],\
            un_addr.u8[4],un_addr.u8[5],\
            un_addr.u8[6],un_addr.u8[7]);

    if (p_netStack->lmac != NULL) {
        p_lmac = p_netStack->lmac;
        l_error = 1;
    } else {
        _printAndExit("Bad lmac pointer");
    }

    /* Start the packet receive process */
    etimer_set(&tmr, 10, _tcpip_handler);
    LOG_INFO("set %p for %p callback\n\r",&tmr, &_tcpip_handler);

    return l_error;
} /* _tcpip_init() */
#endif

#define __ADDRLEN__    2
/*---------------------------------------------------------------------------*/
static int8_t _tcpip_send(const void *pr_payload, uint8_t c_len)
{
    uint8_t _ret;
    uint8_t i = 0;
    LOG_INFO("Try to send packet...");
    uint8_t* pc_frame = malloc(sizeof(uint8_t)*(c_len + __ADDRLEN__));
    if (pc_frame == NULL) {
        LOG_ERR("Insufficient memory");
        return RADIO_TX_ERR;
    }
    memcpy(pc_frame,uip_lladdr.addr + (8 - __ADDRLEN__),__ADDRLEN__);
    memmove(pc_frame+__ADDRLEN__,pr_payload,c_len);
    int status = lcm_publish (pgs_lcm, CHANNEL_NAME, pc_frame, c_len);
    
    free(pc_frame);
    if( status == -1 )
    {
        LOG_ERR( "sendto" );
        return RADIO_TX_ERR;
    }
    else
    {
        _ret = c_len;
        LOG_INFO( "send msg with len = %d", c_len );
        LOG2_HEXDUMP(pr_payload,c_len);

        return RADIO_TX_OK;
    }
} /* _tcpip_send() */

/*---------------------------------------------------------------------------*/
static void _tcpip_read(const lcm_recv_buf_t *rbuf, const char * channel,
		       void * p_macAddr)
{
    int i = 0;
    uint16_t  i_dSize = rbuf->data_size;
    uint8_t*    pc_frame = NULL;
     packetbuf_clear();
    if (i_dSize > PACKETBUF_SIZE) {
        LOG_ERR("Received packet too long");
    } else {
        pc_frame = malloc(sizeof(uint8_t)*(i_dSize + __ADDRLEN__));
        if (pc_frame == NULL) {
            LOG_ERR("Insufficient memory");
        } else {
            memcpy(pc_frame,rbuf->data,i_dSize);
            if ((*pc_frame != uip_lladdr.addr[6]) &&
                (*(pc_frame + 1) != uip_lladdr.addr[7]))
            {
                LOG_INFO("Sender's MAC %x:%x, message len %d", *pc_frame,*(pc_frame + 1),i_dSize);
                memcpy(packetbuf_dataptr(),
                       rbuf->data + __ADDRLEN__, i_dSize - __ADDRLEN__);
                LOG2_HEXDUMP(packetbuf_dataptr(),i_dSize - __ADDRLEN__);
                if((rbuf->data_size > 0) && (p_lmac != NULL)) {
                        packetbuf_set_datalen(i_dSize - __ADDRLEN__);
                        p_lmac->input();
                }
            }
        }
    }
} /* _tcpip_read() */

/*---------------------------------------------------------------------------*/
static int8_t _tcpip_on(void)
{
  return 0;
} /* _tcpip_on() */

/*---------------------------------------------------------------------------*/
static int8_t _tcpip_off(void)
{
  return 0;
}  /* _tcpip_off() */

/*---------------------------------------------------------------------------*/
static void _tcpip_handler(c_event_t c_event, p_data_t p_data)
{
    if (etimer_expired(&tmr))
    {
        /* We can't use lcm_handle trigger every time, as
         * it's a blocking operation. we should check whether a lcm file
         * descriptor is available for reading. We put 10 usec as a timeout*/
        int lcm_fd = lcm_get_fileno(pgs_lcm);
        struct timeval s_tv;
        s_tv.tv_sec = 0;
        s_tv.tv_usec = 10;
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);

        select(lcm_fd + 1, &fds, 0, 0, &s_tv);
        if ( FD_ISSET(lcm_fd, &fds)) {
            lcm_handle(pgs_lcm);
        }

        etimer_restart(&tmr);

    }
}



/*==============================================================================
                                API FUNCTIONS
==============================================================================*/
/** @} */
