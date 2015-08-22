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

   \version 0.0.1
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
#include "evproc.h"
#include "etimer.h"
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "mkaddr.h"
/*==============================================================================
                                     MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_RADIO
#if            LOGGER_ENABLE     ==     TRUE
#define     LOGGER_SUBSYSTEM    "tcpip"
#endif
#include    "logger.h"

/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                          VARIABLE DECLARATIONS
==============================================================================*/
static  int                     l_sockfd;
        struct sockaddr_in      s_bcAddr;       /* AF_INET */
static  int32_t                 l_bcLen;        /* length */
static  uint8_t                 tmp_buf[PACKETBUF_SIZE];
static  struct  etimer          tmr;
/* Pointer to the lmac structure */
static  s_nsLowMac_t*           p_lmac = NULL;
extern  uip_lladdr_t uip_lladdr;
/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/
/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
==============================================================================*/
/* Radio transceiver local functions */
static    void            _printAndExit(const char* rpc_reason);
static    int8_t          _tcpip_on(void);
static    int8_t          _tcpip_off(void);
static    int8_t          _tcpip_init(s_ns_t* p_netStack);
static    int8_t          _tcpip_send(const void *pr_payload, uint8_t c_len);
static    void            _tcpip_handler(c_event_t ev, p_data_t data);

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
    int32_t             l_error;        /* Status return code */
    struct sockaddr_in  s_srvAddr;      /* AF_INET */
    int32_t             l_srvLen;       /* length */
    static int          l_sobr          = TRUE;
    static char*        pc_srvAddrPre   = "127.0.0.1:*";
    static char*        bc_brAddrPre    = "127.255.255.255:9097";
    linkaddr_t          un_addr;

    LOG_INFO("Try to initialize Broadcasting Server for tcpip radio driver");

     /*
     * Form the server address:
     */
    l_srvLen = sizeof s_srvAddr;

    l_error = mkaddr(&s_srvAddr,    /* Returned address */
                     &l_srvLen,     /* Returned length */
                     pc_srvAddrPre, /* Input string addr */
                     "udp");        /* UDP protocol */

    if ( l_error == -1 )
        _printAndExit("Bad server address");

    /*
     * Form the broadcast address:
     */
    l_bcLen = sizeof s_bcAddr;

    l_error = mkaddr(&s_bcAddr,     /* Returned address */
                     &l_bcLen,      /* Returned length */
                     bc_brAddrPre,  /* Input string addr */
                     "udp");        /* UDP protocol */

    if ( l_error == -1 )
        _printAndExit("Bad broadcast address");

    /*
     * Create a UDP socket to use:
     */
    l_sockfd = socket(AF_INET,SOCK_DGRAM,0);
    if ( l_sockfd == -1 )
        _printAndExit("socket()");

    /*
     * Allow broadcasts:
     */
    l_error = setsockopt(l_sockfd, SOL_SOCKET, SO_BROADCAST,
                         &l_sobr,sizeof l_sobr);

    if ( l_error == -1 )
        _printAndExit("setsockopt(SO_BROADCAST)");

    /*
     * Bind an address to our socket, so that client programs can listen to this
     * server:
     */
    l_error = bind(l_sockfd, (struct sockaddr *)&s_srvAddr, l_srvLen);

    if ( l_error == -1 )
        _printAndExit("bind()");

    int flags = fcntl(l_sockfd, F_GETFL, 0);
    if (flags < 0){
        LOG_ERR("%s\n\r","fcntl error");
        exit(1);
    }
    flags = (flags|O_NONBLOCK);
    fcntl(l_sockfd, F_SETFL, flags);

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
    LOG_INFO("Try to initialize Broadcasting Client for tcpip radio driver");

    int32_t                 l_error;
    int32_t                 l_addrLen;
    struct sockaddr_in      s_sockAddr;         /* AF_INET */
    int32_t                 l_inetLen;          /* length */
    char                    ac_dgram[512];      /* Recv buffer */
    static int              l_reuseAddr         = TRUE;
    static char*            pc_bcAddrPre        = "127.255.255.255:9097";
    linkaddr_t              un_addr;

    /*
     * Create a UDP socket to use:
     */
    l_sockfd = socket(AF_INET,SOCK_DGRAM,0);
    if ( l_sockfd == -1 )
        _printAndExit("socket()");

    /*
     * Form the broadcast address:
     */
    l_inetLen = sizeof s_sockAddr;

    l_error = mkaddr(&s_sockAddr, &l_inetLen, pc_bcAddrPre, "udp");

    if ( l_error == -1 )
        _printAndExit("Bad broadcast address");

    /*
     * Allow multiple listeners on the broadcast address:
     */
    l_error = setsockopt(l_sockfd, SOL_SOCKET, SO_REUSEADDR, &l_reuseAddr,
                         sizeof l_reuseAddr);

    if ( l_error == -1 )
        _printAndExit("setsockopt(SO_REUSEADDR)");

    /*
     * Bind our socket to the broadcast address:
     */
    l_error = bind(l_sockfd, (struct sockaddr *)&s_sockAddr, l_inetLen);

    if ( l_error == -1 )
        _printAndExit("bind(2)");

    int flags = fcntl(l_sockfd, F_GETFL, 0);
    if (flags < 0){
        LOG_ERR("%s\n\r","fcntl error");
        exit(1);
    }
    flags = (flags|O_NONBLOCK);
    fcntl(l_sockfd, F_SETFL, flags);

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

/*---------------------------------------------------------------------------*/
static int8_t _tcpip_send(const void *pr_payload, uint8_t c_len)
{
    uint8_t _ret;
    uint8_t i = 0;
    int ret = sendto( l_sockfd, pr_payload, c_len, 0,
            (struct sockaddr *)&s_bcAddr, l_bcLen );
    if( ret == -1 )
    {
        LOG_ERR( "sendto" );
        return RADIO_TX_ERR;
    }
    else
    {
        _ret = ret;
        LOG_INFO( "send msg with len = %d\n", ret );
        while( _ret-- )
        {
            printf( "%02X", (uint8_t)( *( (uint8_t *)pr_payload + i ) ) );
            i++;
        }
        LOG_RAW( "\n" );

        return RADIO_TX_OK;
    }
} /* _tcpip_send() */

/*---------------------------------------------------------------------------*/
static int _tcpip_read(void *buf, unsigned short buf_len)
{
    uint8_t     _ret;
    uint8_t     i=0;
    int ret = recv(l_sockfd, tmp_buf, buf_len, 0);
    if((ret == -1) && (errno != 11)){
        LOG_ERR( "recv" );
        return 0;
    }
    if(ret > 0){
            LOG_INFO("receive msg len = %d\n",ret);
            memcpy(buf,tmp_buf,ret);
            _ret = ret;
            while(_ret--){
                LOG_RAW("%02X", (uint8_t)(*((uint8_t *)tmp_buf + i)));
                i++;
            }
            LOG_RAW("\r\n");
        return ret;
    }
    return 0;
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
static void _tcpip_handler(c_event_t ev, p_data_t data)
{
    uint8_t c_len = 0;
    if (etimer_expired(&tmr)) {
        if ((c_len = _tcpip_read(packetbuf_dataptr(), PACKETBUF_SIZE)) > 0)    {
            packetbuf_set_datalen(c_len);
            if((c_len > 0) && (p_lmac != NULL)) {
                packetbuf_set_datalen(c_len);
                p_lmac->input();
            }
        }
        etimer_restart(&tmr);
    }
}



/*==============================================================================
                                API FUNCTIONS
==============================================================================*/
/** @} */
