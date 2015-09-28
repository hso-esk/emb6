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
 * \addtogroup native_radio
 * @{
 */
/*============================================================================*/
/*! \file   native.c

 \author Artem Yushev 

 \brief  Fake radio transceiver based on LCM IPC.

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
#define     FAKERADIO_CHANNEL         "EMB6_FAKERADIO"
#define     __ADDRLEN__    2
/*==============================================================================
 ENUMS
 ==============================================================================*/

/*==============================================================================
 VARIABLE DECLARATIONS
 ==============================================================================*/
static struct etimer tmr;
/* Pointer to the lmac structure */
static s_nsLowMac_t* p_lmac = NULL;
extern uip_lladdr_t uip_lladdr;
static lcm_t* ps_lcm;
/*==============================================================================
 GLOBAL CONSTANTS
 ==============================================================================*/
/*==============================================================================
 LOCAL FUNCTION PROTOTYPES
 ==============================================================================*/
/* Radio transceiver local functions */
static void _printAndExit( const char* rpc_reason );
static int8_t _native_on( void );
static int8_t _native_off( void );
static int8_t _native_init( s_ns_t* p_netStack );
static int8_t _native_send( const void *pr_payload, uint8_t c_len );
static void _native_read( const lcm_recv_buf_t *rbuf, const char * channel,
        void * p_macAddr );
static void _native_handler( c_event_t c_event, p_data_t p_data );
/*==============================================================================
 STRUCTURES AND OTHER TYPEDEFS
 ==============================================================================*/
const s_nsIf_t native_driver = {
        "native_driver",
        _native_init,
        _native_send,
        _native_on,
        _native_off, };
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
static void _printAndExit( const char* rpc_reason )
{
    fputs( strerror( errno ), stderr );
    fputs( ": ", stderr );
    fputs( rpc_reason, stderr );
    fputc( '\n', stderr );
    exit( 1 );
}

#ifdef __NATIVEROLE_BRSERVER__
/*----------------------------------------------------------------------------*/
/** \brief  NATIVE initialization for IPC
 *
 *  \param  p_netStack    Pointer to s network stack.
 *  \return int8_t        Status code.
 */
/*----------------------------------------------------------------------------*/
static int8_t _native_init(s_ns_t* p_netStack)
{
    linkaddr_t un_addr;
    uint8_t l_error;
    /* Please refer to lcm_create() reference. Deafult parameter is taken 
     from there */
    ps_lcm = lcm_create(NULL);

    if(!ps_lcm)
        _printAndExit("LCM init failed");

    LOG_INFO("%s\n\r","native driver was initialized");

    /* Mac address should not be NULL pointer, although it can't be, but still
     *  */
    if (mac_phy_config.mac_address == NULL)
    {
        _printAndExit("MAC address is NULL");
    }

    lcm_subscribe (ps_lcm, FAKERADIO_CHANNEL, _native_read, NULL);

    /* Initialise global lladdr structure with a given mac */
    memcpy((void *)&un_addr.u8, &mac_phy_config.mac_address, 8);
    memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
    linkaddr_set_node_addr(&un_addr);

    LOG_INFO( "MAC address %x:%x:%x:%x:%x:%x:%x:%x", un_addr.u8[0],
              un_addr.u8[1], un_addr.u8[2], un_addr.u8[3], un_addr.u8[4],
              un_addr.u8[5], un_addr.u8[6], un_addr.u8[7] );

    /* Low mac layer should be ok, not NULL */
    if (p_netStack->lmac != NULL)
    {
        p_lmac = p_netStack->lmac;
        l_error = 1;
    }
    else
    {
        _printAndExit("Bad lmac pointer");
    }

    /* Start the packet receive process */
    etimer_set(&tmr, 10, _native_handler);
    LOG1_INFO("set %p for %p callback\n\r",&tmr, &_native_handler);
    LOG_WARN("Don't forget to destroy lcm object!");
    return l_error;
} /* _native_init() */
#else
/*----------------------------------------------------------------------------*/
/** \brief  NATIVE transport initialization for IPC
 *
 *  \param  p_netStack    Pointer to s network stack.
 *  \return int8_t        Status code.
 */
/*----------------------------------------------------------------------------*/
static int8_t _native_init( s_ns_t* p_netStack )
{
    linkaddr_t un_addr;
    uint8_t l_error;
    LOG_INFO( "Try to initialize Broadcasting Client for native radio driver" );

    /* Please refer to lcm_create() reference. Deafult parameter is taken
     from there */
    ps_lcm = lcm_create( NULL );

    if( !ps_lcm )
        _printAndExit("LCM init failed");

    lcm_subscribe( ps_lcm, FAKERADIO_CHANNEL, _native_read, NULL );

    LOG_INFO( "native driver was initialized" );

    /* Mac address should not be NULL pointer, although it can't be, but still
     *  */
    if( mac_phy_config.mac_address == NULL )
    {
        _printAndExit( "MAC address is NULL" );
    }

    /* Initialise global lladdr structure with a given mac */
    memcpy( (void *)&un_addr.u8, &mac_phy_config.mac_address, 8 );
    memcpy( &uip_lladdr.addr, &un_addr.u8, 8 );
    linkaddr_set_node_addr( &un_addr );

    LOG_INFO( "MAC address %x:%x:%x:%x:%x:%x:%x:%x", un_addr.u8[0],
            un_addr.u8[1], un_addr.u8[2], un_addr.u8[3], un_addr.u8[4],
            un_addr.u8[5], un_addr.u8[6], un_addr.u8[7] );

    if( p_netStack->lmac != NULL )
    {
        p_lmac = p_netStack->lmac;
        l_error = 1;
    }
    else
    {
        _printAndExit( "Bad lmac pointer" );
    }

    /* Start the packet receive process */
    etimer_set( &tmr, 10, _native_handler );

    return l_error;
} /* _native_init() */
#endif

/*----------------------------------------------------------------------------*/
/** \brief  NATIVE transport message send
 *          Idea behind is that we concatenate to payload last two bytes of a
 *          MAC address in order to filter our own packets on a receiving side
 *  \param  pr_payload    Pointer to a payload.
 *  \param  c_len         Length of a payload
 *  \return int8_t        Status code.
 */
/*----------------------------------------------------------------------------*/
static int8_t _native_send( const void *pr_payload, uint8_t c_len )
{
    int status;
    /* We assume that usage of malloc here is ok as it's a simple Linux port
     * However in the end we do a free() call. Function description expains
     * why we need to allocate extra 2 bytes here
     */
    uint8_t* pc_frame = malloc( sizeof(uint8_t) * ( c_len + __ADDRLEN__ ) );

    /* Check malloc result */
    if( pc_frame == NULL )
    {
        LOG_ERR( "Insufficient memory" );
        return RADIO_TX_ERR;
    }

    /* Add two last bytes of MAC address to the beginning of a frame */
    *pc_frame = uip_lladdr.addr[6];
    *( pc_frame + 1 ) = uip_lladdr.addr[7];
    memmove( pc_frame + __ADDRLEN__, pr_payload, c_len );
    status = lcm_publish( ps_lcm, FAKERADIO_CHANNEL, pc_frame,
                          c_len + __ADDRLEN__ );

    /* Free allocated memmory */
    free( pc_frame );

    /* Return execution status to a caller */
    if( status == -1 )
    {
        LOG_ERR( "Send packet failed" );
        return RADIO_TX_ERR;
    }
    else
    {
        LOG_OK( "TX packet [%d]", c_len );
        LOG2_HEXDUMP( pr_payload, c_len );
        return RADIO_TX_OK;
    }
} /* _native_send() */

/*----------------------------------------------------------------------------*/
/** \brief  NATIVE transport message reception
 *          Idea behind is that we concatenate to payload last two bytes of a
 *          MAC address in order to filter our own packets on a receiving side
 *  \param  rps_rbuf      Pointer to a payload.
 *  \param  rpc_channel   Reception channel
 *  \param  userdata      Not used.
 *  \return void
 */
/*----------------------------------------------------------------------------*/
static void _native_read( const lcm_recv_buf_t *rps_rbuf,
        const char * rpc_channel, void * userdata )
{
    uint16_t i_dSize = rps_rbuf->data_size;

    /* Clear buffer where to store received payload */
    packetbuf_clear();

    /* Check whether recieved packet is not too long */
    if( i_dSize > PACKETBUF_SIZE )
    {
        LOG_ERR( "Received packet too long" );
    }
    else
    {
        /* If first two bytes of the received frame match with our
         * MAC address then discard a frame
         */
        if( ( *( (uint8_t *)rps_rbuf->data )     != uip_lladdr.addr[6] ) ||
            ( *( (uint8_t *)rps_rbuf->data + 1 ) != uip_lladdr.addr[7] ) )
        {
            memcpy( packetbuf_dataptr(), rps_rbuf->data + __ADDRLEN__,
                    i_dSize - __ADDRLEN__ );
            LOG_OK( "RX packet [%d] from 0x%02X:0x%02X", i_dSize,
                    *(uint8_t * )rps_rbuf->data,
                    *( (uint8_t * )rps_rbuf->data + 1 ) );
            LOG2_HEXDUMP( packetbuf_dataptr(), i_dSize - __ADDRLEN__ );
            if( ( rps_rbuf->data_size > 0 ) && ( p_lmac != NULL ) )
            {
                packetbuf_set_datalen( i_dSize - __ADDRLEN__ );
                p_lmac->input();
            }
            else
            {
                LOG_ERR( "Failed to receive packet" );
            }
        }
    }
} /* _native_read() */

/*----------------------------------------------------------------------------*/
/** \brief  NATIVE transport wrapper function
 *  \return 0
 */
/*----------------------------------------------------------------------------*/
static int8_t _native_on( void )
{
    return 0;
} /* _native_on() */

/*----------------------------------------------------------------------------*/
/** \brief  NATIVE transport wrapper function.
 *          lcm structure destroy can be invoked here
 *  \return 0
 */
/*----------------------------------------------------------------------------*/
static int8_t _native_off( void )
{
    return 0;
} /* _native_off() */

/*----------------------------------------------------------------------------*/
/** \brief  NATIVE transport handler for periodic polling
 *          triggered every 10 msec
 *  \param  c_event       Source of an event.
 *  \param  p_data        Pointer to a data
 *  \return void
 */
/*----------------------------------------------------------------------------*/
static void _native_handler( c_event_t c_event, p_data_t p_data )
{
    int32_t lcm_fd;
    struct timeval s_tv;
    s_tv.tv_sec = 0;
    s_tv.tv_usec = 10;
    fd_set fds;

    if( etimer_expired( &tmr ) )
    {
        /* We can't use lcm_handle trigger every time, as
         * it's a blocking operation. We should instead check whether a lcm file
         * descriptor is available for reading. We put 10 usec as a timeout.
         */
        lcm_fd = lcm_get_fileno( ps_lcm );
        FD_ZERO( &fds );
        FD_SET( lcm_fd, &fds );

        select( lcm_fd + 1, &fds, 0, 0, &s_tv );
        /* If descriptor is available for reading then there is a incoming data
         * to read.
         */
        if( FD_ISSET( lcm_fd, &fds ) )
        {
            lcm_handle( ps_lcm );
        }

        /* Restart a timer anyway. */
        etimer_restart( &tmr );

    }
}

/*==============================================================================
 API FUNCTIONS
 ==============================================================================*/
/** @} */
