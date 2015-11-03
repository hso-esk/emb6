/*
l * emb6 is licensed under the 3-clause BSD license. This license gives everyone
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
         Phuong Nguyen

 \brief  Fake radio transceiver based on LCM IPC.

 \version 1.02.01
 */
/*============================================================================*/


/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"


#ifdef NETSTK_CFG_RF_NATIVE_EN
#include "bsp.h"
#include "packetbuf.h"
#include "tcpip.h"
#include "etimer.h"
#include <errno.h>
#include <sys/time.h>
#include <lcm/lcm.h>

/*
********************************************************************************
*                                   VERSION
********************************************************************************
*/
#define NATIVE_RF_VERSION       0x10201UL     /* Version Vx.yy.zz */

/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define     LOGGER_ENABLE             LOGGER_RADIO
#include    "logger.h"
#define     FAKERADIO_CHANNEL         "EMB6_FAKERADIO"
#define     __ADDRLEN__    2


/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void RF_Init (void *p_netstk, e_nsErr_t *p_err);
static void RF_On (e_nsErr_t *p_err);
static void RF_Off (e_nsErr_t *p_err);
static void RF_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void RF_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err);
static void RF_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);
static void RF_EventHandler(c_event_t c_event, p_data_t p_data);

static void RF_Read(const lcm_recv_buf_t *rps_rbuf, const char *rpc_channel, void *userdata );
static void RF_PrintAndExit(const char *rpc_reason);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static struct etimer RF_EventTmr;

static lcm_t    *RF_LCM;
static s_ns_t   *RF_Netstack;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
extern uip_lladdr_t uip_lladdr;


const s_nsRF_t RFDrvNative =
{
   "RF Native",
    RF_Init,
    RF_On,
    RF_Off,
    RF_Send,
    RF_Recv,
    RF_Ioctl,
};


/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/

/** \brief  This function reports the error and exits back to the shell.
 *
 *  \param  rpc_reason  Error to show
 *  \return Node
 */
/*----------------------------------------------------------------------------*/
static void RF_PrintAndExit(const char *rpc_reason)
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
static void RF_Init (void *p_netstk, e_nsErr_t *p_err)
{
    linkaddr_t un_addr;
    uint8_t l_error;


    /*
     * Store pointer to netstack structure
     */
    RF_Netstack = (s_ns_t *)p_netstk;
    if (RF_Netstack->phy == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        RF_PrintAndExit("Bad PHY driver pointer");
    }


    /*
     * Please refer to lcm_create() reference. Default parameter is taken
     * from there */
    RF_LCM = lcm_create(NULL);
    if (!RF_LCM) {
        RF_PrintAndExit("LCM init failed");
    }

    LOG_INFO("%s\n\r", "native driver was initialized");

    /*
     * MAC address should not be NULL pointer, although it can't be, but still
     */
    if (mac_phy_config.mac_address == NULL) {
        RF_PrintAndExit("MAC address is NULL");
    }

    lcm_subscribe(RF_LCM,
                  FAKERADIO_CHANNEL,
                  RF_Read,
                  NULL);

    /* Initialize global logical link address structure with a given MAC */
    memcpy((void *) &un_addr.u8, &mac_phy_config.mac_address, 8);
    memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
    linkaddr_set_node_addr(&un_addr);

    LOG_INFO("MAC address %x:%x:%x:%x:%x:%x:%x:%x", un_addr.u8[0],
            un_addr.u8[1], un_addr.u8[2], un_addr.u8[3], un_addr.u8[4],
            un_addr.u8[5], un_addr.u8[6], un_addr.u8[7]);

    /*
     * Start the packet receive process
     */
    etimer_set(&RF_EventTmr,
                10,
                RF_EventHandler);

    LOG1_INFO("set %p for %p callback\n\r", &RF_EventTmr, &RF_EventHandler);
    LOG_WARN("Don't forget to destroy lcm object!");
}
#else

/*----------------------------------------------------------------------------*/
/** \brief  NATIVE transport initialization for IPC
 *
 *  \param  p_netStack    Pointer to s network stack.
 *  \return int8_t        Status code.
 */
/*----------------------------------------------------------------------------*/
static void RF_Init (void *p_netstk, e_nsErr_t *p_err)
{
    linkaddr_t un_addr;


#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }

    if (p_netstk == NULL) {
        *p_err = NETSTK_ERR_INIT;
        return;
    }
#endif


    /*
     * Store pointer to netstack structure
     */
    RF_Netstack = (s_ns_t *)p_netstk;
    if (RF_Netstack->phy == NULL) {
        *p_err = NETSTK_ERR_INVALID_ARGUMENT;
        RF_PrintAndExit("Bad PHY driver pointer");
    }

    LOG_INFO("Try to initialize Broadcasting Client for native radio driver");

    /*
     * Please refer to lcm_create() reference. Default parameter is taken
     * from there
     */

    /*
     * Instantiate a LCM object
     */
    RF_LCM = lcm_create(NULL);
    if (!RF_LCM) {
        RF_PrintAndExit("LCM init failed");
        return;
    }

    /*
     * Subscribe to a channel
     */
    lcm_subscribe(RF_LCM,
                  FAKERADIO_CHANNEL,
                  RF_Read,
                  NULL);
    LOG_INFO("native driver was initialized");

    /*
     * Mac address should not be NULL pointer, although it can't be, but still
     */
    if (mac_phy_config.mac_address == NULL) {
        RF_PrintAndExit("MAC address is NULL");
        return;
    }

    /*
     * Initialize global logical link address structure with a given MAC
     */
    memcpy((void *)&un_addr.u8, &mac_phy_config.mac_address, 8);
    memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
    linkaddr_set_node_addr(&un_addr);

    LOG_INFO("MAC address %x:%x:%x:%x:%x:%x:%x:%x", un_addr.u8[0],
            un_addr.u8[1], un_addr.u8[2], un_addr.u8[3], un_addr.u8[4],
            un_addr.u8[5], un_addr.u8[6], un_addr.u8[7]);


    /*
     * Start the packet receive process
     */
    etimer_set(&RF_EventTmr,
                10,
                RF_EventHandler);

    /* set returned error code */
    *p_err = NETSTK_ERR_NONE;
}
#endif


/** \brief  NATIVE transport message send
 *          Idea behind is that we concatenate to payload last two bytes of a
 *          MAC address in order to filter our own packets on a receiving side
 *  \param  pr_payload    Pointer to a payload.
 *  \param  c_len         Length of a payload
 *  \return int8_t        Status code.
 */
static void RF_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
    int status;
    uint8_t *pc_frame;

    /*
     * We assume that usage of malloc here is ok as it's a simple Linux port
     * However in the end we do a free() call. Function description expains
     * why we need to allocate extra 2 bytes here
     */
    pc_frame = malloc(sizeof(uint8_t) * (len + __ADDRLEN__));

    /* Check malloc result */
    if (pc_frame == NULL) {
        LOG_ERR("Insufficient memory");
        *p_err = NETSTK_ERR_BUF_OVERFLOW;
    }

    /* Add two last bytes of MAC address to the beginning of a frame */
    *pc_frame = uip_lladdr.addr[6];
    *(pc_frame + 1) = uip_lladdr.addr[7];
    memmove(pc_frame + __ADDRLEN__, p_data, len);
    status = lcm_publish(RF_LCM,
                         FAKERADIO_CHANNEL,
                         pc_frame,
                         len + __ADDRLEN__);

    /*
     * Free allocated memory
     */
    free(pc_frame);

    /*
     * Return execution status to a caller
     */
    if (status == -1) {
        LOG_ERR("Send packet failed");
        *p_err = NETSTK_ERR_FATAL;
    } else {
        LOG_OK("TX packet [%d]", len);
        LOG2_HEXDUMP(p_data, len);
        *p_err = NETSTK_ERR_NONE;
    }
}


/** \brief  NATIVE transport message reception
 *          Idea behind is that we concatenate to payload last two bytes of a
 *          MAC address in order to filter our own packets on a receiving side
 *  \param  rps_rbuf      Pointer to a payload.
 *  \param  rpc_channel   Reception channel
 *  \param  userdata      Not used.
 *  \return void
 */
static void RF_Read(const lcm_recv_buf_t *rps_rbuf,
                    const char           *rpc_channel,
                    void                 *userdata )
{
    e_nsErr_t   err = NETSTK_ERR_NONE;
    uint16_t    i_dSize = rps_rbuf->data_size;
    uint8_t    *p_data;
    uint16_t    len;

    /*
     * Clear buffer where to store received payload
     */
    packetbuf_clear();

    /*
     * Check whether recieved packet is not too long
     */
    if (i_dSize > PACKETBUF_SIZE) {
        LOG_ERR("Received packet too long");
    } else {
        /*
         * If first two bytes of the received frame match with our
         * MAC address then discard a frame
         */
        if ((*((uint8_t *)rps_rbuf->data    ) != uip_lladdr.addr[6]) ||
            (*((uint8_t *)rps_rbuf->data + 1) != uip_lladdr.addr[7])) {

            /*
             * Logging
             */
            LOG_OK("RX packet [%d] from 0x%02X:0x%02X", i_dSize,
                    *(uint8_t * )rps_rbuf->data,
                    *((uint8_t * )rps_rbuf->data + 1));
            LOG2_HEXDUMP(packetbuf_dataptr(), i_dSize - __ADDRLEN__);

            /*
             * Signal the next higher layer of the received frame
             */
            len    = i_dSize - __ADDRLEN__;
            p_data = rps_rbuf->data + __ADDRLEN__;
            RF_Netstack->phy->recv(p_data, len, &err);
        }
    }
}


/** \brief  NATIVE transport wrapper function
 *  \return 0
 */
static void RF_On (e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    *p_err = NETSTK_ERR_NONE;
}


/** \brief  NATIVE transport wrapper function.
 *          lcm structure destroy can be invoked here
 *  \return 0
 */
static void RF_Off (e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    *p_err = NETSTK_ERR_NONE;
}


/** \brief  NATIVE transport handler for periodic polling
 *          triggered every 10 msec
 *  \param  c_event       Source of an event.
 *  \param  p_data        Pointer to a data
 *  \return void
 */
static void RF_EventHandler( c_event_t c_event, p_data_t p_data )
{
    int32_t lcm_fd;
    struct timeval s_tv;
    s_tv.tv_sec = 0;
    s_tv.tv_usec = 10;
    fd_set fds;


    if (etimer_expired(&RF_EventTmr)) {
        /*
         * We can't use lcm_handle trigger every time, as
         * it's a blocking operation. We should instead check whether a lcm file
         * descriptor is available for reading. We put 10 usec as a timeout.
         */
        lcm_fd = lcm_get_fileno(RF_LCM);
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);

        select(lcm_fd + 1, &fds, 0, 0, &s_tv);
        /* If descriptor is available for reading then there is a incoming data
         * to read.
         */
        if (FD_ISSET(lcm_fd, &fds)) {
            lcm_handle(RF_LCM);
        }

        /* Restart a timer anyway. */
        etimer_restart(&RF_EventTmr);
    }
}


static void RF_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err)
{
#ifdef NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    /* default returned error code for fake radio transceiver */
    *p_err = NETSTK_ERR_NONE;
}


static void RF_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
#ifdef NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    /* default returned error code for fake radio transceiver */
    *p_err = NETSTK_ERR_NONE;
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* NETSTK_CFG_RF_NATIVE_EN */

/** @} */
