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
 * 	 \addtogroup emb6
 * 	 @{
 *   \addtogroup stack_API Stack API
 *   @{
*/
/*! \file   emb6.h

    \author Peter Lehmann peter.lehmann@hs-offenburg.de
            Phuong Nguyen minh.nguyen@hs-offenburg.de

    \brief  emb6 API

	\version 0.0.2
*/

#ifndef EMB6_H_
#define EMB6_H_

/*==============================================================================
                      	  	  INCLUDES
==============================================================================*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

/*
********************************************************************************
*                               VERSION DECLARATION
********************************************************************************
*/
#define EMB6_VERSION        000002UL          /* Version format Vx.yy.zz */


/*=============================================================================
                                BASIC CONSTANTS
==============================================================================*/
#undef  FALSE
#undef  TRUE
#define FALSE                               0 /* do not change                */
#define TRUE                                1 /* do not change                */
typedef void                                (*pfn_intCallb_t)(void *);

/* use only Exact-width integer types, linked with TMR_OVRFLOW_VAL 			  */
typedef uint32_t                            clock_time_t;
/* linked with clock_time_t, maximum value for clock_time_t variables	      */
#define TMR_OVRFLOW_VAL                     0xffffffff


/*=============================================================================
                                CONFIGURATIONS
==============================================================================*/
#include "emb6_conf.h"



/*==============================================================================
                                     MACROS
 =============================================================================*/
/** defines for the modulation type for RF transceiver */
#define MODULATION_QPSK100          0
#define MODULATION_BPSK20           1


/*==============================================================================
                        RPL Configuration
 =============================================================================*/

/*! \struct rpl_configuration
	\brief for dynamic RPL parameter configuration
*/
typedef struct rpl_configuration
{
    /* The DIO interval (n) represents 2^n ms. default value = 8 */
    uint8_t DIOintmin;
    /* Maximum amount of timer doublings. default value = 12 */
    uint8_t DIOintdoub;
    /* This value decides which DAG instance we should participate in by def.
     * default value = 0x1e (30) */
    uint8_t defInst;
    /* Initial metric attributed to a link when the ETX is unknown.
     * default value = 2 */
    uint8_t linkMetric;
    /* Default route lifetime unit. This is the granularity of time used in RPL
     * lifetime values, in seconds.
     * default value = 0xffff */
    uint16_t defRouteTimeUnit;
    /* Default route lifetime as a multiple of the lifetime unit.
     * default value = 0xff */
    uint8_t defRouteTime;
}s_rpl_conf_t;

/*! RPL configuration struct, do not change */
extern s_rpl_conf_t     rpl_config;

/*==============================================================================
                         MAC & PHY Parameter Configuration
 =============================================================================*/

/*! \struct mac_phy_configuration
*   \brief for initial mac_phy parameter configuration,
*   if changed during runtime RF-interface must be re-initialized
*/
typedef struct mac_phy_configuration
{
    /** MAC address, default value: { 0x00,0x50,0xc2,0xff,0xfe,0xa8,0xdd,0xdd}*/
    uint8_t mac_address[8];
    /** PAN ID, default value: 0xABCD */
    uint16_t pan_id;
    /** initial tx power, default value: 11 dBm */
    int8_t init_power;
    /** initial rx sensitivity, default value: -100 dBm */
    int8_t init_sensitivity;
    /** rf modulation type, default value BPSK20 */
    uint8_t modulation;
}s_mac_phy_conf_t;

/*! MAC configuration struct, do not change */
extern s_mac_phy_conf_t     mac_phy_config;

/*==============================================================================
                                     ENUMS
==============================================================================*/
/*! \enum e_radio_tx_status_t
    \brief Return code of an interface driver
*/
typedef enum {
    RADIO_TX_OK = 1,
    RADIO_TX_COLLISION = 2,
    RADIO_TX_NOACK = 3,
    RADIO_TX_ERR = 4
}e_radio_tx_status_t;

/*==============================================================================
                          SYSTEM STRUCTURES AND OTHER TYPEDEFS
 =============================================================================*/

//! We are using IEEE 802.15.4
#define UIP_CONF_LL_802154                  TRUE
#define UIP_CONF_LLH_LEN                    0UL
#define	PRINT_PCK_STAT                      FALSE
#define TIMESTAMP_PERIOD_SEC                10UL	// in sec

#ifdef LINKADDR_CONF_SIZE
#define LINKADDR_SIZE                       LINKADDR_CONF_SIZE
#else /* LINKADDR_SIZE */
#define LINKADDR_SIZE                       8UL
#endif /* LINKADDR_SIZE */


typedef union {
    unsigned char u8[LINKADDR_SIZE];
} linkaddr_t;

/** \brief 16 bit 802.15.4 address */
typedef struct uip_802154_shortaddr {
    uint8_t addr[2];
} uip_802154_shortaddr;
/** \brief 64 bit 802.15.4 address */
typedef struct uip_802154_longaddr {
    uint8_t addr[8];
} uip_802154_longaddr;

#if UIP_CONF_LL_802154
/** \brief 802.15.4 address */
typedef uip_802154_longaddr                 uip_lladdr_t;
#define UIP_802154_SHORTADDR_LEN            2UL
#define UIP_802154_LONGADDR_LEN             8UL
#define UIP_LLADDR_LEN                      UIP_802154_LONGADDR_LEN
#else /*UIP_CONF_LL_802154*/
#if UIP_CONF_LL_80211
/** \brief 802.11 address */
typedef uip_80211_addr uip_lladdr_t;
#define UIP_LLADDR_LEN 6
#else /*UIP_CONF_LL_80211*/
/** \brief Ethernet address */
typedef uip_eth_addr uip_lladdr_t;
#define UIP_LLADDR_LEN 6
#endif /*UIP_CONF_LL_80211*/
#endif /*UIP_CONF_LL_802154*/


/*==============================================================================
 *                                                                             *
 *                                NETSTACK                                     *
 *                                                                             *
 =============================================================================*/


/*
********************************************************************************
*                           ENUMERATION DECLARATIONS
********************************************************************************
*/
/**
 * @brief   Netstack error code enumeration declaration
 */
typedef enum netstk_err
{
    /*
     * Common error codes
     */
    NETSTK_ERR_NONE = 0U,
    NETSTK_ERR_INIT,
    NETSTK_ERR_BUSY,
    NETSTK_ERR_BUF_OVERFLOW,
    NETSTK_ERR_INVALID_ARGUMENT,
    NETSTK_ERR_INVALID_FRAME,
    NETSTK_ERR_TX_TIMEOUT,
    NETSTK_ERR_TX_NOACK,
    NETSTK_ERR_CMD_UNSUPPORTED,
    NETSTK_ERR_CHANNEL_ACESS_FAILURE,
    NETSTK_ERR_CRC,
    NETSTK_ERR_BAD_FORMAT,
    NETSTK_ERR_FATAL,

    /*
     * LLC error codes
     */
    NETSTK_ERR_LLC_XXX                      = 100U,

    /*
     * MAC error codes
     */
    NETSTK_ERR_MAC_XXX                      = 200U,
#if NETSTK_CFG_IEEE_802154_IGNACK
    NETSTK_ERR_MAC_ACKOFF,
#endif /* NETSTK_CFG_IEEE_802154_IGNACK */

    NETSTK_ERR_MAC_ULE_XXX                  = 250U,
    NETSTK_ERR_MAC_ULE_UNSUPPORTED_FRAME,
    NETSTK_ERR_MAC_ULE_LAST_STROBE,
    NETSTK_ERR_MAC_ULE_BROADCAST_NOACK,
    NETSTK_ERR_MAC_ULE_INVALID_ADDR,
    NETSTK_ERR_MAC_ULE_TX_COLLISION_SAME_DEST,
    NETSTK_ERR_MAC_ULE_TX_COLLISION_DIFF_DEST,
    NETSTK_ERR_MAC_ULE_INVALID_ACK,
    NETSTK_ERR_MAC_ULE_NO_STROBE,

    /*
     * PHY error codes
     */
    NETSTK_ERR_PHY_XXX                      = 300U,

    /*
     * RF error codes
     */
    NETSTK_ERR_RF_XXX                       = 400U,
    NETSTK_ERR_RF_SEND                      = 401U,

}e_nsErr_t;


/**
 * @brief   Netstack I/O Control command enumeration declaration
 */
typedef enum netstk_ioc_cmd
{
    /*
     * Common command codes
     */
    NETSTK_CMD_NONE = 0,            /*!< Reserved command code          */
    NETSTK_CMD_TX_CBFNCT_SET,       /*!< Set TX Callback function       */
    NETSTK_CMD_TX_CBARG_SET,        /*!< Set TX Callback argument       */
    NETSTK_CMD_RX_CBFNT_SET,        /*!< Set RX Callback function       */
    NETSTK_CMD_RX_BUF_READ,

    /*
     * DLLC command codes
     */
    NETSTK_CMD_DLLC_RSVD = 100U,

    /*
     * MAC command codes
     */
    NETSTK_CMD_MAC_RSVD = 200U,

    /*
     * PHY command codes
     */
    NETSTK_CMD_PHY_RSVD = 300U,
    NETSTK_CMD_PHY_LAST_PKT_TX,
    NETSTK_CMD_PHY_CRC_LEN_SET,

    /*
     * LPR command codes
     */
    NETSTK_CMD_LPR_RSVD = 400U,

    /*
     * RF command codes
     */
    NETSTK_CMD_RF_RSVD = 500U,      /*!< RF reserved command code       */
    NETSTK_CMD_RF_TXPOWER_SET,      /*!< Set TX Power                   */
    NETSTK_CMD_RF_TXPOWER_GET,      /*!< Get TX Power                   */
    NETSTK_CMD_RF_SENS_SET,         /*!< Set Sensitivity                */
    NETSTK_CMD_RF_SENS_GET,         /*!< Get Sensitivity                */
    NETSTK_CMD_RF_RSSI_GET,         /*!< Get RSSI                       */
    NETSTK_CMD_RF_CCA_GET,          /*!< Get CCA                        */
    NETSTK_CMD_RF_ANT_DIV_SET,      /*!< Set Antenna Div.               */
    NETSTK_CMD_RF_RF_SWITCH_SET,    /*!< */
    NETSTK_CMD_RF_IS_RX_BUSY,       /*!< Indicate if RF is in RX state  */
    NETSTK_CMD_RF_IS_TX_BUSY,       /*!< Indicate if RF is in TX state  */
    NETSTK_CMD_RF_SYNC_SET,         /*!< Set SYNC words                 */
    NETSTK_CMD_RF_SYNC_GET,         /*!< Get SYNC words                 */
    NETSTK_CMD_RF_PROMISC_SET,      /*!< Set promisc mode                */

    NETSTK_CMD_RF_802154G_EU_CHAN,  /*!< Channel center for IEEE802.15.4 in Europe  */
    NETSTK_CMD_RF_CHAN_NUM_SET,     /*!< Set operation channel number */
    NETSTK_CMD_RF_OP_MODE_SET,      /*!< Set operation mode */

    NETSTK_CMD_RF_WOR_EN,           /*!< Enable/Disable WOR mode */

}e_nsIocCmd_t;


/**
 * @brief   IEEE-802.15.4g operating mode supports
 */
typedef enum netstk_rf_operating_mode
{
    /*!< Common Signaling Mode (CSM) */
    NETSTK_RF_OP_MODE_CSM,

#if (NETSTK_CFG_PHY_OP_MODE_1_EN == TRUE)
    /*!< MR-FSK operating mode #1 */
    NETSTK_RF_OP_MODE_1,
#endif

#if (NETSTK_CFG_PHY_OP_MODE_2_EN == TRUE)
    /*!< MR-FSK operating mode #2 */
    NETSTK_RF_OP_MODE_2,
#endif

#if (NETSTK_CFG_PHY_OP_MODE_3_EN == TRUE)
    /*!< MR-FSK operating mode #3 */
    NETSTK_RF_OP_MODE_3,
#endif

    /*!< Invalid mode */
    NETSTK_RF_OP_MODE_MAX,

} e_nsRfOpMode;


#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
#define NETSTK_RF_IEEE802154G_CHAN_QTY_CSM                  34u
#define NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE1              34u
#define NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE2              17u
#define NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE3              17u
#endif


/*
********************************************************************************
*                            DATA TYPES DECLARATIONS
********************************************************************************
*/
typedef uint16_t        netstk_devid_t;

typedef void (* mac_callback_t)(void *ptr, int status, int transmissions);

typedef void (*nsTxCbFnct_t) (void *p_arg, e_nsErr_t *p_err);

typedef void (*nsRxCbFnct_t) (uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);


/*
********************************************************************************
*                           STRUCTURE DECLARATIONS
********************************************************************************
*/
typedef struct netstack                     s_ns_t;
typedef struct netstack_socket              s_nsSocket_t;
typedef struct netstack_headerCompression   s_nsHeadComp_t;
typedef struct netstack_dllsec              s_nsdllsec_t;
typedef struct netstack_framer              s_nsFramer_t;
typedef struct netstk_module_api            s_nsModuleDrv_t;

typedef s_nsModuleDrv_t   s_nsDLLC_t;
typedef s_nsModuleDrv_t   s_nsMAC_t;
typedef s_nsModuleDrv_t   s_nsPHY_t;
typedef s_nsModuleDrv_t   s_nsRF_t;


/**
 * @brief   Netstack driver structure declaration
 */
struct netstack
{
    const s_nsFramer_t      *frame;             /*!< Pointer to 6LoWPAN framing driver                  */
    const s_nsHeadComp_t    *hc;                /*!< Pointer to 6LoWPAN header compressor Driver        */
    const s_nsdllsec_t      *dllsec;            /*!< Pointer to Data Link Layer Security Driver         */
    const s_nsDLLC_t        *dllc;              /*!< Pointer to Data Link Layer Control Driver          */
    const s_nsMAC_t         *mac;               /*!< Pointer to Medium Access Control Driver            */
    const s_nsPHY_t         *phy;               /*!< Pointer to Physical Driver                         */
    const s_nsRF_t          *rf;                /*!< Pointer to Radio Frequency Driver                  */
    uint8_t                 c_configured;       /*!< Indicate whether netstack is already configured    */
};


/**
 * @brief   Netstack submodule driver structure declaration
 */
struct netstk_module_api
{
    char     *name;                                                             /*!< Driver name                    */
    void    (*init  )(void *p_netstk, e_nsErr_t *p_err);                        /*!< Initialization handler         */
    void    (*on    )(e_nsErr_t *p_err);                                        /*!< Turn the driver on             */
    void    (*off   )(e_nsErr_t *p_err);                                        /*!< Turn the driver on             */
    void    (*send  )(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);         /*!< Data transmission handler      */
    void    (*recv  )(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);         /*!< Data reception handler         */
    void    (*ioctrl)(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);         /*!< Miscellaneous functionality    */
};


/**
 * @brief   Netstack socket driver structure declaration
 */
struct netstack_socket
{
    char *name;

    /* Initialize the BSD socket driver */
    void (* create)(s_ns_t* p_ns);

    /* Connect to remote node*/
    void (* connect)(void);

    void (* bind)(void);

    /* Send data to remote node*/
    void (* send)(void);

    /* Send data to remote node*/
    void (* sendto)(void);

    /* Close the BSD socket driver */
    void (* close)(s_ns_t* p_ns);

};


struct netstack_headerCompression
{
    char *name;

    /** Initialize the network driver */
    void (* init)(s_ns_t* p_ns);

    /** Callback for getting notified of incoming packet. */
    void (* input)(void);
};


/**
 * The structure of a data link layer security driver.
 */
struct netstack_dllsec
{
    char *name;

    /** Initializes link layer security and thereafter starts upper layers. */
    void (* init)(s_ns_t* p_ns);

    /** Secures outgoing frames before passing them to NETSTACK_MAC. */
    void (* send)(mac_callback_t sent_callback, void *ptr);

    /**
     * Once the netstack_framer wrote the headers, the llsec driver
     * can generate a MIC over the entire frame.
     * \return Returns != 0 <-> success
     */
    int (* on_frame_created)(void);

    /**
     * Decrypts incoming frames;
     * filters out injected or replayed frames.
     */
    void (* input)(void);

    /** Returns the security-related overhead per frame in bytes */
    uint8_t (* get_overhead)(void);
};

struct netstack_framer
{
    char *name;

    int8_t (* init)(s_ns_t* p_ns);

    int8_t (* length)(void);

    int8_t (* create)(void);

    int8_t (* create_and_secure)(s_ns_t* p_ns);

    int8_t (* parse)(void);
};


/*
********************************************************************************
*                           SOCKET DRIVERS DECLARATIONS
********************************************************************************
*/
/*! Supported BSD-like socket interface */
/*extern const s_nsSocket_t udp_socket_driver;
extern const s_nsSocket_t tcp_socket_driver;*/


/*
********************************************************************************
*                       HEADER COMPRESSOR DRIVERS DECLARATIONS
********************************************************************************
*/
/*! Supported headers compression handlers */
extern const s_nsHeadComp_t     hc_driver_sicslowpan;


/*! This driver are pretending to be a hc layer
 *  for sniffing data and sending them via USART     */
extern const s_nsHeadComp_t     hc_driver_slipnet;


/*
********************************************************************************
*                           LLSEC DRIVERS DECLARATIONS
********************************************************************************
*/
/*! Supported link layer security handlers */
extern const s_nsdllsec_t       dllsec_driver_null;


/*
********************************************************************************
*                   6LOWPAN FRAMER DRIVERS DECLARATIONS
********************************************************************************
*/
/*! Supported framers */
extern const s_nsFramer_t       framer_802154;
extern const s_nsFramer_t       framer_noframer;
extern const s_nsFramer_t       framer_nullframer;


/*
********************************************************************************
*                           DLLC DRIVERS DECLARATIONS
********************************************************************************
*/
extern  const s_nsDLLC_t        dllc_driver_null;
extern  const s_nsDLLC_t        dllc_driver_802154;

/*
********************************************************************************
*                           MAC DRIVERS DECLARATIONS
********************************************************************************
*/
extern  const s_nsMAC_t         mac_driver_null;
extern  const s_nsMAC_t         mac_driver_802154;
extern  const s_nsMAC_t         mac_driver_ule;

/*
********************************************************************************
*                           PHY DRIVERS DECLARATIONS
********************************************************************************
*/
extern  const s_nsPHY_t         phy_driver_null;
extern  const s_nsPHY_t         phy_driver_802154;


/*
********************************************************************************
*                           RF DRIVERS DECLARATIONS
********************************************************************************
*/

extern  const s_nsRF_t          rf_driver_null;
extern  const s_nsRF_t          rf_driver_native;
extern  const s_nsRF_t          rf_driver_at212;
extern  const s_nsRF_t          rf_driver_at212b;
extern  const s_nsRF_t          rf_driver_ticc112x;
extern  const s_nsRF_t          rf_driver_ticc120x;

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/*============================================================================*/
/*!
\brief   initialize all stack functions

    This function inits the board support packet and the complete netstack

\param  p_err   a variable to store returned error code
\return none
*/
/*============================================================================*/
void emb6_init(s_ns_t* pst_netStack, e_nsErr_t *p_err);

/*============================================================================*/
/*!
\brief   emb6 process function

    This function handles all events and timers of the emb6 stack in a loop

\param 	 delay sets a delay in µs at the end of the function

\return  none

*/
/*============================================================================*/
void emb6_process(uint16_t delay);

/*============================================================================*/
/*!
\brief   Function which assign a given pointer to a current network stack ptr


\return  pointer to a stack

*/
/*============================================================================*/
s_ns_t * emb6_get(void);

/*============================================================================*/
/*!
\brief   emb6 error handler function

\return  pointer to returned error code

*/
/*============================================================================*/
void emb6_errorHandler(e_nsErr_t *p_err);

/*=============================================================================
                                	UTILS SECTION
==============================================================================*/
// Used in several files.
#define CCIF
#define CLIF

#ifndef	QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                   4
#endif

#ifndef QUEUEBUF_CONF_REF_NUM
#define QUEUEBUF_CONF_REF_NUM               4
#endif

#endif /* EMB6_H_ */

/** @} */
/** @} */
