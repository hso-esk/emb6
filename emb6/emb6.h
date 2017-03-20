/*
 * --- License --------------------------------------------------------------*
 */
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
 * Copyright (c) 2016,
 * Hochschule Offenburg, University of Applied Sciences
 * Institute of reliable Embedded Systems and Communications Electronics.
 * All rights reserved.
 */

/*
 *  --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       emb6.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Interface description for the main emb6 interface.
 *
 *              This files provides the main interface for emb6. This includes
 *              the according functions to run and configure emb6.
 */
#ifndef __EMB6_H__
#define __EMB6_H__


/*
 *  --- Includes -------------------------------------------------------------*
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>


/*
 * --- Macro Definitions --------------------------------------------------- *
 */

#define CCIF
#define CLIF

/** emb::6 version */
#define EMB6_VERSION                                        000002UL

/* define TRUE and FALSE constants */
#undef  FALSE
#undef  TRUE
#define FALSE                                               0
#define TRUE                                                1

/* use only Exact-width integer types, linked with TMR_OVRFLOW_VAL */
typedef uint32_t                            clock_time_t;
/* linked with clock_time_t, maximum value for clock_time_t variables	 */
#define TMR_OVRFLOW_VAL                     0xffffffff

/** defines for the modulation type for RF transceiver */
#define MODULATION_QPSK100                                  0
#define MODULATION_BPSK20                                   1
#define MODULATION_2FSK50                                   2

/* Defines the number of IEEE802.15.4g channels for the according
 * operation modes.
 */
#define NETSTK_RF_IEEE802154G_CHAN_QTY_CSM                  34u
#define NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE1              34u
#define NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE2              17u
#define NETSTK_RF_IEEE802154G_CHAN_QTY_OPMODE3              17u

#define UIP_CONF_LL_802154                                  TRUE
#define PRINT_PCK_STAT                                      FALSE
#define TIMESTAMP_PERIOD_SEC                                10UL  // in sec

#ifdef LINKADDR_CONF_SIZE
#define LINKADDR_SIZE                                       LINKADDR_CONF_SIZE
#else /* LINKADDR_SIZE */
#define LINKADDR_SIZE                                       8UL
#endif /* LINKADDR_SIZE */

/* Address lenghts */
#define UIP_802154_SHORTADDR_LEN                            2UL
#define UIP_802154_LONGADDR_LEN                             8UL
#define UIP_LLADDR_LEN                                      UIP_802154_LONGADDR_LEN


/* include configuration */
#include "emb6assert.h"
#include "emb6_conf.h"

/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/**
 * \brief Status of the stack.
 */
typedef enum
{
    /** Stack is waiting for initialization. */
    STACK_STATUS_INIT_WAIT,

    /** Stack is in idle mode. */
    STACK_STATUS_IDLE,

    /* stack is active and operating */
    STACK_STATUS_ACTIVE,

    /* stack is connected to a network */
    STACK_STATUS_NETWORK,

    /* stack is an error state */
    STACK_STATUS_ERROR,

} e_stack_status_t;

/**
 * \brief Return code of an interface driver
 */
typedef enum
{
    RADIO_TX_OK = 1,
    RADIO_TX_COLLISION = 2,
    RADIO_TX_NOACK = 3,
    RADIO_TX_ERR = 4

} e_radio_tx_status_t;


/**
 * \brief   Netstack error code enumeration declaration
 */
typedef enum
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
    NETSTK_ERR_TX_COLLISION,
    NETSTK_ERR_CMD_UNSUPPORTED,
    NETSTK_ERR_CHANNEL_ACESS_FAILURE,
    NETSTK_ERR_CRC,
    NETSTK_ERR_BAD_FORMAT,
    NETSTK_ERR_INVALID_ADDRESS,
    NETSTK_ERR_FATAL,

    /*
     * LLC error codes
     */
    NETSTK_ERR_LLC_XXX                      = 100U,

    /*
     * MAC error codes
     */
    NETSTK_ERR_MAC_XXX                      = 200U,
    NETSTK_ERR_MAC_ULE_XXX                  = 250U,
    NETSTK_ERR_MAC_ULE_UNSUPPORTED_FRAME,
    NETSTK_ERR_MAC_ULE_LAST_STROBE,
    NETSTK_ERR_MAC_ULE_BROADCAST_NOACK,
    NETSTK_ERR_MAC_ULE_INVALID_ADDR,
    NETSTK_ERR_MAC_ULE_TX_COLLISION_SAME_DEST,
    NETSTK_ERR_MAC_ULE_TX_COLLISION_DIFF_DEST,
    NETSTK_ERR_MAC_ULE_INVALID_ACK,
    NETSTK_ERR_MAC_ULE_NO_STROBE,
    NETSTK_ERR_MAC_ULE_NO_STROBE_ACK,
    NETSTK_ERR_MAC_ULE_NO_DATA,

    /*
     * PHY error codes
     */
    NETSTK_ERR_PHY_XXX                      = 300U,

    /*
     * RF error codes
     */
    NETSTK_ERR_RF_XXX                       = 400U,
    NETSTK_ERR_RF_SEND                      = 401U,

} e_nsErr_t;


/**
 * \brief   Netstack I/O Control command enumeration declaration
 */
typedef enum
{
    /*
     * Common command codes
     */
    NETSTK_CMD_NONE = 0,
    NETSTK_CMD_TX_CBFNCT_SET,
    NETSTK_CMD_TX_CBARG_SET,
    NETSTK_CMD_RX_CBFNT_SET,
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


    /*
     * LPR command codes
     */
    NETSTK_CMD_LPR_RSVD = 400U,

    /*
     * RF command codes
     */
    NETSTK_CMD_RF_RSVD = 500U,
    NETSTK_CMD_RF_TXPOWER_SET,
    NETSTK_CMD_RF_TXPOWER_GET,
    NETSTK_CMD_RF_SENS_SET,
    NETSTK_CMD_RF_SENS_GET,
    NETSTK_CMD_RF_RSSI_GET,
    NETSTK_CMD_RF_CCA_GET,
    NETSTK_CMD_RF_ANT_DIV_SET,
    NETSTK_CMD_RF_RF_SWITCH_SET,
    NETSTK_CMD_RF_IS_RX_BUSY,
    NETSTK_CMD_RF_IS_TX_BUSY,
    NETSTK_CMD_RF_SYNC_SET,
    NETSTK_CMD_RF_SYNC_GET,
    NETSTK_CMD_RF_PROMISC_SET,
    NETSTK_CMD_RF_RETX,
    NETSTK_CMD_RF_CHAN_NUM_SET,
    NETSTK_CMD_RF_OP_MODE_SET,
    NETSTK_CMD_RF_WOR_EN,

} e_nsIocCmd_t;


/**
 * \brief   IEEE-802.15.4g operating mode supports
 */
typedef enum
{
    /** Common Signaling Mode (CSM) */
    NETSTK_RF_OP_MODE_CSM,

#if (NETSTK_CFG_PHY_OP_MODE_1_EN == TRUE)
    /** MR-FSK operating mode #1 */
    NETSTK_RF_OP_MODE_1,
#endif

#if (NETSTK_CFG_PHY_OP_MODE_2_EN == TRUE)
    /** MR-FSK operating mode #2 */
    NETSTK_RF_OP_MODE_2,
#endif

#if (NETSTK_CFG_PHY_OP_MODE_3_EN == TRUE)
    /** MR-FSK operating mode #3 */
    NETSTK_RF_OP_MODE_3,
#endif

    /** Invalid mode */
    NETSTK_RF_OP_MODE_MAX,

} e_nsRfOpMode;


/**
 * \brief Dynamic RPL parameter configuration
 */
typedef struct rpl_configuration
{
    /** The DIO interval (n) represents 2^n ms. default value = 8 */
    uint8_t DIOintmin;

    /** Maximum amount of timer doublings. default value = 12 */
    uint8_t DIOintdoub;

    /** This value decides which DAG instance we should participate in by def.
      * default value = 0x1e (30) */
    uint8_t defInst;

    /** Initial metric attributed to a link when the ETX is unknown.
      * default value = 2 */
    uint8_t linkMetric;

    /** Default route lifetime unit. This is the granularity of time used in RPL
      * lifetime values, in seconds.
      * default value = 0xffff */
    uint16_t defRouteTimeUnit;

    /** Default route lifetime as a multiple of the lifetime unit.
      * default value = 0xff */
    uint8_t defRouteTime;

}s_rpl_conf_t;


/**
 * \brief Dynamic PHY/MAC configuration
 */
typedef struct
{
    /** Indicates if the configuration is already saved i.e., in EEPROM */
    uint8_t is_saved;

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

    /** frame check sequence length (CRC size) */
    uint8_t fcs_len;

    /** IEEE Std. 802.15.4g operation channel number */
    uint8_t chan_num;

    /** IEEE Std. 802.15.4g operating mode */
    e_nsRfOpMode op_mode;

    /** Preamble length in octets */
    uint8_t preamble_len;

#if (NETSTK_CFG_LOW_POWER_MODE_EN == TRUE)
    /** sleep period in Low-Power mode */
    uint32_t sleepTimeout;
#endif

} s_mac_phy_conf_t;


/**
 * \brief Describes the link layer address.
 */
typedef union
{
    /** link layer address */
    unsigned char u8[LINKADDR_SIZE];

} linkaddr_t;

/**
 * \brief 16 bit 802.15.4 address
 */
typedef struct uip_802154_shortadd
{
    /* short address */
    uint8_t addr[2];

} uip_802154_shortaddr;

/**
 * \brief 64 bit 802.15.4 address
 */
typedef struct uip_802154_longaddr
{
    /* long address */
    uint8_t addr[8];

} uip_802154_longaddr;


/** 802.15.4 address */
typedef uip_802154_longaddr uip_lladdr_t;



typedef void (*mac_callback_t)( void *ptr, int status, int transmissions );
typedef void (*nsTxCbFnct_t)( void *p_arg, e_nsErr_t *p_err );
typedef void (*nsRxCbFnct_t)( uint8_t *p_data, uint16_t len, e_nsErr_t *p_err );

/*
 * Forward declaration of the different layer structures.
 */
typedef struct s_ns s_ns_t;
typedef struct s_nsSocket s_nsSocket_t;
typedef struct s_nsHeadComp s_nsHeadComp_t;
typedef struct s_nsDllsec s_nsDllsec_t;
typedef struct s_nsFramer s_nsFramer_t;
typedef struct s_nsModuleDrv s_nsModuleDrv_t;


/**
 * The structure for the framer.
 */
struct s_nsFramer
{
  /** Name of the compression */
    char *name;

    /** Initialize the network driver */
    int8_t (*init)(s_ns_t* p_ns);

    /** get length */
    int8_t (*length)(void);

    /** create frame */
    int8_t (*create)(void);

    /** parse frame */
    int8_t (*parse)(void);

};


/**
 * The structure for the header compression driver.
 */
struct s_nsHeadComp
{
    /** Name of the compression */
    char *name;

    /** Initialize the network driver */
    void (*init)(s_ns_t* p_ns);

    /** Callback for getting notified of incoming packet. */
    void (*input)(void);

};


/**
 * The structure of a data link layer security driver.
 */
struct s_nsDllsec
{
    /** Name of the driver */
    char *name;

    /** Initializes link layer security and thereafter starts upper layers. */
    void (*init)(s_ns_t* p_ns);

    /** Secures outgoing frames before passing them to NETSTACK_MAC. */
    void (*send)(mac_callback_t sent_callback, void *ptr);

    /**
     * Once the netstack_framer wrote the headers, the llsec driver
     * can generate a MIC over the entire frame.
     * \return Returns != 0 <-> success
     */
    int (*on_frame_created)(void);

    /**
     * Decrypts incoming frames;
     * filters out injected or replayed frames.
     */
    void (*input)(void);

    /** Returns the security-related overhead per frame in bytes */
    uint8_t (*get_overhead)(void);

};


/**
 * \brief   Netstack submodule driver structure declaration
 */
struct s_nsModuleDrv
{
    /** Driver name */
    char *name;

    /** Initialization handler */
    void (*init)(void *p_netstk, e_nsErr_t *p_err);

    /** Turn the driver on */
    void (*on)(e_nsErr_t *p_err);

    /** Turn the driver off */
    void (*off)(e_nsErr_t *p_err);

    /** Data transmission handler */
    void (*send)(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);

    /** Data reception handler */
    void (*recv)(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);

    /** IO control */
    void (*ioctrl)(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

};


/** DLLC driver forward declaration */
typedef s_nsModuleDrv_t s_nsDLLC_t;

/** MAC driver forward declaration */
typedef s_nsModuleDrv_t s_nsMAC_t;

/** PHY driver forward declaration */
typedef s_nsModuleDrv_t s_nsPHY_t;

/** Radio driver forward declaration */
typedef s_nsModuleDrv_t s_nsRF_t;


/**
 * \brief   Netstack driver structure declaration
 */
struct s_ns
{
    /** Pointer to 6LoWPAN framing driver */
    const s_nsFramer_t* frame;

    /**Pointer to 6LoWPAN header compressor Driver */
    const s_nsHeadComp_t* hc;

    /** Pointer to Data Link Layer Security Driver */
    const s_nsDllsec_t* dllsec;

    /** Pointer to Data Link Layer Control Driver */
    const s_nsDLLC_t* dllc;

    /** Pointer to Medium Access Control Driver */
    const s_nsMAC_t* mac;

    /**Pointer to Physical Driver */
    const s_nsPHY_t* phy;

    /** Pointer to Radio Frequency Driver */
    const s_nsRF_t* rf;

    /** Indicate whether netstack is already configured */
    uint8_t c_configured;

    /** status of the stack */
    e_stack_status_t status;

};


/**
 * \brief   Netstack socket driver structure declaration.
 */
struct s_nsSocket
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


/** Declaration of a emb6 demo */
typedef struct s_demo s_demo_t;

/**
 * emb6 demo definition.
 */
struct s_demo
{
  /** initialization function */
  int8_t(*pf_init)(void);

  /** configuration function */
  int8_t(*pf_conf)(s_ns_t* p_ns);

  /** next demo */
  s_demo_t* p_next;
};


/*
 *  --- External Variable Declaration ----------------------------------------*
 */

/** External declaration for RPL configuration */
extern s_rpl_conf_t rpl_config;

/** External declaration for the PHY/MAC configuration */
extern s_mac_phy_conf_t  mac_phy_config;


/* Supported headers compression handlers */
extern const s_nsHeadComp_t hc_driver_sicslowpan;
/* This driver are pretending to be a hc layer
 * for sniffing data and sending them via USART */
extern const s_nsHeadComp_t hc_driver_slipnet;


/* Supported link layer security handlers */
extern const s_nsDllsec_t dllsec_driver_null;


/* Supported framers */
extern const s_nsFramer_t framer_802154;
extern const s_nsFramer_t framer_noframer;
extern const s_nsFramer_t framer_nullframer;

/* Supported dllc drivers */
extern const s_nsDLLC_t dllc_driver_null;
extern const s_nsDLLC_t dllc_driver_802154;

/* Supported mac drivers */
extern const s_nsMAC_t mac_driver_null;
extern const s_nsMAC_t mac_driver_802154;
extern const s_nsMAC_t mac_driver_smartmac;

/* Supported phy drivers */
extern const s_nsPHY_t phy_driver_null;
extern const s_nsPHY_t phy_driver_802154;

/* Supported rf drivers */
extern const s_nsRF_t rf_driver_null;
extern const s_nsRF_t rf_driver_native;
extern const s_nsRF_t rf_driver_at212;
extern const s_nsRF_t rf_driver_at212b;
extern const s_nsRF_t rf_driver_ticc112x;
extern const s_nsRF_t rf_driver_ticc120x;
extern const s_nsRF_t rf_driver_ticc13xx;


/*
 *  --- Global Functions Definition ------------------------------------------*
 */


/**
 * emb6_init()
 *
 * \brief   Initialize the emb::6 stack.
 *
 *          This function is called to initialize the complete stack. This
 *          includes the initialization of all layer an the underlying
 *          board support package and hardware.
 *          It also sets the internal stack pointer to the parameter
 *          given.
 *
 * \param   pst_netStack    Pointer to the stack structure to initialize.
 * \param   ps_demos        Demos to initialize.
 * \param   p_err           Pointer to store error status to.
 */
void emb6_init( s_ns_t* ps_ns, s_demo_t* ps_demos, e_nsErr_t* p_err );


/**
 * emb6_process()
 *
 * \brief   Process the emb::6 stack.
 *
 *          This function must be called to execute the stack. The
 *          function internally checks for event to process. It can
 *          be delayed to e.g. to reduce CPU usage.
 *          This function can either be called periodically using a negative
 *          delay or once using a delay >= 0. For the second case the function
 *          will not return.
 *
 * \param   delay   Delay for the internal loop or a negative value for
 *                  single execution.
 */
void emb6_process( int32_t delay );


/**
 * emb6_get()
 *
 * \brief   Get the current emb::6 stack pointer.
 *
 *          This function returns the emb::6 stack pointer which was given
 *          during the initialization
 *
 * \return  Pointer to the stack structure given during initialization.
 */
const s_ns_t* emb6_get( void );


/**
 * emb6_getStatus()
 *
 * \brief   Get the status of the stack.
 *
 *          This function returns the current status of the stack.
 *
 * \return  Status of the stack.
 */
e_stack_status_t emb6_getStatus( void );



/**
 * emb6_start()
 *
 * \brief   Start the emb::6 stack.
 *
 *          DETAILED DESCRIPTION HERE.
 *
 * \param   p_err   Pointer to store the error code.
 */
void emb6_start( e_nsErr_t *p_err );


/**
 * emb6_stop()
 *
 * \brief   Stop the emb::6 stack.
 *
 *          DETAILED DESCRIPTION HERE.
 *
 * \param   p_err   Pointer to store the error code.
 */
void emb6_stop( e_nsErr_t *p_err );


/**
 * emb6_errorHandler()
 *
 * \brief   Global error handler.
 *
 *          This function acts as a global error handler. It can be called
 *          whenever an unexpected behavior was detected.
 *          The error handler calls an endless loop.
 *
 * \param   p_err   Error that occurred.
 */
void emb6_errorHandler( e_nsErr_t* p_err );

#endif /* __EMB6_H__ */

