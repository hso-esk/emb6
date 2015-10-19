/**
 * @file    netstk_def.h
 * @date    Oct 16, 2015
 * @author  PN
 * @brief   This header file contains defines, structure declarations and data
 *          types for all modules belonging to Netstack layers
 */

#ifndef NETSTK_PRESENT
#define NETSTK_PRESENT

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "netstk_cfg.h"
#include "emb6.h"

/*
********************************************************************************
*                               VERSION DECLARATION
********************************************************************************
*/
#define NETSTK_VERSION        000001u         /* Version format Vx.yy.zz */



/*
********************************************************************************
*                            DATA TYPES DECLARATIONS
********************************************************************************
*/
#if STK_CFG_REFACTOR_EN
typedef uint16_t        NETSTK_DEV_ID;
#endif


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
    NETSTK_ERR_NONE = 0u,
    NETSTK_ERR_INIT,
    NETSTK_ERR_BUSY,
    NETSTK_ERR_BUF_OVERFLOW,
    NETSTK_ERR_INVALID_ARGUMENT,
    NETSTK_ERR_INVALID_FRAME,
    NETSTK_ERR_TX_TIMEOUT,
    NETSTK_ERR_TX_NOACK,
    NETSTK_ERR_CMD_UNSUPPORTED,
    NETSTK_ERR_FATAL,


    /*
     * LLC error codes
     */
    NETSTK_ERR_LLC_XXX = 100U,

    /*
     * MAC error codes
     */
    NETSTK_ERR_MAC_XXX = 200U,

    /*
     * PHY error codes
     */
    NETSTK_ERR_PHY_XXX = 300U,


    /*
     * LPR error codes
     */
    NETSTK_ERR_LPR_XXX = 400U,
    NETSTK_ERR_LPR_UNSUPPORTED_FRAME,
    NETSTK_ERR_LPR_BROADCAST_LAST_STROBE,
    NETSTK_ERR_LPR_BROADCAST_NOACK,
    NETSTK_ERR_LPR_CHANNEL_ACESS_FAILURE,
    NETSTK_ERR_LPR_INVALID_ADDR,
    NETSTK_ERR_LPR_TX_COLLISION_SAME_DEST,
    NETSTK_ERR_LPR_TX_COLLISION_DIFF_DEST,
    NETSTK_ERR_LPR_INVALID_ACK,


    /*
     * RF error codes
     */
    NETSTK_ERR_RF_XXX = 500U,
    NETSTK_ERR_RF_SEND,

}NETSTK_ERR;


/**
 * @brief   Netstack I/O Control command enumeration declaration
 */
typedef enum netstk_ioc_cmd
{
    /*
     * Common command codes
     */
    NETSTK_CMD_NONE = 0,
    NETSTK_CMD_TX_CBFNCT_SET,
    NETSTK_CMD_TX_CBARG_SET,
    NETSTK_CMD_RX_CBFNT_SET,

    /*
     * LLC command codes
     */
    NETSTK_CMD_LLC_XXX = 100U,

    /*
     * MAC command codes
     */
    NETSTK_CMD_MAC_XXX = 200U,
    NETSTK_CMD_MAC_DSN_SET,

    /*
     * PHY command codes
     */
    NETSTK_CMD_PHY_XXX = 300U,


    /*
     * LPR command codes
     */
    NETSTK_CMD_LPR_XXX = 400U,

    /*
     * RF command codes
     */
    NETSTK_CMD_RF_XXX = 500U,

    RADIO_IOC_CMD_TXPOWER_SET,
    RADIO_IOC_CMD_TXPOWER_GET,
    RADIO_IOC_CMD_SENS_SET,
    RADIO_IOC_CMD_SENS_GET,
    RADIO_IOC_CMD_RSSI_GET,
    RADIO_IOC_CMD_CCA_GET,
    RADIO_IOC_CMD_ANT_DIV_SET,
    RADIO_IOC_CMD_RF_SWITCH,
    RADIO_IOC_CMD_SYNC_SET,
    RADIO_IOC_CMD_SYNC_GET,

    RADIO_IOC_CMD_IS_RX_BUSY,
    RADIO_IOC_CMD_IS_TX_BUSY,

}NETSTK_IOC_CMD;


/*
********************************************************************************
*                           STRUCTURE DECLARATIONS
********************************************************************************
*/

/**
 * @brief   Netstack common API structure declaration
 */
typedef struct netstk_module_api     NETSTK_MODULE_DRV;

struct netstk_module_api
{
    char     *name;

    void    (*init  )(void *p_netstk, NETSTK_ERR *p_err);                       /*!< Initialization handler       */

#if 0
    void    (*deinit)(NETSTK_ERR *p_err);                                       /*!< Deinitialization handler     */
#endif

    void    (*on    )(NETSTK_ERR *p_err);

    void    (*off   )(NETSTK_ERR *p_err);

    void    (*send  )(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);        /*!< Data transmission handler    */

    void    (*recv  )(uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);        /*!< Data reception handler       */

    void    (*ioctrl)(NETSTK_IOC_CMD cmd, void *p_val, NETSTK_ERR *p_err);      /*!< Miscellaneous functionality  */
};


/**
 * @brief   Netstack Stack API structure declaration
 */
typedef struct netstk_api    NETSTK_DRV;

struct netstk_api
{
    NETSTK_MODULE_DRV      *llc;           /*!< Pointer to Logical Link Control Driver     */

    NETSTK_MODULE_DRV      *mac;           /*!< Pointer to Medium Access Control Driver    */

    NETSTK_MODULE_DRV      *phy;           /*!< Pointer to Physical Driver                 */

    NETSTK_MODULE_DRV      *lpr;           /*!< Pointer to Low-Power Radio Driver          */

    NETSTK_MODULE_DRV      *rf;            /*!< Pointer to Radio Frequency Driver          */
};


/*
********************************************************************************
*                           STRUCTURE DECLARATIONS
********************************************************************************
*/
typedef void (*NETSTK_CBFNCT) (void *p_arg, NETSTK_ERR *p_err);

typedef void (*NETSTK_CBRXFNCT) (uint8_t *p_data, uint16_t len, NETSTK_ERR *p_err);

/*
********************************************************************************
*                           LLC DRIVERS DECLARATIONS
********************************************************************************
*/
#if NETSTK_CFG_LLC_NULL_EN
extern  NETSTK_MODULE_DRV   LLCDrvNull;
#endif

#if NETSTK_CFG_LLC_802154_EN
extern  NETSTK_MODULE_DRV   LLCDrv802154;
#endif


/*
********************************************************************************
*                           MAC DRIVERS DECLARATIONS
********************************************************************************
*/
#if NETSTK_CFG_MAC_NULL_EN
extern  NETSTK_MODULE_DRV   MACDrvNull;
#endif

#if NETSTK_CFG_MAC_802154_EN
extern  NETSTK_MODULE_DRV   MACDrv802154;
#endif


/*
********************************************************************************
*                           PHY DRIVERS DECLARATIONS
********************************************************************************
*/
#if NETSTK_CFG_PHY_NULL_EN
extern  NETSTK_MODULE_DRV   PHYDrvNull;
#endif


/*
********************************************************************************
*                           LPR DRIVERS DECLARATIONS
********************************************************************************
*/
#if NETSTK_CFG_LPR_APSS_EN
extern  NETSTK_MODULE_DRV   LPRDrvAPSS;
#endif

#if NETSTK_CFG_LPR_NULL_EN
extern  NETSTK_MODULE_DRV   LPRDrvNull;
#endif


/*
********************************************************************************
*                           RF DRIVERS DECLARATIONS
********************************************************************************
*/
#if NETSTK_CFG_RF_CC1120_EN
extern  NETSTK_MODULE_DRV   RFDrvCC1120;
#endif

#if NETSTK_CFG_RF_NULL_EN
extern  NETSTK_MODULE_DRV   RFDrvNull;
#endif


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
#endif /* NETSTK_PRESENT */
