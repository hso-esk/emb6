/*
 * include.h
 *
 *  Created on: Aug 18, 2015
 *      Author: phuongnguyen
 */

#ifndef SOURCE_INCLUDE_H_
#define SOURCE_INCLUDE_H_

#include <stdint.h>
#include <stddef.h>



/**
 * @brief   CPU Critical Section
 */
#ifdef IAR_COMPILER
#include "bsp.h"
#define CPU_ENTER_CRITICAL()        bsp_enterCritical()
#define CPU_EXIT_CRITICAL()         bsp_exitCritical()
#define LED_RX_ON()                 bsp_ledSet(E_BSP_LED_1)
#define LED_RX_OFF()                bsp_ledClear(E_BSP_LED_1)
#define LED_TX_ON()                 bsp_ledSet(E_BSP_LED_2)
#define LED_TX_OFF()                bsp_ledClear(E_BSP_LED_2)
#define LED_SCAN_ON()               bsp_ledSet(E_BSP_LED_3)
#define LED_SCAN_OFF()              bsp_ledClear(E_BSP_LED_3)
#else
#define CPU_ENTER_CRITICAL()
#define CPU_EXIT_CRITICAL()
#define LED_RX_ON()
#define LED_RX_OFF()
#define LED_TX_ON()
#define LED_TX_OFF()
#define LED_SCAN_ON()
#define LED_SCAN_OFF()
#endif


#define REFACTOR_APSS_EN                    ( 0u )
#define STKCFG_MEASURE_EN                   ( 1u )
#define APSS_CCA_BASED_SCAN_EN              ( 0u )

#ifdef TDD_ENABLED
#define APSS_RX
#define APSS_TX
#define STKCFG_ARG_CHK_EN                   ( 1u )
#else
#define STKCFG_ARG_CHK_EN                   ( 0u )
#endif


/**
 * @addtogroup  GLOBAL_DATA_TYPES   Global stack data type definitions
 * @{
 */
typedef uint8_t     STK_ERR;
typedef uint8_t     STK_DATA;
typedef uint16_t    STK_DATALEN;
typedef uint8_t     STK_IOC_CMD;
typedef uint16_t    STK_IOC_VAL;
typedef uint16_t    STK_DEV_ID;

typedef uint8_t     APSS_STATE;
typedef uint8_t     APSS_FRAME_TYPE;

typedef uint8_t     RADIO_STATE;
typedef uint16_t    RADIO_ERR;
typedef uint8_t     RADIO_IOC_CMD;
typedef uint16_t    RADIO_IOC_VAL;

typedef void (*STK_FNCT_VOID) (void *p_arg);

typedef struct mac_drv_api      MAC_DRV_API;
typedef struct apss_drv_api     APSS_DRV_API;
typedef struct apss_framer_api  APSS_FRAMER_API;
typedef struct radio_drv_api    RADIO_DRV_API;


/**
 * @}
 */


/**
 * @addtogroup  STACK_ERROR_CODES   Stack error codes
 * @{
 */
#define STK_ERR_NONE                    (STK_ERR) ( 0u )
#define STK_ERR_BUSY                    (STK_ERR) ( 1u )
#define STK_ERR_TX_RADIO_SEND           (STK_ERR) ( 2u )
#define STK_ERR_TX_TIMEOUT              (STK_ERR) ( 3u )
#define STK_ERR_TX_NOPACK               (STK_ERR) ( 4u )
#define STK_ERR_NULL_POINTER            (STK_ERR) ( 5u )

#define STK_ERR_APSS_TERMINATED         (STK_ERR) ( 11u )
#define STK_ERR_APSS_ACK_INVALID        (STK_ERR) ( 12u )
#define STK_ERR_APSS_FRAME_INVALID      (STK_ERR) ( 13u )
#define STK_ERR_APSS_BROADCAST          (STK_ERR) ( 14u )


/**
 * @}
 */


/**
 * @addtogroup  RADIO_ERROR_CODES  Radio error codes
 * @{
 */
#define RADIO_ERR_NONE                  (RADIO_ERR) ( 0u )
#define RADIO_ERR_CMD_UNSUPPORTED       (RADIO_ERR) ( 1u )
#define RADIO_ERR_TX                    (RADIO_ERR) ( 2u )
#define RADIO_ERR_ONOFF                 (RADIO_ERR) ( 3u )

/**
 * @}
 */


/**
 * @addtogroup  APSS_STATES     APSS states
 * @{
 */
#define APSS_STATE_NONE                 (APSS_STATE) (  0u )
#define APSS_STATE_IDLE                 (APSS_STATE) (  1u )
#define APSS_STATE_SLEEP                (APSS_STATE) (  2u )

#define APSS_STATE_SCAN_STARTED         (APSS_STATE) ( 10u )
#define APSS_STATE_SCAN_CCA             (APSS_STATE) ( 11u )
#define APSS_STATE_SCAN_CCA_SLEEP       (APSS_STATE) ( 12u )
#define APSS_STATE_SCAN_WFSP            (APSS_STATE) ( 13u )
#define APSS_STATE_SCAN_WFSPE           (APSS_STATE) ( 14u )
#define APSS_STATE_SCAN_DONE            (APSS_STATE) ( 15u )

#define APSS_STATE_TX_SP                (APSS_STATE) ( 20u )
#define APSS_STATE_TX_SPD               (APSS_STATE) ( 21u )
#define APSS_STATE_TX_SPWFA             (APSS_STATE) ( 22u )
#define APSS_STATE_TX_SPWFAE            (APSS_STATE) ( 23u )
#define APSS_STATE_TX_P                 (APSS_STATE) ( 24u )
#define APSS_STATE_TX_PWFA              (APSS_STATE) ( 25u )
#define APSS_STATE_TX_DONE              (APSS_STATE) ( 26u )

#define APSS_STATE_RX_SP                (APSS_STATE) ( 30u )    /*!< Received a valid Strobe and reply with an ACK immediately  */
#define APSS_STATE_RX_SPD               (APSS_STATE) ( 31u )    /*!< Received a valid Strobe and reply with an ACK after a Delay */
#define APSS_STATE_RX_WFP               (APSS_STATE) ( 32u )
#define APSS_STATE_RX_WFPE              (APSS_STATE) ( 33u )
#define APSS_STATE_RX_P                 (APSS_STATE) ( 34u )

/**
 * @}
 */

/**
 * @addtogroup  APSS_FRAME_TYPES    APSS frame types
 * @note        APSS frame types use reserved values of frame types specified
 *              in IEEE802.15.4
 * @{
 */

#define APSS_FRAME_TYPE_STROBE              (APSS_FRAME_TYPE) ( 0x14 )
#define APSS_FRAME_TYPE_SACK                (APSS_FRAME_TYPE) ( 0x15 )
/**
 * @}
 */

/**
 * @addtogroup  RADIO_IOC_CMD Radio I/O control commands
 * @{
 */
#define RADIO_IOC_CMD_TXPOWER_SET           (RADIO_IOC_CMD) (  1u )
#define RADIO_IOC_CMD_TXPOWER_GET           (RADIO_IOC_CMD) (  2u )
#define RADIO_IOC_CMD_SENS_SET              (RADIO_IOC_CMD) (  3u )
#define RADIO_IOC_CMD_SENS_GET              (RADIO_IOC_CMD) (  4u )
#define RADIO_IOC_CMD_RSSI_GET              (RADIO_IOC_CMD) (  5u )
#define RADIO_IOC_CMD_CCA_GET               (RADIO_IOC_CMD) (  6u )
#define RADIO_IOC_CMD_ANT_DIV_SET           (RADIO_IOC_CMD) (  7u )
#define RADIO_IOC_CMD_RF_SWITCH             (RADIO_IOC_CMD) (  8u )
#define RADIO_IOC_CMD_SYNC_SET              (RADIO_IOC_CMD) (  9u )
#define RADIO_IOC_CMD_SYNC_GET              (RADIO_IOC_CMD) ( 10u )
#define RADIO_IOC_CMD_STATE_GET             (RADIO_IOC_CMD) ( 11u )

#if 0 // not needed at the time being
#define RADIO_IOC_CMD_MACADDR_SET           (RADIO_IOC_CMD) ( 11u )
#define RADIO_IOC_CMD_MACADDR_GET           (RADIO_IOC_CMD) ( 12u )
#define RADIO_IOC_CMD_ED_GET                (RADIO_IOC_CMD) ( 20u )
#endif

/**
 * @}
 */


/**
 * @addtogroup  RADIO_IOC_VAL
 * @{
 */
#define RADIO_IOC_VAL_SYNC_SP               (RADIO_IOC_VAL) ( 0x930B )
#define RADIO_IOC_VAL_SYNC_DATA             (RADIO_IOC_VAL) ( 0x51DE )

#define RADIO_IOC_VAL_STATE_NONE            (RADIO_IOC_VAL) ( 0u )
#define RADIO_IOC_VAL_STATE_IDLE            (RADIO_IOC_VAL) ( 1u )
#define RADIO_IOC_VAL_STATE_RX              (RADIO_IOC_VAL) ( 2u )
#define RADIO_IOC_VAL_STATE_TX              (RADIO_IOC_VAL) ( 3u )

/**
 * @}
 */

/**
 * @brief   MAC driver API structure declaration
 */
struct mac_drv_api {
    char     *Name;

    void    (*IsrRx) (uint8_t *p_data, uint8_t len, STK_ERR *p_err);            /*!< Packet reception handler   */

    void    (*CbTx ) (STK_ERR err);                                             /*!< Packet reception handler   */
};



/**
 * @brief   APSS driver API structure declaration
 */
struct apss_drv_api
{
    char     *Name;

    void    (*Init ) (STK_ERR *p_err);

    void    (*On   ) (STK_ERR *p_err);                                          /*!< Open the driver            */

    void    (*Off  ) (STK_ERR *p_err);                                          /*!< Close the driver           */

    void    (*Send ) (STK_FNCT_VOID fnct, void *arg, STK_ERR *p_err);           /*!< Write data to radio        */

    void    (*Recv ) (void *p_buf, uint8_t len, STK_ERR *p_err);                /*!< Read data from radio       */

    void    (*IsrRx) (uint8_t *p_data, uint8_t len, STK_ERR *p_err);            /*!< Packet reception handler   */

    void    (*Ioctl) (STK_IOC_CMD cmd, STK_IOC_VAL *p_val, STK_ERR *p_err);     /*!< Input/Output control       */

    void    (*Task ) (void *p_arg);                                             /*!< State machine handler      */
};


/**
 * @brief   Asynchronous Power Saving Scheme Framer API
 */
struct apss_framer_api
{
    char         *Name;

    void        (*Init          ) (STK_ERR *p_err);

    void        (*Deinit        ) (STK_ERR *p_err);

    uint8_t*    (*CreateWakeup  ) (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);

    uint8_t*    (*CreateACK     ) (uint16_t *p_len, LIB_TMR_TICK *p_delay, STK_ERR *p_err);

    void        (*Parse         ) (uint8_t *p_pkt, uint16_t len, STK_ERR *p_err);
};


/**
 * @brief   Radio transceiver driver API
 */
struct radio_drv_api
{
    char     *Name;

    void    (*Init ) (RADIO_ERR *p_err);

    void    (*On   ) (RADIO_ERR *p_err);                                            /*!< Open the driver        */

    void    (*Off  ) (RADIO_ERR *p_err);                                            /*!< Close the driver       */

    void    (*Send ) (const void *p_payload, uint8_t len, RADIO_ERR *p_err);        /*!< Write data to radio    */

    void    (*Recv ) (void *p_buf, uint8_t len, RADIO_ERR *p_err);                  /*!< Read data from radio   */

    void    (*Ioctl) (RADIO_IOC_CMD cmd, RADIO_IOC_VAL *p_val, RADIO_ERR *p_err);   /*!< Input/Output control   */

    void    (*Task ) (void *p_arg);                                                 /*!< State machine handler  */
};


/**
 * @addtogroup  Global_variables
 * @{
 */
extern  APSS_STATE               APSSState;
extern  RADIO_STATE              Radio_State;
extern  STK_DEV_ID               APSSDevId;
extern  STK_DEV_ID               APSSDestId;
extern  MAC_DRV_API             *HighMacDrv;
extern  APSS_DRV_API            *LowMacDrv;
extern  APSS_FRAMER_API         *APSSFramer;
extern  RADIO_DRV_API           *RadioDrv;

extern  APSS_DRV_API             APSSDrv;
extern  APSS_FRAMER_API          XMACFramer;
extern  APSS_FRAMER_API          SmartMACFramer;
extern  RADIO_DRV_API            CC112xDrv;
extern  RADIO_DRV_API            StubCC112x_Drv;


/* TODO this temporary buffer is used for testing purpose and should be replaced
 * with packet_buffer module of the emb6 */
extern  uint8_t StkBuf[128];
extern  uint8_t StkBufLen;


/**
 * @}
 */

/**
 * @addtogroup  Configurations
 * @{
 */
#if APSS_CCA_BASED_SCAN_EN
#define APSS_TMR_SCAN_CCA_INTERVAL  (LIB_TMR_TICK) (   5u )         /*!< minimum    */
#define APSS_TMR_SCAN_DURATION      (LIB_TMR_TICK) (  30u )         /*!< minimum    */
#else
#define APSS_TMR_SCAN_DURATION      (LIB_TMR_TICK) (  25u )         /*!< minimum    */
#endif

#define APSS_TMR_WFP_TIMEOUT        (LIB_TMR_TICK) (  10u )         /*!< fixed      */
#define APSS_TMR_WFA_TIMEOUT        (LIB_TMR_TICK) (  10u )         /*!< fixed      */
#define APSS_TMR_POWERUP_INTERVAL   (LIB_TMR_TICK) ( 1000u )        /*!< variable   */
#define APSS_TMR_TXSP_TIMEOUT       (APSS_TMR_POWERUP_INTERVAL * 2)
/**
 * @}
 */



/*
********************************************************************************
*                           CONFIGURATION CHECKING
********************************************************************************
*/
#if     defined(XMAC_TX) | defined(XMAC_RX)
#define APSS_FRAMER                     XMACFramer
#elif   defined(SMARTMAC_TX) | defined(SMARTMAC_RX)
#define APSS_FRAMER                     SmartMACFramer
#else
#error  Only XMAC and SmartMAC are supported
#endif

#if     defined(XMAC_TX) | defined(SMARTMAC_TX)
#define APSS_TX
#elif   defined(XMAC_RX) | defined(SMARTMAC_RX)
#define APSS_RX
#else
#error  Only TX or RX operation modes are supported
#endif


#endif /* SOURCE_INCLUDE_H_ */
