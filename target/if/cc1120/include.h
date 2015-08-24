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
 * @addtogroup  GLOBAL_DATA_TYPES   Global stack data type definitions
 * @{
 */
typedef uint8_t     STK_ERR;
typedef uint8_t     STK_DATA;
typedef uint16_t    STK_DATALEN;
typedef uint8_t     STK_IOC_CMD;
typedef uint16_t    STK_IOC_VAL;
typedef uint16_t    STK_DEV_ID;

typedef uint8_t     XMAC_STATE;
typedef uint8_t     XMAC_FRAME_TYPE;

typedef uint8_t     RADIO_STATE;
typedef uint16_t    RADIO_ERR;
typedef uint8_t     RADIO_IOC_CMD;
typedef uint16_t    RADIO_IOC_VAL;

typedef void (*STK_FNCT_VOID) (void *p_arg);

/**
 * @}
 */


/**
 * @addtogroup  STACK_ERROR_CODES   Stack error codes
 * @{
 */
#define STK_ERR_NONE                    (STK_ERR) ( 0u )
#define STK_ERR_BUSY                    (STK_ERR) ( 1u )
/**
 * @}
 */


/**
 * @addtogroup  RADIO_ERROR_CODES  Radio error codes
 * @{
 */
#define RADIO_ERR_NONE                  (RADIO_ERR) ( 0u )
#define RADIO_ERR_CMD_UNSUPPORTED       (RADIO_ERR) ( 1u )
/**
 * @}
 */


/**
 * @addtogroup  XMAC_STATES     XMAC states
 * @{
 */
#define XMAC_STATE_CREATED          (XMAC_STATE) (  0u )
#define XMAC_STATE_IDLE             (XMAC_STATE) (  1u )
#define XMAC_STATE_SLEEP            (XMAC_STATE) (  2u )

#define XMAC_STATE_SCAN             (XMAC_STATE) ( 10u )
#define XMAC_STATE_SCAN_WFSP        (XMAC_STATE) ( 11u )

#define XMAC_STATE_TXSP             (XMAC_STATE) ( 25u )
#define XMAC_STATE_TXSPWFA          (XMAC_STATE) ( 26u )
#define XMAC_STATE_TX               (XMAC_STATE) ( 27u )
#define XMAC_STATE_TXWFA            (XMAC_STATE) ( 28u )

#define XMAC_STATE_RX               (XMAC_STATE) ( 34u )

/**
 * @}
 */

/**
 * @addtogroup  XMAC_FRAME_TYPES    XMAC frame types
 * @note        XMAC frame types use reserved values of frame types specified
 *              in IEEE802.15.4
 * @{
 */

#define XMAC_FRAME_TYPE_TIMESTAMP           (XMAC_FRAME_TYPE) ( 0x14 )
#define XMAC_FRAME_TYPE_ACK                 (XMAC_FRAME_TYPE) ( 0x15 )
/**
 * @}
 */

/**
 * @addtogroup  RADIO_IOC_CMD Radio I/O control commands
 * @{
 */
#define RADIO_IOC_CMD_TXPOWER_SET         (RADIO_IOC_CMD) (  1u )
#define RADIO_IOC_CMD_TXPOWER_GET         (RADIO_IOC_CMD) (  2u )
#define RADIO_IOC_CMD_SENS_SET            (RADIO_IOC_CMD) (  3u )
#define RADIO_IOC_CMD_SENS_GET            (RADIO_IOC_CMD) (  4u )
#define RADIO_IOC_CMD_RSSI_GET            (RADIO_IOC_CMD) (  5u )
#define RADIO_IOC_CMD_CCA_GET             (RADIO_IOC_CMD) (  6u )
#define RADIO_IOC_CMD_ANT_DIV_SET         (RADIO_IOC_CMD) (  7u )
#define RADIO_IOC_CMD_RF_SWITCH           (RADIO_IOC_CMD) (  8u )
#define RADIO_IOC_CMD_SYNC_SET            (RADIO_IOC_CMD) (  9u )
#define RADIO_IOC_CMD_SYNC_GET            (RADIO_IOC_CMD) ( 10u )

/**
 * @}
 */


/**
 * @addtogroup  RADIO_SYNC
 * @{
 */
#define RADIO_IOC_VAL_SYNC_SP             (RADIO_IOC_VAL) ( 0x930B )
#define RADIO_IOC_VAL_SYNC_DATA           (RADIO_IOC_VAL) ( 0x51DE )
/**
 * @}
 */

/**
 * @brief   MAC driver API structure declaration
 */
typedef struct mac_drv_api      MAC_DRV_API;

struct mac_drv_api {
    char     *Name;
    void    (*IsrRx) (uint8_t *p_data, uint8_t len, STK_ERR *p_err);            /*!< Packet reception handler   */
};



/**
 * @brief   XMAC driver API structure declaration
 */
typedef struct xmac_drv_api     XMAC_DRV_API;

struct xmac_drv_api {
    char     *Name;

    void    (*Init ) (STK_ERR *p_err);

    void    (*On   ) (STK_ERR *p_err);                                          /*!< Open the driver            */

    void    (*Off  ) (STK_ERR *p_err);                                          /*!< Close the driver           */

#if 1
    void    (*Send ) (STK_FNCT_VOID fnct, void *arg, STK_ERR *p_err);           /*!< Write data to radio        */
#else
    void    (*Send ) (const void *p_payload, uint8_t len, STK_ERR *p_err);      /*!< Write data to radio        */
#endif

    void    (*Recv ) (void *p_buf, uint8_t len, STK_ERR *p_err);                /*!< Read data from radio       */

    void    (*IsrRx) (uint8_t *p_data, uint8_t len, STK_ERR *p_err);            /*!< Packet reception handler   */

    void    (*Ioctl) (STK_IOC_CMD cmd, STK_IOC_VAL *p_val, STK_ERR *p_err);     /*!< Input/Output control       */

    void    (*Task ) (void *p_arg);                                             /*!< State machine handler      */
};


/**
 * @brief   Radio transceiver driver API
 */
typedef struct radio_drv_api    RADIO_DRV_API;

struct radio_drv_api {
    char     *Name;

    void    (*Init ) (RADIO_ERR *p_err);

    void    (*On   ) (RADIO_ERR *p_err);                                        /*!< Open the driver        */

    void    (*Off  ) (RADIO_ERR *p_err);                                        /*!< Close the driver       */

    void    (*Send ) (const void *p_payload, uint8_t len, RADIO_ERR *p_err);    /*!< Write data to radio    */

    void    (*Recv ) (void *p_buf, uint8_t len, RADIO_ERR *p_err);              /*!< Read data from radio   */

    void    (*Ioctl) (RADIO_IOC_CMD cmd, RADIO_IOC_VAL *p_val, RADIO_ERR *p_err);          /*!< Input/Output control   */
};


/**
 * @addtogroup  Global_variables
 * @{
 */
extern  XMAC_STATE          XmacState;
extern  RADIO_STATE         Radio_State;
extern  STK_DEV_ID          XmacDevId;
extern  STK_DEV_ID          XmacDestId;
extern  MAC_DRV_API        *HighMacDrv;
extern  XMAC_DRV_API       *LowMacDrv;
extern  RADIO_DRV_API      *RadioDrv;

extern  const XMAC_DRV_API  XmacDrv;
extern  const RADIO_DRV_API CC112x_Drv;

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
#define XMAC_TMR_POWERUP_INTERVAL   (UTIL_TMR_TICK) ( 500u )    /*!< 500ms */
#define XMAC_TMR_SCAN_DURATION      (UTIL_TMR_TICK) (   5u )    /*!<   5ms */
#define XMAC_TMR_WFP_TIMEOUT        (UTIL_TMR_TICK) (   3u )    /*!<   3ms */
#define XMAC_TMR_WFA_TIMEOUT        (UTIL_TMR_TICK) (   3u )    /*!<   3ms */
#define XMAC_TMR_TXSP_TIMEOUT       (UTIL_TMR_TICK) ( 600u )    /*!< 600ms */

#define XMAC_TXSP_RETRY_MAX         (STK_DATA) ( 30u )
/**
 * @}
 */

#endif /* SOURCE_INCLUDE_H_ */
