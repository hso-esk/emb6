/**
 * @file    radio.h
 * @author  PN
 * @brief   Radio Frequency driver module
 */

#ifndef RADIO_PRESENT
#define RADIO_PRESENT

/**
 * @addtogroup  RADIO_ERR_CODE
 * @{
 */
typedef uint16_t    RADIO_ERR;

#define RADIO_ERR_CMD_UNSUPPORTED       (RADIO_ERR) ( 1u )

/**
 * @}
 */

/**
 * @addtogroup  IOC I/O control
 * @{
 */
typedef uint8_t     IOCTRL;
typedef uint16_t    RADIO_VAL;

#define IOC_CMD_TXPOWER_SET         (IOCTRL) (  1u )
#define IOC_CMD_TXPOWER_GET         (IOCTRL) (  2u )
#define IOC_CMD_SENS_SET            (IOCTRL) (  3u )
#define IOC_CMD_SENS_GET            (IOCTRL) (  4u )
#define IOC_CMD_RSSI_GET            (IOCTRL) (  5u )
#define IOC_CMD_CCA_GET             (IOCTRL) (  6u )
#define IOC_CMD_ANT_DIV_SET         (IOCTRL) (  7u )
#define IOC_CMD_RF_SWITCH           (IOCTRL) (  8u )
#define IOC_CMD_SYNC_SET            (IOCTRL) (  9u )
#define IOC_CMD_SYNC_GET            (IOCTRL) ( 10u )

/**
 * @}
 */

#define IOC_VAL_SYNC_SP             (RADIO_VAL) ( 0x930Bu )
#define IOC_VAL_SYNC_DATA           (RADIO_VAL) ( 0x51DEu )

/**
 * @brief   Radio transceiver driver API
 */
typedef struct radio_drv_api    RADIO_DRV_API;

struct radio_drv_api {
    char     *Name;

    void    (*Init ) (s_ns_t *p_ns, RADIO_ERR *p_err);

    void    (*On   ) (RADIO_ERR *p_err);                                        /*!< Open the driver        */

    void    (*Off  ) (RADIO_ERR *p_err);                                        /*!< Close the driver       */

    void    (*Send ) (const void *p_payload, uint8_t len, RADIO_ERR *p_err);    /*!< Write data to radio    */

    void    (*Recv ) (void *p_buf, uint8_t len, RADIO_ERR *p_err);              /*!< Read data from radio   */

    void    (*Ioctl) (IOCTRL cmd, RADIO_VAL *p_val, RADIO_ERR *p_err);          /*!< Input/Output control   */
};


#endif /* RADIO_PRESENT */
