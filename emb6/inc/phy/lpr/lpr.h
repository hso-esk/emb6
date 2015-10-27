/**
 * @file    lpr.h
 * @date    Aug 19, 2015
 * @author  PN
 */

#ifndef LPR_PRESENT
#define LPR_PRESENT


typedef struct lpr_framer_api      LPR_FRAMER_DRV;

/**
 * @brief   Asynchronous Power Saving Scheme Framer API
 */
struct lpr_framer_api
{
    char         *Name;

    void        (*Init        )(NETSTK_ERR *p_err);

    void        (*Deinit      )(NETSTK_ERR *p_err);

    uint8_t*    (*Create      )(uint8_t frame_type, uint16_t *p_len, uint32_t *p_delay, NETSTK_ERR *p_err);

    void        (*Parse       )(uint8_t *p_pkt, uint16_t len, NETSTK_ERR *p_err);
};



#define LPR_INVALID_DEV_ID                 (NETSTK_DEV_ID)( 0xffff )
#define LPR_IS_PENDING_TX()                (LPRDstId < LPR_INVALID_DEV_ID)

/**
 * @addtogroup  LPR_FRAME_TYPES    APSS frame types
 * @note        APSS frame types use reserved values of frame types specified
 *              in IEEE802.15.4
 * @{
 */
typedef uint8_t LPR_FRAME_TYPE;

#define LPR_FRAME_TYPE_STROBE              (LPR_FRAME_TYPE) ( 0x14 )
#define LPR_FRAME_TYPE_SACK                (LPR_FRAME_TYPE) ( 0x15 )
#define LPR_FRAME_TYPE_BROADCAST           (LPR_FRAME_TYPE) ( 0x16 )

/**
 * @}
 */


#if     LPR_CFG_LOOSE_SYNC_EN
typedef struct lpr_pwron_tbl_entry     LPR_PWRON_TBL_ENTRY;

/**
 * @brief   Power-On table structure declaration
 */
struct lpr_pwron_tbl_entry
{
    uint32_t    LastWakeup;     /*!< Last wake-up record                            */

    uint16_t    DestId;         /*!< Destination ID                                 */

    uint16_t    StrobeSentQty;  /*!< Quantity of sent strobes as waking-up signal   */
};

extern LPR_PWRON_TBL_ENTRY LPRPwrOnTbl[LPR_CFG_PWRON_TBL_SIZE];
#endif

extern NETSTK_DEV_ID LPRSrcId;
extern NETSTK_DEV_ID LPRDstId;
extern LPR_FRAMER_DRV XMACFramer;
extern LPR_FRAMER_DRV SmartMACFramer;

#endif /* LPR_PRESENT */
