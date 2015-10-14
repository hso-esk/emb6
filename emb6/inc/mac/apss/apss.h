/*
 * apss.h
 *
 *  Created on: Aug 19, 2015
 *      Author: phuongnguyen
 */

#ifndef APSS_H_
#define APSS_H_

#define APSS_CFG_REFACTOR_EN                             ( 1u )


#define APSS_INVALID_DEV_ID                 (STK_DEV_ID)( 0xffff )
#define APSS_IS_PENDING_TX()                (APSSDstId < APSS_INVALID_DEV_ID)

/**
 * @addtogroup  APSS_FRAME_TYPES    APSS frame types
 * @note        APSS frame types use reserved values of frame types specified
 *              in IEEE802.15.4
 * @{
 */
typedef uint8_t APSS_FRAME_TYPE;

#define APSS_FRAME_TYPE_STROBE              (APSS_FRAME_TYPE) ( 0x14 )
#define APSS_FRAME_TYPE_SACK                (APSS_FRAME_TYPE) ( 0x15 )
#define APSS_FRAME_TYPE_BROADCAST           (APSS_FRAME_TYPE) ( 0x16 )

/**
 * @}
 */


#if     APSS_CFG_LOOSE_SYNC_EN
typedef struct apss_pwron_tbl_entry     APSS_PWRON_TBL_ENTRY;

/**
 * @brief   Power-On table structure declaration
 */
struct apss_pwron_tbl_entry
{
    uint32_t    LastWakeup;     /*!< Last wake-up record                            */

    uint16_t    DestId;         /*!< Destination ID                                 */

    uint16_t    StrobeSentQty;  /*!< Quantity of sent strobes as waking-up signal   */
};

extern APSS_PWRON_TBL_ENTRY APSSPwrOnTbl[APSS_CFG_PWRON_TBL_SIZE];
#endif

extern STK_DEV_ID APSSSrcId;
extern STK_DEV_ID APSSDstId;

#endif /* APSS_H_ */
