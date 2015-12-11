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
 * @file    mac_ule.h
 * @date    Aug 19, 2015
 * @author  PN
 */

#ifndef MAC_ULE_PRESENT
#define MAC_ULE_PRESENT


typedef struct mac_ule_framer_api      s_nsMacUleFramerDrv_t;

/**
 * @brief   Ultra-Low-Energy MAC framer API structure declaration
 */
struct mac_ule_framer_api
{
    char         *Name;

    void        (*Init  )(e_nsErr_t *p_err);

    void        (*Deinit)(e_nsErr_t *p_err);

    uint8_t*    (*Create)(uint8_t frame_type, uint16_t *p_len, uint32_t *p_delay, e_nsErr_t *p_err);

    void        (*Parse )(uint8_t *p_frame_type, uint8_t *p_pkt, uint16_t len, e_nsErr_t *p_err);
};


#define MAC_ULE_DEV_ID_BROADCAST            (NETSTK_DEV_ID)( 0xFFFF )
#define MAC_ULE_ID_INVALID                  (NETSTK_DEV_ID)( 0x0000 )
#define MAC_ULE_IS_PENDING_TX()             (NetstkDstId > MAC_ULE_ID_INVALID)

/**
 * @addtogroup  LPR_FRAME_TYPES    APSS frame types
 * @note        APSS frame types use reserved values of frame types specified
 *              in IEEE802.15.4
 * @{
 */
typedef uint8_t MAC_ULE_FRAME_TYPE;

#define MAC_ULE_FRAME_TYPE_STROBE           (MAC_ULE_FRAME_TYPE)( 0x14 )
#define MAC_ULE_FRAME_TYPE_SACK             (MAC_ULE_FRAME_TYPE)( 0x15 )
#define MAC_ULE_FRAME_TYPE_BROADCAST        (MAC_ULE_FRAME_TYPE)( 0x16 )

/**
 * @}
 */


#if MAC_ULE_CFG_LOOSE_SYNC_EN
typedef struct mac_ule_pwron_tbl_entry     s_nsMacUlePwrOnTblEntry_t;

/**
 * @brief   Power-On table structure declaration
 */
struct mac_ule_pwron_tbl_entry
{
    uint32_t    LastWakeup;     /*!< Last wake-up record                            */

    uint16_t    DestId;         /*!< Destination ID                                 */

    uint16_t    StrobeSentQty;  /*!< Quantity of sent strobes as waking-up signal   */
};

extern s_nsMacUlePwrOnTblEntry_t MacUlePwrOnTbl[MAC_ULE_CFG_PWRON_TBL_SIZE];
#endif

extern NETSTK_DEV_ID NetstkSrcId;
extern NETSTK_DEV_ID NetstkDstId;
extern s_nsMacUleFramerDrv_t SmartMACFramer;

#endif /* MAC_ULE_PRESENT */
