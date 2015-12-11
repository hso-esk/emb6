/**
 * @file    phy_802154g_framer.h
 * @date    01.12.2015
 * @author  PN
 */

#ifndef PHY_FRAMER154_PRESENT
#define PHY_FRAMER154_PRESENT


#if NETSTK_CFG_IEEE_802154G_EN
#define PHY_HEADER_LEN                  (uint16_t)(    2u )
#define PHY_PSDU_MAX                    (uint16_t)( 2047u )
#else
#define PHY_HEADER_LEN                  (uint16_t)(    1u )
#define PHY_PSDU_MAX                    (uint16_t)(  127u )
#endif

uint16_t phy_framer802154_getPktLen(uint8_t *p_data, uint16_t len);

#endif /* PHY_FRAMER154_PRESENT */
