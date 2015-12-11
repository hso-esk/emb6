/**
 * @file    phy_framer_802154.c
 * @date    01.12.2015
 * @author  PN
 */

#include "emb6.h"
#include "phy_framer_802154.h"

uint16_t phy_framer802154_getPktLen(uint8_t *p_data, uint16_t len)
{
    if (len < PHY_HEADER_LEN) {
        return 0;
    }

    uint16_t psdu_len = 0;

#if NETSTK_CFG_IEEE_802154G_EN
    uint16_t phr;

    phr = (p_data[0] << 8) | (p_data[1]);
    psdu_len = phr & 0x07FF;
#else
    psdu_len = p_data[0];
#endif

    return psdu_len;
}
