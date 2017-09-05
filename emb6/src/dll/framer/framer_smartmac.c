/**
 * @file	  framer_smartmac.c
 * @date	  19.04.2016
 * @author 	Phuong Nguyen
 */

#include "emb6.h"
#include "board_conf.h"
#include "phy_framer_802154.h"
#include "framer_smartmac.h"
#include "packetbuf.h"
#include "crc.h"

/* MAC header length of SmartMAC's strobe is calculated as follow
 * FCF(2) + SEQ(1) + PANID(2) + DST.ADDR(2) + SRC.ADDR(2) = 11 */
#define SMARTMAC_STROBE_MAC_HDR_LEN         ( 11u )

uint8_t framer_smartmac_getStrobeLen(void) {
  return (PHY_HEADER_LEN + SMARTMAC_STROBE_MAC_HDR_LEN + packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN));
}

uint16_t framer_smartmac_create(frame_smartmac_st *p_frame, uint8_t *p_buf)
{
  uint16_t len = 0;
  uint8_t *p = p_buf;

  if ((p_frame == NULL) || (p_buf == NULL)) {
    return 0;
  }

  /* write frame control fields */
  *p++ = ((p_frame->type & 7)) |
         ((p_frame->ack_required & 1) << 5);

  *p++ = 0b10011000; /* frame version 2006, short address modes for both destination and source */

  /* write sequence number */
  *p++ = p_frame->counter;

  /* write PAN ID */
  *p++ = (p_frame->pan_id     );
  *p++ = (p_frame->pan_id >> 8);

  /* write destination address */
  memcpy(p, &p_frame->dest_addr, 2);
  p += 2;

  /* write source address */
  memcpy(p, &p_frame->src_addr, 2);
  p += 2;

  /* compute frame length */
  len = p - p_buf;

#if !defined(NETSTK_SUPPORT_HW_CRC)
  uint8_t *p_mfr;
  uint32_t fcs;
  packetbuf_attr_t fcs_len;

  /* write footer */
  p_mfr = p;
  fcs_len = packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);
  if (fcs_len == 4) {
    /* 32-bit CRC */
    fcs = crc_32_calc(p_buf, len);
    p_mfr[0] = (fcs & 0xFF000000u) >> 24;
    p_mfr[1] = (fcs & 0x00FF0000u) >> 16;
    p_mfr[2] = (fcs & 0x0000FF00u) >> 8;
    p_mfr[3] = (fcs & 0x000000FFu);
  } else {
    /* 16-bit CRC */
    fcs = crc_16_calc(p_buf, len);
    p_mfr[0] = (fcs & 0xFF00u) >> 8;
    p_mfr[1] = (fcs & 0x00FFu);
  }
  len += fcs_len;
#endif /* #if !defined(NETSTK_SUPPORT_HW_CRC) */

  return len;
}


uint16_t framer_smartmac_parse(uint8_t *p_buf, uint16_t len, frame_smartmac_st *p_frame)
{
  uint16_t hdr_len = 0;
  uint8_t *p = p_buf;

  if ((p_frame == NULL) || (p_buf == NULL)) {
    return 0;
  }

  /* set all fields to zero */
  memset(p_frame, 0, sizeof(*p_frame));

  /* read frame control fields */
  p_frame->type = p[0] & 7;
  p_frame->ack_required = (p[0] >> 5) & 1;
  p += 2;

  /* read counter */
  p_frame->counter = *p;
  p += 1;

  /* read PAN ID */
  memcpy(&p_frame->pan_id, p, 2);
  p += 2;

  /* read destination address */
  memcpy(&p_frame->dest_addr, p, 2);
  p += 2;

  /* read source address */
  memcpy(&p_frame->src_addr, p, 2);
  p += 2;

  /* compute frame header length */
  hdr_len = p - p_buf;
  return hdr_len;
}
