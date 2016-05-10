/**
 * @file	  framer_smartmac.c
 * @date	  19.04.2016
 * @author 	Phuong Nguyen
 */

#include "emb6.h"
#include "rt_tmr.h"
#include "framer_smartmac.h"


uint16_t frame_smartmac_create(frame_smartmac_st *p_frame, uint8_t *p_buf)
{
  uint16_t len = 0;
  uint8_t *p = p_buf;

  if ((p_frame == NULL) || (p_buf == NULL)) {
    return 0;
  }

  /* write frame control fields */
  *p++ = ((p_frame->type & 7)) |
         ((p_frame->ack_required & 1) << 5);

  *p++ = 0b00100110; /* frame version 2006, short address modes for both destination and source */

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
  return len;
}


uint16_t frame_smartmac_parse(uint8_t *p_buf, uint16_t len, frame_smartmac_st *p_frame)
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
