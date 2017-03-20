/**
 * @file	  framer_smartmac.h
 * @date	  19.04.2016
 * @author 	Phuong Nguyen
 */

#ifndef FRAMER_SMARTMAC_H_
#define FRAMER_SMARTMAC_H_

#define SMARTMAC_FRAME_STROBE                 (uint8_t)( 0x04 )
#define SMARTMAC_FRAME_STROBE_ACK             (uint8_t)( 0x05 )

typedef struct s_frame_smartmac frame_smartmac_st;

struct s_frame_smartmac {
  uint8_t type;
  uint8_t ack_required;
  uint8_t counter;
  uint16_t pan_id;
  uint16_t dest_addr;
  uint16_t src_addr;
};

uint8_t framer_smartmac_getStrobeLen(void);
uint16_t framer_smartmac_create(frame_smartmac_st *p_frame, uint8_t *p_buf);
uint16_t framer_smartmac_parse(uint8_t *p_buf, uint16_t len, frame_smartmac_st *p_frame);


#endif /* FRAMER_SMARTMAC_H_ */
