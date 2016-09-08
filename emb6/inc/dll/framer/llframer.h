/**
 * @file	  llframer.h
 * @date	  28.05.2016
 * @author 	Phuong Nguyen
 */

#ifndef LLFRAMER_H_
#define LLFRAMER_H_


#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
#define LLFRAME_MIN_NUM_RX_BYTES           5U   // PHR + MHR.FCF + MHR.SEQ
#define LLFRAME_MAX_NUM_RX_BYTES        2047U   // (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)
#else
#define LLFRAME_MIN_NUM_RX_BYTES           4U   // PHR + MHR.FCF + MHR.SEQ
#define LLFRAME_MAX_NUM_RX_BYTES         128U   // (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)
#endif

/* The frame attributes are public */
typedef struct s_llframe_attr {
  uint16_t tot_len;
  uint8_t min_addr_len;
  uint8_t is_ack_required;
  uint8_t seq_no;
  uint8_t crc_len;
  uint8_t crc_offset;
} llframe_attr_t;

uint16_t llframe_parse(llframe_attr_t *p_frame, uint8_t *p_buf, uint16_t len);
uint8_t llframe_createAck(llframe_attr_t *p_frame, uint8_t *p_buf, uint16_t len);
uint8_t llframe_addrFilter(llframe_attr_t *p_frame, uint8_t *p_buf, uint16_t len);
uint32_t llframe_crcInit(llframe_attr_t *p_frame);
uint32_t llframe_crcUpdate(llframe_attr_t *p_frame, uint8_t *p_data, uint16_t len, uint32_t curCRC);
uint32_t llframe_crcFinal(llframe_attr_t *p_frame, uint32_t curCRC);
uint8_t llframe_crcFilter(llframe_attr_t *p_frame, uint32_t actCRC, uint8_t *p_expCRC, uint16_t len);

#endif /* LLFRAMER_H_ */
