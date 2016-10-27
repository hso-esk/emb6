/**
 * @file	  framer_802154_ll.h
 * @date	  28.05.2016
 * @author 	Phuong Nguyen
 */

#ifndef FRAMER_802154_LL_H
#define FRAMER_802154_LL_H


#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
#define FRAMER802154LL_MIN_NUM_RX_BYTES           5U   // PHR + MHR.FCF + MHR.SEQ
#define FRAMER802154LL_MAX_NUM_RX_BYTES        2049U   // (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)
#else
#define FRAMER802154LL_MIN_NUM_RX_BYTES           4U   // PHR + MHR.FCF + MHR.SEQ
#define FRAMER802154LL_MAX_NUM_RX_BYTES         128U   // (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)
#endif

/* The frame attributes are public */
typedef struct s_framer802154ll_attr {
  uint16_t tot_len;
  uint8_t min_addr_len;
  uint8_t is_ack_required;
  uint8_t seq_no;
  uint8_t crc_len;
  uint8_t crc_offset;
} framer802154ll_attr_t;

uint16_t framer802154ll_parse(framer802154ll_attr_t *p_frame, uint8_t *p_buf, uint16_t len);
uint8_t framer802154ll_createAck(framer802154ll_attr_t *p_frame, uint8_t *p_buf, uint16_t len);
uint8_t framer802154ll_addrFilter(framer802154ll_attr_t *p_frame, uint8_t *p_buf, uint16_t len);
uint32_t framer802154ll_crcInit(framer802154ll_attr_t *p_frame);
uint32_t framer802154ll_crcUpdate(framer802154ll_attr_t *p_frame, uint8_t *p_data, uint16_t len, uint32_t curCRC);
uint32_t framer802154ll_crcFinal(framer802154ll_attr_t *p_frame, uint32_t curCRC);
uint8_t framer802154ll_crcFilter(framer802154ll_attr_t *p_frame, uint32_t actCRC, uint8_t *p_expCRC, uint16_t len);

#endif /* FRAMER_802154_LL_H */
