/**
 * @file	  llframer.c
 * @date	  28.05.2016
 * @author 	Phuong Nguyen
 */

#include "emb6.h"
#include "framer_802154.h"
#include "framer_smartmac.h"
#include "phy_framer_802154.h"
#include "packetbuf.h"
#include "linkaddr.h"
#include "crc.h"
#include "logger.h"
#include "llframer.h"


uint16_t llframe_parse(llframe_attr_t *p_frame, uint8_t *p_buf, uint16_t len) {
  uint8_t *p_mhr;
  uint8_t frameType;
  uint8_t destAddrMode;
  uint16_t numRemBytes;
  uint16_t pktLen;

  /* set all attributes to zero */
  memset(p_frame, 0, sizeof(*p_frame));

  /* PHR */
  pktLen = phy_framer802154_getPktLen(p_buf, len);
  if (pktLen == 0) {
    /* invalid length */
    return 0;
  }

  p_frame->tot_len = PHY_HEADER_LEN + pktLen;
  p_frame->crc_offset = PHY_HEADER_LEN;
  if ((p_frame->tot_len < LLFRAME_MIN_NUM_RX_BYTES) ||
      (p_frame->tot_len > LLFRAME_MAX_NUM_RX_BYTES)) {
    /* invalid length */
    return 0;
  }

#if NETSTK_CFG_IEEE_802154G_EN
  uint16_t phr;

  /* achieve PHY header */
  phr = (p_buf[0] << 8) | (p_buf[1]);
  if (PHY_PSDU_CRC16(phr)) {
    p_frame->crc_len = 2;
  }
  else {
    p_frame->crc_len = 4;
  }
#else
  p_frame->crc_len = 2;
#endif

  /* MHR */
  p_mhr = &p_buf[PHY_HEADER_LEN];

  /* frame type filtering */
  frameType = p_mhr[0] & 0x07;
  if ((frameType != FRAME802154_DATAFRAME) &&
      (frameType != FRAME802154_ACKFRAME) &&
      (frameType != FRAME802154_CMDFRAME) &&
      (frameType != SMARTMAC_FRAME_STROBE)) {
    /* unsupported frame types */
    return 0;
  }

  p_frame->is_ack_required = (p_mhr[0] & 0x20) >> 5;
  p_frame->seq_no = p_mhr[2];
  p_frame->min_addr_len = 2;

  /* destination address and destination PAN ID (2) */
  destAddrMode = (p_mhr[1] >> 2) & 3;
  if (destAddrMode == FRAME802154_SHORTADDRMODE) {
    p_frame->min_addr_len += 2;
  } else if (destAddrMode == FRAME802154_LONGADDRMODE) {
    p_frame->min_addr_len += 8;
  } else {
    /* destination address is not present */
    p_frame->min_addr_len = 0;
  }

  /* return number of remaining bytes without checksum */
  numRemBytes = p_frame->tot_len - LLFRAME_MIN_NUM_RX_BYTES;
  return numRemBytes;
}


uint8_t llframe_createAck(llframe_attr_t *p_frame, uint8_t *p_buf, uint16_t len) {
  uint8_t *p = p_buf;
  uint8_t *p_mhr = &p_buf[PHY_HEADER_LEN];
  uint8_t ack_len;
  uint32_t ack_crc;

  /* PHR */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  if (p_frame->crc_len == 2) {
    /* 16-bit CRC */
    *p++ = 0x10;
  }
  else {
    *p++ = 0x00;
  }
#endif


  /* PHR */
  if (p_frame->crc_len == 2) {
    *p++ = 0x05;
  }
  else {
    *p++ = 0x07;
  }

  /* MHR
  * TODO missing implementation Strobe ACK
  */
  *p++ = 0x02;
  *p++ = 0x10;
  *p++ = p_frame->seq_no;

  /* MFR */
  if (p_frame->crc_len == 2) {
    /* 16-bit CRC */
    ack_crc = crc_16_calc(p_mhr, 3);
    *p++ = (ack_crc & 0xFF00u) >> 8;
    *p++ = (ack_crc & 0x00FFu);
  }
  else {
    /* 32-bit CRC */
    ack_crc = crc_32_calc(p_mhr, 3);
    *p++ = (ack_crc & 0xFF000000u) >> 24;
    *p++ = (ack_crc & 0x00FF0000u) >> 16;
    *p++ = (ack_crc & 0x0000FF00u) >> 8;
    *p++ = (ack_crc & 0x000000FFu);
  }

  /* compute length of ACK */
  ack_len = p - p_buf;

  return ack_len;
}


uint8_t llframe_addrFilter(llframe_attr_t *p_frame, uint8_t *p_buf, uint16_t len) {
  uint8_t ix;
  uint8_t *p;
  uint8_t destAddr[8];
  uint8_t destAddrLen;
  uint16_t destPANId;
  packetbuf_attr_t dev_pan_id;

  if (p_frame->min_addr_len > 0) {
    /* set destination addressing fields to zero */
    destPANId = 0;
    memset(destAddr, 0, sizeof(destAddr));

    /* verify destination PAN Id */
    p = &p_buf[PHY_HEADER_LEN + 3];
    destPANId = p[0] | (p[1] << 8);

    /* is destination PAN ID a broadcast ID */
    if (destPANId == FRAME802154_BROADCASTPANDID) {
      /* then frame is always accepted */
      return TRUE;
    }

    /* is destination PAN Id different from source PAN Id? */
    dev_pan_id = packetbuf_attr(PACKETBUF_ATTR_MAC_PAN_ID);
    if (destPANId != dev_pan_id) {
      /* then discard the frame */
      TRACE_LOG_ERR("+++ LLFRAMER: invalid destPANId %04x", destPANId);
      return FALSE;
    }

    /* advance pointer by length of PAN Id */
    p += 2;

    /* verify destination address */
    destAddrLen = p_frame->min_addr_len - 2;
    if (destAddrLen > 0) {
      /* read destination address */
      for (ix = 0; ix < destAddrLen; ix++) {
        destAddr[7 - ix] = p[ix];
      }

      /* is destination address a short address? */
      if (destAddrLen == 2) {
        packetbuf_attr_t macShortAddr, destShortAddr;
        macShortAddr = packetbuf_attr(PACKETBUF_ATTR_MAC_SHORT_ADDR);
        destShortAddr = destAddr[7] | (destAddr[6] << 8);

        /* is destination address a broadcast address? */
        if (destShortAddr == FRAME802154_BROADCASTADDR) {
          /* then frame is accepted */
          return TRUE;
        }
        else if (macShortAddr == destShortAddr) {
          return TRUE;
        }
        else {
          /* then frame is discarded */
          TRACE_LOG_MAIN("<LLFRAMER> invalid destShortAddr %02x%02x", destAddr[7], destAddr[6]);
          return FALSE;
        }
      }
      /* is destination address a long address? */
      else {
        /* does destination address match device source address */
        if ((linkaddr_cmp((linkaddr_t *) destAddr, &linkaddr_node_addr)) == 1) {
          /* then frame is accepted */
          return TRUE;
        } else {
          /* then frame is discarded */
          TRACE_LOG_MAIN("+++ LLFRAMER: invalid destLongAddr");
          return FALSE;
        }
      }
    }
  }
  return TRUE;
}

uint32_t llframe_crcInit(llframe_attr_t *p_frame) {
  if (p_frame->crc_len == 2) {
    return CRC16_INIT;
  } else {
    return CRC32_INIT;
  }
}

uint32_t llframe_crcUpdate(llframe_attr_t *p_frame, uint8_t *p_data, uint16_t len, uint32_t curCRC) {
  if (p_frame->crc_len == 2) {
    return crc_16_updateN(curCRC, p_data, len);
  }
  else {
    return crc_32_updateN(curCRC, p_data, len);
  }
}

uint32_t llframe_crcFinal(llframe_attr_t *p_frame, uint32_t curCRC) {
  if (p_frame->crc_len == 4) {
    uint16_t len;
    len = p_frame->tot_len - p_frame->crc_len - PHY_HEADER_LEN;
    if (len < 4) {
      curCRC = crc_32_update(curCRC, 0x00);
    }
    curCRC ^= CRC32_INIT;
  }

  return curCRC;
}


uint8_t llframe_crcFilter(llframe_attr_t *p_frame, uint32_t actCRC, uint8_t *p_expCRC, uint16_t len) {
  uint32_t expCRC = 0;
  uint8_t is_matched;

  if (p_frame->crc_len == 2) {
    /* 16-bit CRC */
    expCRC = (p_expCRC[0] << 8) |
             (p_expCRC[1]);
  } else {
    /* 32-bit CRC */
    expCRC = ((uint32_t)p_expCRC[0] << 24) |
             ((uint32_t)p_expCRC[1] << 16) |
             ((uint32_t)p_expCRC[2] <<  8) |
             ((uint32_t)p_expCRC[3]);
  }

  is_matched = (expCRC == actCRC);
  return is_matched;
}

