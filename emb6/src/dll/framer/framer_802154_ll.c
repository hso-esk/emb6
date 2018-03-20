/**
 * @file	  llframer.c
 * @date	  28.05.2016
 * @author 	Phuong Nguyen
 */

#include "emb6.h"
#include "framer_802154.h"
#include "framer_smartmac.h"
#include "framer_802154_ll.h"
#include "phy_framer_802154.h"
#include "packetbuf.h"
#include "linkaddr.h"
#include "crc.h"
#include "logger.h"


uint16_t framer802154ll_parse(framer802154ll_attr_t *p_frame, uint8_t *p_buf, uint16_t len) {
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
  if ((p_frame->tot_len < FRAMER802154LL_MIN_NUM_RX_BYTES) ||
      (p_frame->tot_len > FRAMER802154LL_MAX_NUM_RX_BYTES)) {
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
      (frameType != FRAME802154_BEACONFRAME) &&
      (frameType != SMARTMAC_FRAME_STROBE)) {
    /* unsupported frame types */
    return 0;
  }

  // check frame version and ACK-required fields
  uint8_t frame_version = (p_mhr[1] >> 4) & 3;
  uint8_t seq_no_compressed = p_mhr[1] & 1;

  // Auto-ACK feature of radio is currently not supported for frame
  // version 0b10 with sequence number suppressed
  if (FRAME802154_IEEE802154E_2012 == frame_version) {
    if (1 == seq_no_compressed) {
      p_frame->is_ack_required = 0;
      p_frame->seq_no = 0;
    }
    else {
      p_frame->is_ack_required = (p_mhr[0] & 0x20) >> 5;
      p_frame->seq_no = p_mhr[2];
    }
  }
  else if ((FRAME802154_IEEE802154_2003 == frame_version)
          || (FRAME802154_IEEE802154_2006 == frame_version)) {
    // IEEE Std 802.15.4-2015, 7.2.1.6
    // "If the frame version field is 0b00 or 0b01, the sequence number
    // suppression field shall zero". In other word, sequence number
    // shall be present
    p_frame->is_ack_required = (p_mhr[0] & 0x20) >> 5;
    p_frame->seq_no = p_mhr[2];
  }
  else {
    /* unsupported frame version */
    return 0;
  }

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
  numRemBytes = p_frame->tot_len - FRAMER802154LL_MIN_NUM_RX_BYTES;
  return numRemBytes;
}


uint8_t framer802154ll_createAck(framer802154ll_attr_t *p_frame, uint8_t *p_buf, uint16_t len) {
  uint8_t *p = p_buf;
  uint8_t *p_mhr = &p_buf[PHY_HEADER_LEN];
  uint8_t ack_len;
  uint32_t ack_crc = 0;

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
#if !defined(NETSTK_SUPPORT_HW_CRC)
    ack_crc = crc_16_calc(p_mhr, 3);
#endif /* #if !defined(NETSTK_SUPPORT_HW_CRC) */
    *p++ = (ack_crc & 0xFF00u) >> 8;
    *p++ = (ack_crc & 0x00FFu);
  }
  else {
    /* 32-bit CRC */
#if !defined(NETSTK_SUPPORT_HW_CRC) && (NETSTK_CFG_IEEE_802154G_EN == TRUE)
    ack_crc = crc_32_calc(p_mhr, 3);
#endif /* #if !defined(NETSTK_SUPPORT_HW_CRC) */
    *p++ = (ack_crc & 0xFF000000u) >> 24;
    *p++ = (ack_crc & 0x00FF0000u) >> 16;
    *p++ = (ack_crc & 0x0000FF00u) >> 8;
    *p++ = (ack_crc & 0x000000FFu);
  }

  /* compute length of ACK */
  ack_len = p - p_buf;

  return ack_len;
}


uint8_t framer802154ll_addrFilter(framer802154ll_attr_t *p_frame, uint8_t *p_buf, uint16_t len) {
  uint8_t ix;
  uint8_t *p;
  uint8_t destAddr[8];
  uint8_t destAddrLen;
  uint16_t destPANId;
  packetbuf_attr_t dev_pan_id;

  uint8_t *p_mhr = &p_buf[PHY_HEADER_LEN];
  uint8_t frame_type = p_mhr[0] & 0x07;
  uint8_t seq_no_suppressed = p_mhr[1] & 1;

  if (p_frame->min_addr_len > 0) {
    /* set destination addressing fields to zero */
    destPANId = 0;
    memset(destAddr, 0, sizeof(destAddr));

    /* verify destination PAN Id */
    if (1 == seq_no_suppressed)
      p = &p_buf[PHY_HEADER_LEN + 2]; // sequence number is suppressed
    else
      p = &p_buf[PHY_HEADER_LEN + 3]; // sequence number is present

    destPANId = p[0] | (p[1] << 8);
    dev_pan_id = packetbuf_attr(PACKETBUF_ATTR_MAC_PAN_ID);

    /* is destination PAN ID a broadcast ID */
    if (destPANId == FRAME802154_BROADCASTPANDID) {
      if (FRAME802154_BEACONFRAME == frame_type) {
        // IEEE Std 802.15.4-2015, 6.7.2 Reception and rejection, page 102
        // "If the frame type indicates that the frame is a beacon frame, the
        // source PAN ID shall match macPanId unless macPanId is equal to the
        // broadcast PAN ID, in which case the beacon frame shall be accepted
        // regardless of the source PAN ID"
        uint16_t srcPANId = p[2] | (p[3] << 8);
        if ((dev_pan_id != FRAME802154_BROADCASTPANDID) && (dev_pan_id != srcPANId)) {
          return FALSE;
        }
      }

      /* frame is accepted */
      return TRUE;
    }

    /* is destination PAN Id different from source PAN Id? */
    if ((dev_pan_id != FRAME802154_BROADCASTPANDID) &&
        (destPANId != dev_pan_id))
    {
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

#if !defined(NETSTK_SUPPORT_HW_CRC)
uint32_t framer802154ll_crcInit(framer802154ll_attr_t *p_frame) {
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  if (p_frame->crc_len == 2) {
#endif
    return CRC16_INIT;
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  } else {
    return CRC32_INIT;
  }
#endif
}

uint32_t framer802154ll_crcUpdate(framer802154ll_attr_t *p_frame, uint8_t *p_data, uint16_t len, uint32_t curCRC) {
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  if (p_frame->crc_len == 2) {
#endif
    return crc_16_updateN(curCRC, p_data, len);
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  } else {
    return crc_32_updateN(curCRC, p_data, len);
  }
#endif
}

uint32_t framer802154ll_crcFinal(framer802154ll_attr_t *p_frame, uint32_t curCRC) {
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  if (p_frame->crc_len == 4) {
    uint16_t len;
    len = p_frame->tot_len - p_frame->crc_len - PHY_HEADER_LEN;
    if (len < 4) {
      curCRC = crc_32_update(curCRC, 0x00);
    }
    curCRC ^= CRC32_INIT;
  }
#endif
  return curCRC;
}


uint8_t framer802154ll_crcFilter(framer802154ll_attr_t *p_frame, uint32_t actCRC, uint8_t *p_expCRC, uint16_t len) {
  uint32_t expCRC = 0;
  uint8_t is_matched;

#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  if (p_frame->crc_len == 2) {
#endif
    /* 16-bit CRC */
    expCRC = ((p_expCRC[0] << 8) & 0xFF00) |
             ((p_expCRC[1]     ) & 0x00FF);
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  } else {
    /* 32-bit CRC */
    expCRC = (((uint32_t)p_expCRC[0] << 24) & 0xFF000000) |
             (((uint32_t)p_expCRC[1] << 16) & 0x00FF0000) |
             (((uint32_t)p_expCRC[2] <<  8) & 0x0000FF00) |
             (((uint32_t)p_expCRC[3]      ) & 0x000000FF);
  }
#endif

  is_matched = (expCRC == actCRC);
  return is_matched;
}
#endif /* #if !defined(NETSTK_SUPPORT_HW_CRC) */
