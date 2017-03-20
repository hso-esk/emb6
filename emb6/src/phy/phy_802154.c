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
 * @file    phy_null.c
 * @date    16.10.2015
 * @author  PN
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"
#include "board_conf.h"
#include "phy_framer_802154.h"
#include "packetbuf.h"
#include "rt_tmr.h"
#include "crc.h"

#define     LOGGER_ENABLE        LOGGER_PHY
#include    "logger.h"


/*
********************************************************************************
*                                   LOCAL DEFINES
********************************************************************************
*/


/*
********************************************************************************
*                          LOCAL FUNCTION DECLARATIONS
********************************************************************************
*/
static void phy_init(void *p_netstk, e_nsErr_t *p_err);
static void phy_on(e_nsErr_t *p_err);
static void phy_off(e_nsErr_t *p_err);
static void phy_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void phy_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void phy_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
static void phy_insertCrc(uint8_t *p_data, uint16_t len);
static uint16_t phy_crc16(uint8_t *p_data, uint16_t len);
static uint32_t phy_crc32(uint8_t *p_data, uint16_t len);
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */
static uint8_t *phy_insertHdr(uint8_t *p_data, uint16_t len);



/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t   *pphy_netstk;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsPHY_t phy_driver_802154 = {
 "PHY 802154",
  phy_init,
  phy_on,
  phy_off,
  phy_send,
  phy_recv,
  phy_ioctl
};

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

/**
 * @brief   Initialize driver
 *
 * @param   p_netstk    Pointer to netstack structure
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_init(void *p_netstk, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if (p_netstk == NULL) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  /* store pointer to netstack structure */
  pphy_netstk = (s_ns_t *) p_netstk;

#if (EMB6_TEST_CFG_WOR_EN == TRUE)
  /* don't change default settings for WOR test */
#else
  /* set operation mode */
  pphy_netstk->rf->ioctrl(NETSTK_CMD_RF_OP_MODE_SET, &mac_phy_config.op_mode, p_err);

  /* set operation frequency channel number */
  pphy_netstk->rf->ioctrl(NETSTK_CMD_RF_CHAN_NUM_SET, &mac_phy_config.chan_num, p_err);
#endif

#if (NETSTK_CFG_WOR_EN == 1)
  uint8_t wor_en = TRUE;
  pphy_netstk->rf->ioctrl(NETSTK_CMD_RF_WOR_EN, &wor_en, p_err);
#endif

  /* set transmission power */
  pphy_netstk->rf->ioctrl(NETSTK_CMD_RF_TXPOWER_SET, &mac_phy_config.init_power, p_err);

  /* initialize PHY PIB attributes */
  packetbuf_attr_t symbol_period;
  packetbuf_attr_t symbol_per_octet;
  packetbuf_attr_t shr_duration;
  packetbuf_attr_t tx_timeout;

  if (mac_phy_config.modulation == MODULATION_2FSK50) {
    /* 2FSK, 50kbps => symbol period = 20us */
    symbol_period = 20;
    symbol_per_octet = 8;

    packetbuf_set_attr(PACKETBUF_ATTR_PHY_SYMBOL_PERIOD, symbol_period);
    packetbuf_set_attr(PACKETBUF_ATTR_PHY_SYMBOLS_PER_OCTET, symbol_per_octet);
    packetbuf_set_attr(PACKETBUF_ATTR_PHY_PREAMBLE_SYMBOL_LEN, mac_phy_config.preamble_len * symbol_per_octet);
    packetbuf_set_attr(PACKETBUF_ATTR_PHY_MAX_PACKET_SIZE, PHY_PSDU_MAX);
    packetbuf_set_attr(PACKETBUF_ATTR_PHY_TURNAROUND_TIME, (12 * symbol_period));

    /* SHR = preamble + 2-byte-SYNC */
    shr_duration = (mac_phy_config.preamble_len + 2) * symbol_per_octet * symbol_period;
    packetbuf_set_attr(PACKETBUF_ATTR_PHY_SHR_DURATION, shr_duration);

    /* set TX timeout in ticks */
    tx_timeout = (shr_duration + PHY_PSDU_MAX * symbol_per_octet * symbol_period) / bsp_getTRes();
    packetbuf_set_attr(PACKETBUF_ATTR_PHY_TXTIMEOUT, tx_timeout);

    /* set returned error */
    *p_err = NETSTK_ERR_NONE;
  }
  else {
    /* currently other modulation schemes are not supported by the driver */
    *p_err = NETSTK_ERR_FATAL;
  }
}

/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_on(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pphy_netstk->rf->on(p_err);
}

/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }
#endif

  pphy_netstk->rf->off(p_err);
}

/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((len == 0) ||
      (p_data == NULL)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

  uint8_t *p_pkt;
  uint16_t pkt_len;

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)
  /* insert MAC checksum */
  phy_insertCrc(p_data, len);
  pkt_len = len + mac_phy_config.fcs_len;
#else
  pkt_len = len;
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */

  /* insert PHY header */
  p_pkt = phy_insertHdr(p_data, pkt_len);
  pkt_len += PHY_HEADER_LEN;

#if LOGGER_ENABLE
  /*
   * Logging
   */
  uint16_t data_len = pkt_len;
  uint8_t *p_dataptr = p_pkt;
  LOG_RAW("PHY_TX: ");
  while (data_len--) {
    LOG_RAW("%02x", *p_dataptr++);
  }
  LOG_RAW("\r\n====================\r\n");
#endif


  /* Issue next lower layer to transmit the prepared frame */
  pphy_netstk->rf->send(p_pkt, pkt_len, p_err);
}

/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL) {
    return;
  }

  if ((len < PHY_HEADER_LEN ) || (p_data == NULL)) {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
#endif

#if LOGGER_ENABLE
  /*
   * Logging
   */
  uint16_t data_len = len;
  uint8_t *p_dataptr = p_data;
  LOG_RAW("\r\n====================\r\n");
  LOG_RAW("PHY_RX: ");
  while (data_len--) {
    LOG_RAW("%02x", *p_dataptr++);
  }
  LOG_RAW("\n\r");
#endif

  /*
  * Parse PHY header
  */
  uint8_t *p_PHY = p_data ;
  packetbuf_attr_t fcs_len;
  fcs_len = packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);

#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == FALSE)
  uint16_t psdu_len;
  p_data += PHY_HEADER_LEN;
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  uint16_t phr;

  /* achieve PHY header */
  phr = (p_PHY[0] << 8) | (p_PHY[1]);
  if (PHY_PSDU_CRC16(phr))
  psdu_len = (len - PHY_HEADER_LEN) - 2;
else
  psdu_len = (len - PHY_HEADER_LEN) - 4;
#else
  psdu_len = (len - PHY_HEADER_LEN) - fcs_len;
#endif
#else /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == FALSE) */
#if (NETSTK_CFG_IEEE_802154G_EN == TRUE)
  uint8_t crc_size;
  uint16_t phr, psdu_len;
  uint32_t crc_exp, crc_act;

  /* achieve PHY header */
  phr = (p_data[0] << 8) | (p_data[1]);

  /* verify frame length field */
  psdu_len = phr & 0x07FF;
  if (len != (PHY_HEADER_LEN + psdu_len)) {
    *p_err = NETSTK_ERR_BAD_FORMAT;
    return;
  }

  if ((psdu_len < PHY_PSDU_MIN(phr)) ||
      (psdu_len > PHY_PSDU_MAX )) {
    *p_err = NETSTK_ERR_BAD_FORMAT;
    return;
  }

  p_data += PHY_HEADER_LEN;

  /* verify CRC */
  crc_exp = 0;
  crc_act = 0;

  if (PHY_PSDU_CRC16(phr)) {
    /* 16-bit CRC was used in the received frame */
    crc_size = 2;
    psdu_len -= crc_size;

    /* obtain CRC of the received frame */
    memcpy(&crc_exp, &p_data[psdu_len], crc_size);
    crc_exp = ((crc_exp & 0x00FF) << 8) |
              ((crc_exp & 0xFF00) >> 8);

    /* calculated actual CRC */
    crc_act = phy_crc16(p_data, psdu_len);

  } else {
    /* 32-bit CRC was used in the received frame */
    crc_size = 4;
    psdu_len -= crc_size;

    /* obtain CRC of the received frame */
    memcpy(&crc_exp, &p_data[psdu_len], crc_size);
    crc_exp = ((crc_exp & 0x000000FF) << 24) |
              ((crc_exp & 0x0000FF00) <<  8) |
              ((crc_exp & 0x00FF0000) >>  8) |
              ((crc_exp & 0xFF000000) >> 24);

    /* calculated actual CRC */
    crc_act = phy_crc32(p_data, psdu_len);
  }

  if (crc_act != crc_exp) {
    *p_err = NETSTK_ERR_CRC;
    return;
  }
#else
  uint8_t psdu_len;
  uint16_t crc_exp, crc_act;

  /* verify frame length */
  psdu_len = *p_data;
  if (len != (PHY_HEADER_LEN + psdu_len)) {
    *p_err = NETSTK_ERR_BAD_FORMAT;
    return;
  }

  if ((psdu_len < PHY_PSDU_MIN()) ||
      (psdu_len > PHY_PSDU_MAX) ) {
    *p_err = NETSTK_ERR_BAD_FORMAT;
    return;
  }

  p_data += PHY_HEADER_LEN;

  /* verify CRC */
  psdu_len -= fcs_len;

  /* calculated actual CRC */
  crc_act = phy_crc16(p_data, psdu_len);

  /* obtain CRC of the received frame */
  memcpy(&crc_exp, &p_data[psdu_len], fcs_len);
  crc_exp = ((crc_exp & 0x00FF) << 8) |
            ((crc_exp & 0xFF00) >> 8);
  if (crc_act != crc_exp) {
    *p_err = NETSTK_ERR_CRC;
    return;
  }
#endif /* #if (NETSTK_CFG_IEEE_802154G_EN == TRUE) */
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == FALSE) */

  /* Inform the next higher layer */
  pphy_netstk->mac->recv(p_data, psdu_len, p_err);
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void phy_ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
  /* Set returned error code to default */
  *p_err = NETSTK_ERR_NONE;
  switch (cmd) {
    default:
      pphy_netstk->rf->ioctrl(cmd, p_val, p_err);
      break;
  }
}


/**
 * @brief   Insert PHY header
 */
static uint8_t * phy_insertHdr(uint8_t *p_data, uint16_t len)
{
  uint8_t *p_hdr;
  uint16_t hdr;

  /* get pointer to PHY header field */
  p_hdr = p_data - PHY_HEADER_LEN;

  /* compute header fields */
  hdr = len;

#if NETSTK_CFG_IEEE_802154G_EN
  uint8_t chksumLen;
  chksumLen = packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);
  if (chksumLen == 2) {
    hdr |= 0x1000u;
  }

  /* write PHY header */
  p_hdr[0] = (hdr & 0xFF00u) >> 8;
  p_hdr[1] = (hdr & 0x00FFu);

#else
  /* write the header */
  p_hdr[0] = hdr & 0x7F;
#endif

  return p_hdr;
}


#if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE)

/**
 * @brief   Insert PHY checksum
 */
static void phy_insertCrc(uint8_t *p_data, uint16_t len)
{
  uint8_t *p_crc;
  uint32_t crc = 0;
  packetbuf_attr_t fcs_len;

  /* get pointer to checksum field */
  p_crc = p_data + len;

  fcs_len = packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);
  if (fcs_len == 4) {
    crc = phy_crc32(p_data, len);
    p_crc[0] = (crc & 0xFF000000u) >> 24;
    p_crc[1] = (crc & 0x00FF0000u) >> 16;
    p_crc[2] = (crc & 0x0000FF00u) >> 8;
    p_crc[3] = (crc & 0x000000FFu);
  } else {
    crc = phy_crc16(p_data, len);
    p_crc[0] = (crc & 0xFF00u) >> 8;
    p_crc[1] = (crc & 0x00FFu);
  }
}

/**
 * @brief   Compute CRC-16 over a byte stream
 * @param   p_data  Point to first byte of the stream
 * @param   len     Length of the stream
 * @return  CRC-16 value of the stream
 */
static uint16_t phy_crc16(uint8_t *p_data, uint16_t len)
{
  uint16_t ix;
  uint32_t crc_res;

  /* calculate CRC */
  crc_res = CRC16_INIT;
  for (ix = 0; ix < len; ix++) {
    crc_res = crc_16_update(crc_res, p_data[ix]);
  }

  return crc_res;
}


/**
 * @brief   Compute CRC-32 over a byte stream
 * @param   p_data  Point to first byte of the stream
 * @param   len     Length of the stream
 * @return  CRC-32 value of the stream
 */
static uint32_t phy_crc32(uint8_t *p_data, uint16_t len)
{
  uint16_t ix;
  uint32_t crc_res;

  /* calculate CRC */
  crc_res = CRC32_INIT;
  for (ix = 0; ix < len; ix++) {
    crc_res = crc_32_update(crc_res, p_data[ix]);
  }

  /* add padding when length is less than 4 octets.
   * See IEEE-802.15.4g-2012, 5.2.1.9 */
  if (len < 4) {
    crc_res = crc_32_update(crc_res, 0x00);
  }
  crc_res ^= CRC32_INIT;

  return crc_res;
}
#endif /* #if (NETSTK_SUPPORT_SW_MAC_AUTOACK == TRUE) */


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
