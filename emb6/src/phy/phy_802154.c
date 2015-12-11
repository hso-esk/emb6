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
#include "phy_framer_802154.h"

#include "lib_tmr.h"
#include "lib_crc.h"
#include "packetbuf.h"

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
static void PHY_Init(void *p_netstk, e_nsErr_t *p_err);
static void PHY_On(e_nsErr_t *p_err);
static void PHY_Off(e_nsErr_t *p_err);
static void PHY_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void PHY_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void PHY_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);


/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t   *PHY_Netstk;
static uint8_t   PHY_FcsLen;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsPHY_t PHYDrv802154 =
{
   "PHY 802154",
    PHY_Init,
    PHY_On,
    PHY_Off,
    PHY_Send,
    PHY_Recv,
    PHY_IOCtrl
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
static void PHY_Init(void *p_netstk, e_nsErr_t *p_err)
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
    PHY_Netstk = (s_ns_t *)p_netstk;

    /* set CRC size to default */
#if NETSTK_CFG_IEEE_802154G_EN
    PHY_FcsLen = 2; /* 32-bit CRC is not yet supported */
#else
    PHY_FcsLen = 2; /* 16-bit CRC */
#endif

    /* set returned error */
    *p_err = NETSTK_ERR_NONE;
}


/**
 * @brief   Turn driver on
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_On(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    PHY_Netstk->rf->on(p_err);
}


/**
 * @brief   Turn driver off
 *
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_Off(e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }
#endif

    PHY_Netstk->rf->off(p_err);
}


/**
 * @brief   Frame transmission handler
 *
 * @param   p_data      Pointer to buffer holding frame to send
 * @param   len         Length of frame to send
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
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

#if LOGGER_ENABLE
    /*
     * Logging
     */
    uint16_t data_len = len;
    uint8_t *p_dataptr = p_data;
    LOG_RAW("PHY_TX: ");
    while (data_len--) {
        LOG_RAW("%02x", *p_dataptr++);
    }
    LOG_RAW("\r\n====================\r\n");
#endif

    int alloc_ok;
    uint32_t fcs;
    uint16_t phr, psdu_len;

    /*
     * insert Frame Checksum (FCS)
     */
    alloc_ok = packetbuf_ftralloc(PHY_FcsLen);
    if (alloc_ok == 0) {
        *p_err = NETSTK_ERR_BUF_OVERFLOW;
        return;
    }

    if (PHY_FcsLen == 4) {
        fcs = crc_32(p_data, len);
    } else {
        fcs = crc_16(p_data, len);
        fcs = ((fcs & 0x00FF) << 8) |
              ((fcs & 0xFF00) >> 8);
    }
    memcpy(packetbuf_ftrptr(), &fcs, PHY_FcsLen);

    /*
     * insert PHR
     */
    alloc_ok = packetbuf_hdralloc(PHY_HEADER_LEN);
    if (alloc_ok == 0) {
        *p_err = NETSTK_ERR_BUF_OVERFLOW;
        return;
    }

    /* compute header fields */
    psdu_len = len + PHY_FcsLen;
    phr = psdu_len;

#if NETSTK_CFG_IEEE_802154G_EN
    if (PHY_FcsLen == 2) {
        phr |= 0x1000;
    }

    /* swap MSB and LSB of the header */
    phr = ((phr & 0xFF00) >> 8) |
          ((phr & 0x00FF) << 8);
#else
    phr = psdu_len & 0x7F;
#endif
    /* write the header */
    memcpy(packetbuf_hdrptr(), &phr, PHY_HEADER_LEN);

    /* Issue next lower layer to transmit the prepared frame */
    PHY_Netstk->rf->send(packetbuf_hdrptr(), packetbuf_totlen(), p_err);
}


/**
 * @brief   Frame reception handler
 *
 * @param   p_data      Pointer to buffer holding frame to receive
 * @param   len         Length of frame to receive
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_Recv(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err)
{
#if NETSTK_CFG_ARG_CHK_EN
    if (p_err == NULL) {
        return;
    }

    if ((len < PHY_HEADER_LEN) ||
        (p_data == NULL)) {
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
#if NETSTK_CFG_IEEE_802154G_EN
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
    p_data += PHY_HEADER_LEN;

    /* verify CRC */
    crc_exp = 0;
    crc_act = 0;

    if (phr & 0x1000) {
        /* 16-bit CRC was used in the received frame */
        crc_size = 2;
        psdu_len -= crc_size;

        /* obtain CRC of the received frame */
        memcpy(&crc_exp, &p_data[psdu_len], crc_size);
        crc_exp = ((crc_exp & 0x00FF) << 8) |
                  ((crc_exp & 0xFF00) >> 8);

        /* calculated actual CRC */
        crc_act = crc_16(p_data, psdu_len);
    } else {
        /* 32-bit CRC was used in the received frame */
        crc_size = 4;
        psdu_len -= crc_size;

        /* obtain CRC of the received frame */
        memcpy(&crc_exp, &p_data[psdu_len], crc_size);
        crc_exp = ((uint32_t)(p_data[psdu_len + 0])) |
                  ((uint32_t)(p_data[psdu_len + 1]) << 8)  |
                  ((uint32_t)(p_data[psdu_len + 2]) << 16) |
                  ((uint32_t)(p_data[psdu_len + 3]) << 24);

        /* calculated actual CRC */
        crc_act = crc_32(p_data, psdu_len);
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
    p_data += PHY_HEADER_LEN;

    /* verify CRC */
    psdu_len -= PHY_FcsLen;
    crc_act = crc_16(p_data, psdu_len);
    memcpy(&crc_exp, &p_data[psdu_len], PHY_FcsLen);
    if (crc_act != crc_exp) {
        *p_err = NETSTK_ERR_CRC;
        return;
    }
#endif

    /* Inform the next higher layer */
    PHY_Netstk->mac->recv(p_data, psdu_len, p_err);
}


/**
 * @brief    Miscellaneous commands handler
 *
 * @param   cmd         Command to be issued
 * @param   p_val       Pointer to a variable related to the command
 * @param   p_err       Pointer to a variable storing returned error code
 */
static void PHY_IOCtrl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err)
{
    uint8_t crc_size;

    /* Set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    switch (cmd) {
        case NETSTK_CMD_PHY_CRC_LEN_SET:
#if NETSTK_CFG_IEEE_802154G_EN
            crc_size = *((uint8_t *)p_val);
            if ((crc_size != 4) && (crc_size != 2)) {
                *p_err = NETSTK_ERR_INVALID_ARGUMENT;
            } else {
                PHY_FcsLen = crc_size;
            }
#else
            (void )&crc_size;
            *p_err = NETSTK_ERR_INVALID_ARGUMENT;
#endif
            break;

        case NETSTK_CMD_PHY_LAST_PKT_TX:
            /* Issue next lower layer to transmit the prepared frame */
            PHY_Netstk->rf->send(packetbuf_hdrptr(), packetbuf_totlen(), p_err);
            break;

        default:
            PHY_Netstk->rf->ioctrl(cmd, p_val, p_err);
            break;
    }
}


/*
********************************************************************************
*                               END OF FILE
********************************************************************************
*/
