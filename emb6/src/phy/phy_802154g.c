/**
 * @file    phy_802154g.c
 * @date    16.11.2015
 * @author  PN
 * @brief   Implementation of IEEE802.15.4g PHY
 */




/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"


#if NETSTK_CFG_PHY_802154G_EN
#include "lib_tmr.h"

#define     LOGGER_ENABLE        LOGGER_PHY
#include    "logger.h"

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
static s_ns_t *PHY_Netstk;


/*
********************************************************************************
*                               GLOBAL VARIABLES
********************************************************************************
*/
const s_nsPHY_t PHYDrv802154g =
{
   "IEEE802.15.4G PHY",
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

    uint8_t channel;

    /* initialize local variables */
    PHY_Netstk = (s_ns_t *)p_netstk;

    /* set returned error code to default */
    *p_err = NETSTK_ERR_NONE;

    /* select IEEE802.15.4G channel 0 in EU */
    channel = 0;
    PHY_Netstk->rf->ioctrl(NETSTK_CMD_RF_802154G_EU_CHAN, &channel, p_err);

    /*
     * Note(s) Preamble, see also IEEE802.15.4g, section 18.1.1.1
     *
     * (1)  The preamble field shall contain phyFSKPreambleLength multiples of
     *      the 8-bit sequence "0101 0101b" or 0x55 for filtered 2-FSK
     *
     *      The preamble field shall contain phyFSKPreambleLength multiples of
     *      the 16-bit sequence "0111 0111 0111 0111b" or 0x7777 for filtered
     *      4-FSK
     */


    /*
     * Note(s)  SFD, see also IEEE802.15.4g, section 18.1.1.2
     *
     * (1)  SFD for filtered 2-FSK shall be a 2-octet sequence
     *      SFD for filtered 4-FSK shall be a 4-octet sequence
     *
     * (2)  Devices that do not support FEC shall support the SFD associated
     *      with  uncoded (PHR + PSDU) and a value of Zero for the PIB attribute
     *      phyMRFSKSFD (compulsory). SFD15_0 = 0111.0010.0000.1001b = 0x7209
     *
     *      These devices may also support the SFD associated with uncoded
     *      (PHR + PSDU) and a value of One for the PIB attribute phyMRFSKSFD
     *      (optional). SFD15_0 = 0111.0000.0101.1110b = 0x705E
     */

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

    /*
     * Issue next lower layer to transmit the prepared frame
     */
    PHY_Netstk->rf->send(p_data, len, p_err);
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
    LOG_RAW("\r\n====================\r\n");
    LOG_RAW("PHY_RX: ");
    while (data_len--) {
        LOG_RAW("%02x", *p_dataptr++);
    }
    LOG_RAW("\n\r");
#endif


    /*
     * Inform the next higher layer
     */
    PHY_Netstk->mac->recv(p_data, len, p_err);
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
    /* Set returned error code to default */
    *p_err = NETSTK_ERR_NONE;
    switch (cmd) {
        case NETSTK_CMD_PHY_RSVD:
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
#endif /* #if NETSTK_CFG_PHY_802154G_EN */
