/*
 * cc1120.c
 *
 *  Created on: Aug 18, 2015
 *      Author: phuongnguyen
 */


#include "emb6_conf.h"
#include "packetbuf.h"

#include "lib_tmr.h"
#include "rf.h"
#include "phy.h"
#include "cc112x.h"

/*
********************************************************************************
*                           LOCAL VARIABLES
********************************************************************************
*/
static s_ns_t *RF_Netstack;


/*
********************************************************************************
*                       LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static void cc112x_Init (s_ns_t *p_netstack, RADIO_ERR *p_err);
static void cc112x_On (RADIO_ERR *p_err);
static void cc112x_Off (RADIO_ERR *p_err);
static void cc112x_Send(const void *p_payload, uint8_t len, RADIO_ERR *p_err);
static void cc112x_Recv(void *p_buf, uint8_t len, RADIO_ERR *p_err);
static void cc112x_Ioctl(RADIO_IOC_CMD cmd, RADIO_IOC_VAL *p_val, RADIO_ERR *p_err);
static void cc112x_Task (void *p_arg);



/*
********************************************************************************
*                           API FUNCTION DEFINITIONS
********************************************************************************
*/

/**
 * @brief   Initialize radio transceiver driver
 *
 * @param   p_err   Point to returned error
 */
static void cc112x_Init (s_ns_t *p_netstack, RADIO_ERR *p_err)
{
    int8_t                    c_txpower = 0;
    e_phy_chpage_t            e_page;
    e_phy_cca_mode_t          e_cca_mode;
    e_phy_sniffmode_t         e_mode;
    e_phy_preamble_len_t      e_preamble;
    e_phy_txpow_tolerance_t   e_txpower_tole;

    if (p_netstack == (s_ns_t *)0) {
        *p_err = RADIO_ERR_INIT;
    } else {
        RF_Netstack = (s_ns_t *)p_netstack;

        while (phy_init() == PHY_ERROR) {
        };

        c_txpower      = 15;
        e_mode         = E_PHY_SNIFFMODE_CS;
        e_page         = E_PHY_CHPAGE_434MHZ_50KBPS;
        e_cca_mode     = E_PHY_CCA_MODE_CARRIER_OR_ED;
        e_preamble     = E_PHY_PREAMBLE_LEN_191;
        e_txpower_tole = E_PHY_TXPOW_TOLERANCE_6DB;

        PHY_PlmeSetReq (E_PHY_PIB_phyTXPower,              &c_txpower);
        PHY_PlmeSetReq (E_PHY_PIB_phySniffMode,            &e_mode);
        PHY_PlmeSetReq (E_PHY_PIB_phyCurrentPage,          &e_page);
        PHY_PlmeSetReq (E_PHY_PIB_phyCCAMode,              &e_cca_mode);
        PHY_PlmeSetReq (E_PHY_PIB_phyPreambleSymbolLength, &e_preamble);
        PHY_PlmeSetReq (E_PHY_PIB_phyTXPowerTolerance,     &e_txpower_tole);

        cc112x_Off(NULL);
    }

#if 0
    /*
     * Set MAC address
     */
    if (mac_phy_config.mac_address == NULL) {
    }
    else {
        memcpy((void *)&un_addr.u8,  &mac_phy_config.mac_address, 8);
        memcpy(&uip_lladdr.addr, &un_addr.u8, 8);
        _rf212_setPanAddr(mac_phy_config.pan_id, 0, (uint8_t *)&un_addr.u8);
        rimeaddr_emb6_set_node_addr(&un_addr);
        _rf212_setChannel(CHANNEL_802_15_4);
    }
#endif
}


/**
 * @brief   Turn radio transceiver on
 *
 * @param   p_err   Point to returned error
 */
static void cc112x_On (RADIO_ERR *p_err)
{
    int8_t err;


#if STKCFG_ARG_CHK_EN
    if (p_err == (RADIO_ERR *)0) {
        return;
    }
#endif

    err = PHY_PlmeRfOn(true);
    if (err != PHY_SUCCESS) {
        *p_err = RADIO_ERR_ONOFF;
    } else {
        *p_err = RADIO_ERR_NONE;
    }
}


/**
 * @brief   Turn radio transceiver off
 *
 * @param   p_err   Point to returned error
 */
static void cc112x_Off (RADIO_ERR *p_err)
{
    int8_t err;


#if STKCFG_ARG_CHK_EN
    if (p_err == (RADIO_ERR *)0) {
        return;
    }
#endif

    err = PHY_PlmeRfOn(false);
    if (err != PHY_SUCCESS) {
        *p_err = RADIO_ERR_ONOFF;
    } else {
        *p_err = RADIO_ERR_NONE;
    }
}


/**
 * @brief   Issue data transmission
 *
 * @param   p_payload   Point to buffer storing data to send
 * @param   len         Length of data to send
 * @param   p_err       Point to returned error
 */
static void cc112x_Send (const void         *p_payload,
                               uint8_t       len,
                               RADIO_ERR    *p_err)
{
    int8_t phy_ret;

    LED_TX_ON();
    phy_ret = PHY_PdDataReq((uint8_t *) p_payload, (uint16_t) len);
    LED_TX_OFF();
    if (phy_ret != PHY_SUCCESS) {
        if (p_err) {
            *p_err = RADIO_ERR_TX;
        }
    } else {
        *p_err = RADIO_ERR_NONE;
    }
}


static void cc112x_Recv (void       *p_buf,
                         uint8_t     len,
                         RADIO_ERR  *p_err)
{
    (void)&p_buf;
    (void)&len;
    (void)&p_err;

    /* use existing corresponding code segments */
}


/**
 * @brief   Input/Output control
 *
 * @param   cmd     IOCTL Command
 * @param   p_val   If wanted to assign value to a information base attribute,
 *                  p_val points to value to set.
 *                  If wanted to read value of a information base attribute,
 *                  p_val points to storing buffer.
 * @param   p_err   Point to returned error
 */
static void cc112x_Ioctl (RADIO_IOC_CMD  cmd,
                          RADIO_IOC_VAL *p_val,
                          RADIO_ERR     *p_err)
{
#if STKCFG_ARG_CHK_EN
    if (p_err == (RADIO_ERR *)0) {
        return;
    }

    if (p_val == (RADIO_IOC_VAL *)0) {
        return;
    }
#endif


    *p_err = RADIO_ERR_NONE;
    switch (cmd) {
        case RADIO_IOC_CMD_TXPOWER_SET :
            PHY_PlmeSetReq(E_PHY_PIB_phyTXPower, p_val);
            break;

        case RADIO_IOC_CMD_TXPOWER_GET :
            PHY_PlmeGetReq(E_PHY_PIB_phyTXPower, p_val);
            break;

        case RADIO_IOC_CMD_CCA_GET :
            *p_val = (RADIO_IOC_VAL)PHY_PlmeCcaReq();
            break;

        case RADIO_IOC_CMD_SYNC_SET :
            //CC112x_SyncSet(p_val);
            break;

        case RADIO_IOC_CMD_SYNC_GET :
            //CC112x_SyncGet(p_val);
            break;

        case RADIO_IOC_CMD_RSSI_GET :
            break;

        case RADIO_IOC_CMD_STATE_GET:
            if (rf_is_rx()) {
                *p_val = RADIO_IOC_VAL_STATE_RX;
                return;
            }

            if (rf_is_tx()) {
                *p_val = RADIO_IOC_VAL_STATE_TX;
                return;
            }

            *p_val = RADIO_IOC_VAL_STATE_IDLE;

            break;

        case RADIO_IOC_CMD_RF_SWITCH :
        case RADIO_IOC_CMD_ANT_DIV_SET :
        case RADIO_IOC_CMD_SENS_SET :
        case RADIO_IOC_CMD_SENS_GET :
        default:
            /* unsupported commands are treated in same way */
            *p_err = RADIO_ERR_CMD_UNSUPPORTED;
            break;
    }
}


/**
 * @brief   Radio transceiver state machine handler
 * @param   p_arg
 */
static void cc112x_Task (void *p_arg)
{
    (void)&p_arg;
    phy_entry();
}


/**
 * @brief   Data indication handler
 *
 * @param   puc_data    Point to buffer storing received data
 * @param   ui_len      Length of the received data
 * @return
 */
int8_t PHY_PdDataInd( uint8_t* puc_data, uint16_t ui_len )
{
    packetbuf_clear();
    packetbuf_set_datalen(ui_len);
    memcpy(packetbuf_dataptr(), puc_data, ui_len);
    RF_Netstack->lmac->input();
    return PHY_SUCCESS;
}


/*
********************************************************************************
*                               DRIVER DEFINITION
********************************************************************************
*/
RADIO_DRV_API CC112xDrv = {
        "CC112x",
        cc112x_Init,
        cc112x_On,
        cc112x_Off,
        cc112x_Send,
        cc112x_Recv,
        cc112x_Ioctl,
        cc112x_Task
};
