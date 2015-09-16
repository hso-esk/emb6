/*
 * cc1120.c
 *
 *  Created on: Aug 18, 2015
 *      Author: phuongnguyen
 */

#include <stdint.h>
#include "lib_tmr.h"
#include "include.h"
#include "hal_cc1120_io.h"
#include "rf.h"
#include "phy.h"
#include "cc112x.h"


/** RSSI valid bit */
#define RF_RSSI_VALID_MSK             (0x01)
/** Offset of the lower RSSI bits */
#define RF_RSSI_LOW_OFST              (3)
/** Length of the lower RSSI bits */
#define RF_RSSI_LOW_LEN               (4)
/** Mask of the lower RSSI bits */
#define RF_RSSI_LOW_MSK               (0x78)
/** RSSI resolution */
#define RF_RSSI_RES                   (0.0625)
/** RSSI offset */
#define RF_RSSI_OFFSET                (102)

#define RF_SYNC_MODE_MSK                (uint8_t) ( 0x1C )

#define RF_REGWR( addr, data )    {uint8_t wr = (data); cc1120_spiWriteReg( (addr), &wr, 1);}
#define RF_REGRD( addr, data )    cc1120_spiReadReg( (addr), &(data), 1)

static void cc112x_Off (RADIO_ERR *p_err);


int8_t PHY_PdDataInd( uint8_t* puc_data, uint16_t ui_len )
{
    STK_ERR err;

    LowMacDrv->IsrRx(puc_data, ui_len, &err);
    return PHY_SUCCESS;
}

static void CC112x_RssiGet (RADIO_IOC_VAL *p_val)
{
    uint8_t rssi2complMSB;
    uint8_t rssi2complLSB;


#if STKCFG_ARG_CHK_EN
    if (p_val == (RADIO_IOC_VAL *)0) {
        return;
    }
#endif


    /* read the RSSI register and wait for a valid value */
    do
    {
      RF_REGRD( CC1120_RSSI0, rssi2complLSB );
    }while( (rssi2complLSB & RF_RSSI_VALID_MSK) == 0 );

    /* read 2nd RSSI register and add to value */
    RF_REGRD( CC1120_RSSI1, rssi2complMSB );
    *p_val = ((RADIO_IOC_VAL)(rssi2complMSB) << RF_RSSI_LOW_LEN) |
             ((RADIO_IOC_VAL)(rssi2complLSB) >> RF_RSSI_LOW_OFST);

    /* get real RSSI from offset and resolution */
    *p_val = (RADIO_IOC_VAL)((*p_val * RF_RSSI_RES) - RF_RSSI_OFFSET);
}


static void CC112x_SyncSet (RADIO_IOC_VAL *p_val)
{
    uint8_t sync = 0;


#if STKCFG_ARG_CHK_EN
    if (p_val == (RADIO_IOC_VAL *)0) {
        return;
    }
#endif


    RF_REGWR( CC1120_SYNC_CFG0, sync );
    *p_val = (RADIO_IOC_VAL) (sync & RF_SYNC_MODE_MSK);
}


static void CC112x_SyncGet (RADIO_IOC_VAL *p_val)
{
    uint8_t sync;


#if STKCFG_ARG_CHK_EN
    if (p_val == (RADIO_IOC_VAL *)0) {
        return;
    }
#endif


    RF_REGRD( CC1120_SYNC_CFG0, sync );
    *p_val = (RADIO_IOC_VAL) (sync & RF_SYNC_MODE_MSK);
}

static void cc112x_Init (RADIO_ERR *p_err)
{
    int8_t                    c_txpower = 0;
    e_phy_chpage_t            e_page;
    e_phy_cca_mode_t          e_cca_mode;
    e_phy_sniffmode_t         e_mode;
    e_phy_preamble_len_t      e_preamble;
    e_phy_txpow_tolerance_t   e_txpower_tole;

    
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

    /* use existing corresponding code segments */
}


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
            *p_val = (RADIO_IOC_VAL) PHY_PlmeCcaReq();
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
            if (p_err) {
                *p_err = RADIO_ERR_CMD_UNSUPPORTED;
            }
            break;
    }
}


static void cc112x_Task (void *p_arg)
{
    phy_entry();
}


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
