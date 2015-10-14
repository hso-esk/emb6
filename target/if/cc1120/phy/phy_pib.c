/*============================================================================*/
/**
 * \file    phy_pib.c
 *
 * \author  Manuel Schappacher
 *
 * \brief   Physical layer PIB implementation.
 *
 */
/*============================================================================*/

#ifdef __cplusplus
extern "C"
{
#endif

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#define __DECL_PHY_PIB_H__
#include "phy_pib.h"

#if !defined(MAC_TDD_ENABLED)
/*============================================================================*/
/*                                CONSTANTS                                   */
/*============================================================================*/

/**
 * The PHY constants
 */
__DECL_PHY_PIB_H__ const s_phy_consts_t gcs_phy_consts = {
   .aMaxPHYPacketSize = PHY_MAX_PACKET_SIZE,
};

/**
 * PIB default values.
 */
__DECL_PHY_PIB_H__ const s_phy_pib_t gcs_phy_pibDefaults =
{
    0,                          /* phyCurrentChannel */
    E_PHY_TXPOW_TOLERANCE_1DB,  /* phyTXPowerTolerance */
    PHY_PIB_TXPOWER_DEFAULT,    /* phyTXPower */
    E_PHY_CCA_MODE_ED_THRESHOLD,/* phyCCAMode */
    E_PHY_CHPAGE_TEST,          /* phyCurrentPage */
    E_PHY_PREAMBLE_LEN_31,      /* phyPreambleSymbolLength */
    E_PHY_SNIFFMODE_OFF         /* phySniffMode */
};

/**
 * The PHY channel pages including channels.
 */
__DECL_PHY_PIB_H__ const e_rf_channel_t gcpe_phy_channels [E_PHY_CHPAGE_MAX][PHY_MAX_CHANNELS_PER_PAGE] = {
    { E_RF_CHANNEL_TEST },
    { E_RF_CHANNEL_868MHZ_50KBPS },
    { E_RF_CHANNEL_434MHZ_50KBPS },
    { E_RF_CHANNEL_434MHZ_20KBPS }
};

/**
 * Tx power tolerance map
 */
__DECL_PHY_PIB_H__ const uint8_t gcpuc_phy_txtolerance_table[E_PHY_TXPOW_TOLERANCE_MAX] = {
    1,  /* E_PHY_TXPOW_TOLERANCE_1DB */
    3,  /* E_PHY_TXPOW_TOLERANCE_3DB */
    6   /* E_PHY_TXPOW_TOLERANCE_6DB */
};

/**
 * CCA map
 */
__DECL_PHY_PIB_H__ const e_rf_cca_mode_t gcpe_phy_cca_table[E_PHY_CCA_MODE_MAX] = {
    E_RF_CCA_MODE_ED_THRESHOLD,     /* E_PHY_CCA_MODE_ED_THRESHOLD */
    E_RF_CCA_MODE_CARRIER_SENSE,    /* E_PHY_CCA_MODE_CARRIER_SENSE */
    E_RF_CCA_MODE_CARRIER_OR_ED,    /* E_PHY_CCA_MODE_CARRIER_OR_ED */
    E_RF_CCA_MODE_ALOHA,            /* E_PHY_CCA_MODE_ALOHA */
    E_RF_CCA_MODE_CARRIER_LBT       /* E_PHY_CCA_MODE_CARRIER_LBT */
};

/**
 * Preamble map
 */
__DECL_PHY_PIB_H__ const e_rf_preamble_t gcpe_phy_preamble_table[E_PHY_PREAMBLE_LEN_MAX] = {
    E_RF_PREAMBLE_SHORT,    /* E_PHY_PREAMBLE_LEN_31 */
    E_RF_PREAMBLE_MID,      /* E_PHY_PREAMBLE_LEN_127 */
    E_RF_PREAMBLE_LONG      /* E_PHY_PREAMBLE_LEN_192 */
};


/*============================================================================*/
/*                            VARIABLES                                       */
/*============================================================================*/

/**
 * The PHY information base table.
 */
__DECL_PHY_PIB_H__ s_phy_pib_table_t gps_phy_pibTable[] = {

    {offsetof(s_phy_pib_t, phyCurrentChannel), sizeof(uint8_t), 0, 1, false},                         /* E_PHY_PIB_phyCurrentChannel */
    {offsetof(s_phy_pib_t, phyTXPowerTolerance), sizeof(e_phy_txpow_tolerance_t), 0, 0, false},       /* E_PHY_PIB_phyTXPowerTolerance */
    {offsetof(s_phy_pib_t, phyTXPower), sizeof(int8_t), 0, 0, false},                                 /* E_PHY_PIB_phyTXPower */
    {offsetof(s_phy_pib_t, phyCCAMode), sizeof(e_phy_cca_mode_t), 0, 0, false},                       /* E_PHY_PIB_phyCCAMode */
    {offsetof(s_phy_pib_t, phyCurrentPage), sizeof(e_phy_chpage_t), 0, 0, false},                     /* E_PHY_PIB_phyCurrentPage */
    {offsetof(s_phy_pib_t, phyPreambleSymbolLength), sizeof(e_phy_preamble_len_t), 0, 0, false},      /* E_PHY_PIB_phyPreambleSymbolLength */
    {offsetof(s_phy_pib_t, phySniffMode), sizeof(e_phy_sniffmode_t), 0, 0, false},                    /* E_PHY_PIB_phySniffMode */
};

/**
 * The PHY Information base.
 */
__DECL_PHY_PIB_H__ s_phy_pib_t gs_phy_pib;

#endif /* MAC_TDD_ENABLED */

#ifdef __cplusplus
}
#endif
