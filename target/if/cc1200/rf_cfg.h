#ifndef __RF_CFG_H__
#define __RF_CFG_H__
#ifndef __DECL_RF_CFG_H__
#define __DECL_RF_CFG_H__
#endif /* #ifndef __DECL_RF_CFG_H__ */

/*============================================================================*/
/**
 * \file    rf_cfg.h
 *
 * \author  Manuel Schappacher
 *
 * \brief   Low Level RF-Chip access configuration file.
 *
 *          This module provides low level access to the RF Chip.
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/

#include <stdint.h>
#include "rf.h"
#include "hal_cc1200_io.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Maximum packet length of an RF packet */
#define RF_MAX_PKT_LEN                255

/** Fixed packet length value */
#define RF_PKT_MODE_FIXED_LEN         (0x00)
/** Variable packet length value */
#define RF_PKT_MODE_VAR_LEN           (0x20)

/** RSSI CS */
#define RF_RSSI_CS                    (0x04)
/** RSSI CS */
#define RF_RSSI_CS_VALID              (0x02)


/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/

/**
 * Register of the RF chip.
 */
typedef struct S_RF_REGISTER_T
{
  /** address of the register */
  uint16_t ui_addr;
  /** data of the register */
  uint8_t uc_data;

} s_rf_register_t;

/**
 * Channel settings.
 */
typedef struct S_RF_CH_SETTINGS_T
{
  /** register descriptions */
  const s_rf_register_t* ps_reg;
  /** number of descriptions */
  uint16_t ui_num;

} s_rf_ch_settings_t;


/*============================================================================*/
/*                                  CONSTANTS                                 */
/*============================================================================*/

/**
 * \brief   Default register values.
 */
static const s_rf_register_t gcs_rf_cfg_def[] =
{
    /* Deault IO Configuration */
    {CC1200_IOCFG3,             0xB0},  /* high impendance */
    {CC1200_IOCFG2,             0xB0},  /* high impendance */
    {CC1200_IOCFG1,             0xB0},  /* high impendance */
    {CC1200_IOCFG0,             0xB0},  /* high impendance */

    {CC1200_IQIC,               0xD8},  /* Digital Image Channel Compensation */
    {CC1200_MDMCFG1,            0x42},  /* General Modern Parameter...  */
    {CC1200_MDMCFG0,            0x05},  /* ...Viterbi enabled           */

    {CC1200_PKT_CFG2,           0x10},  /* CCA_MODE 100, RSSI below threshold AND LBT req. are met */
    {CC1200_PKT_CFG0,           0x20},  /* LENGTH_CFG: 01, variable length. packet length is 1st byte after SYNC */
    {CC1200_PKT_LEN,            0xFF},  /* Maximum packet length, 255 bytes */
    {CC1200_IF_MIX_CFG,         0x1C},  /* Intermediate freq. config. */
    {CC1200_IFAMP,              0x09},  /* Freq. Amplifier  */

    {CC1200_FS_DIG1,            0x07},  /* For test purposes only */
    {CC1200_FS_DIG0,            0xA5},  /* For test purposes only */
    {CC1200_FS_CAL1,            0x40},  /* For test purposes only */
    {CC1200_FS_CAL0,            0x0E},  /* For test purposes only */
    {CC1200_FS_DIVTWO,          0x03},  /* For test purposes only */
    {CC1200_FS_DSM0,            0x33},  /* For test purposes only */
    {CC1200_FS_DVC0,            0x17},  /* For test purposes only */
    {CC1200_FS_PFD,             0x00},  /* For test purposes only */
    {CC1200_FS_PRE,             0x6E},  /* For test purposes only */
    {CC1200_FS_REG_DIV_CML,     0x1C},  /* For test purposes only */
    {CC1200_FS_SPARE,           0xAC},  /* For test purposes only */
    {CC1200_FS_VCO0,            0xB5},  /* For test purposes only */
    {CC1200_XOSC5,              0x0E},  /* For test purposes only */
    {CC1200_XOSC1,              0x03},  /* For test purposes only */
};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_test[] =
{
    {CC1200_SYNC_CFG1,         0xC8},
    {CC1200_SYNC_CFG0,         0x23},
    {CC1200_DEVIATION_M,       0x48},
    {CC1200_MODCFG_DEV_E,      0x0B},
    {CC1200_DCFILT_CFG,        0x4B},
    {CC1200_PREAMBLE_CFG0,     0x8A},
    {CC1200_CHAN_BW,           0x0B},

    /*
     * WOR configurations
     */
    {CC1200_WOR_CFG1,           0x08},
    {CC1200_WOR_CFG0,           0x20},
    {CC1200_WOR_EVENT0_MSB,     0x00},
    {CC1200_WOR_EVENT0_MSB,     0x96},  /* 24byte long preamble, 50kbps */

    {CC1200_SYMBOL_RATE2,       0x94},  /* 50kbps...*/
    {CC1200_SYMBOL_RATE1,       0x7A},  /* ...      */
    {CC1200_SYMBOL_RATE0,       0xE1},  /* ...      */

    {CC1200_AGC_REF,            0x2A},
    {CC1200_AGC_CS_THR,         0xF7},  /* -90dBm   */
    {CC1200_AGC_CFG1,           0x00},
    {CC1200_AGC_CFG0,           0x80},

    {CC1200_PA_CFG0,            0x53},

    /*
     * Frequency configuration
     */
    {CC1200_FS_CFG,            0x14},
    {CC1200_FREQOFF_CFG,       0x20},
    {CC1200_FREQ2,             0x56},
    {CC1200_FREQ1,             0xCC},
    {CC1200_FREQ0,             0xCC},
};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_868mhz50bps[] =
{
    {CC1200_SYNC_CFG1,      0x08},  /* PQT gating enabled, sync theshold 0x08 */
    {CC1200_DEVIATION_M,    0x99},  /* Deviation = 24.963379 kHz */
    {CC1200_MODCFG_DEV_E,   0x0D},  /* Mormal modem mode, 2-GFSK, Deviation = 20.019531 kHz */
    {CC1200_DCFILT_CFG,     0x15},
    {CC1200_PREAMBLE_CFG1,  0x18},
    {CC1200_CHAN_BW,        0x02},  /* Channel filter enabled, BW = 100kHz */

    {CC1200_SYMBOL_RATE2,   0x99},  /* Symbol Rate = 50ksps */
    {CC1200_SYMBOL_RATE1,   0x99},  /* Symbol Rate = 50ksps */
    {CC1200_SYMBOL_RATE0,   0x99},  /* Symbol Rate = 50ksps */

    /* WOR configuration */
    {CC1200_WOR_CFG0,       0x24},  /* enable clock division, disable Ev2, disable RCOSC calibration, enable RCOSC */
    {CC1200_WOR_EVENT0_MSB, 0x00},  /* tEv1 = 3.76ms */
    {CC1200_WOR_EVENT0_LSB, 0x78},  /* tEv1 = 3.76ms */

    {CC1200_AGC_REF,        0x3C},
    {CC1200_AGC_CS_THR,     0x00},  /* AGC Carrier Sense Threshold -102 dBm (+102dB offset!!) */
    {CC1200_AGC_CFG1,       0xA9},
    {CC1200_AGC_CFG0,       0xC0},

    {CC1200_PA_CFG0,        0x79},

    /* Frequency configuration 868MHz */
    {CC1200_FS_CFG,         0x12},
    {CC1200_FREQOFF_CFG,    0x20},
    {CC1200_FREQ2,          0x6C},
    {CC1200_FREQ1,          0x80},
    {CC1200_FREQ0,          0x00},
};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_434mhz50bps[] =
{
    {CC1200_SYNC_CFG1,         0xC8},
    {CC1200_SYNC_CFG0,         0x23},
    {CC1200_DEVIATION_M,       0x48},
    {CC1200_MODCFG_DEV_E,      0x0B},
    {CC1200_DCFILT_CFG,        0x4B},
    {CC1200_PREAMBLE_CFG0,     0x8A},
    {CC1200_CHAN_BW,           0x0B},

    /*
     * WOR configurations
     */
    {CC1200_WOR_CFG1,           0x08},
    {CC1200_WOR_CFG0,           0x20},
    {CC1200_WOR_EVENT0_MSB,     0x00},
    {CC1200_WOR_EVENT0_MSB,     0x96},  /* 24byte long preamble, 50kbps */

    {CC1200_SYMBOL_RATE2,       0x94},  /* 50kbps...*/
    {CC1200_SYMBOL_RATE1,       0x7A},  /* ...      */
    {CC1200_SYMBOL_RATE0,       0xE1},  /* ...      */

    {CC1200_AGC_REF,            0x2A},
    {CC1200_AGC_CS_THR,         0xF7},  /* -90dBm   */
    {CC1200_AGC_CFG1,           0x00},
    {CC1200_AGC_CFG0,           0x80},

    {CC1200_PA_CFG0,            0x53},

    /*
     * Frequency configuration
     */
    {CC1200_FS_CFG,            0x14},
    {CC1200_FREQOFF_CFG,       0x20},
    {CC1200_FREQ2,             0x56},
    {CC1200_FREQ1,             0xCC},
    {CC1200_FREQ0,             0xCC},
};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_434mhz20bps[] = {
    {CC1200_SYNC_CFG1,      0x08},  /* PQT gating enabled, sync threshold 0x08 */
    {CC1200_DEVIATION_M,    0x99},  /* Deviation = 25 kHz */
    {CC1200_MODCFG_DEV_E,   0x0D},  /* Mormal modem mode, 2-GFSK, Deviation = 25 kHz */
    {CC1200_DCFILT_CFG,     0x15},
    {CC1200_PREAMBLE_CFG1,  0x18},
    {CC1200_CHAN_BW,        0x02},  /* Channel filter enabled, BW = 100kHz */

    {CC1200_SYMBOL_RATE2,   0x84},  /* Symbol Rate = 20ksps */
    {CC1200_SYMBOL_RATE1,   0x7A},  /* Symbol Rate = 20ksps */
    {CC1200_SYMBOL_RATE0,   0xE1},  /* Symbol Rate = 20ksps */

    /* WOR configuration */
    {CC1200_WOR_CFG0,       0x24},  /* enable clock division, disable Ev2, disable RCOSC calibration, enable RCOSC */
    {CC1200_WOR_EVENT0_MSB, 0x01},  /* tEv1 = 9.38ms */
    {CC1200_WOR_EVENT0_LSB, 0x2C},  /* tEv1 = 9.38ms */

    {CC1200_AGC_REF,        0x3C},
    {CC1200_AGC_CS_THR,     0x02},  /* AGC Carrier Sense Threshold -102 dBm (+102dB offset!!) */
    {CC1200_AGC_CFG1,       0xA9},
    {CC1200_AGC_CFG0,       0xC0},

    {CC1200_PA_CFG0,        0x79},

    /* Frequency configuration 434MHz */
    {CC1200_FS_CFG,         0x14},
    {CC1200_FREQOFF_CFG,    0x20},
    {CC1200_FREQ2,          0x6C},
    {CC1200_FREQ1,          0x80},
    {CC1200_FREQ0,          0x00},
};

/**
 * \brief   Register values used for Tx.
 */
static const s_rf_register_t gcs_rf_cfg_tx[] = {

    {CC1200_IOCFG3,         0x06},  /* Enable IO3 to trigger at the end of the packet (falling edge) */
    {CC1200_IOCFG2,         0xB0},  /* high impendance */
    {CC1200_IOCFG0,         0x02},  /* Enable IO to trigger on Tx-FIFO threshold limit */

    {CC1200_IOCFG1,         0xB0},  /* high impendance */
};

/**
 * \brief   Register values used for Rx.
 */
static const s_rf_register_t gcs_rf_cfg_rx[] = {

    {CC1200_IOCFG3,         0x06},                /* Enable IO3 to trigger at the end of the packet (falling edge) */
    {CC1200_IOCFG2,         0x06},                /* Enable IO2 to trigger at the reception of the sync word (rising edge) */
    {CC1200_IOCFG0,         0x00},                /* Enable IO to trigger on Rx-FIFO threshold limit */
    {CC1200_RFEND_CFG0,     0x00},                /* Disable RX termination */

    {CC1200_IOCFG1,         0xB0},                /* high impendance */
    {CC1200_PKT_CFG0,       RF_PKT_MODE_VAR_LEN}, /* Set variable length mode. */
    {CC1200_PKT_LEN,        RF_MAX_PKT_LEN},      /* Set maximum packet length */
};

/**
 * \brief   Register values used for eWOR.
 */
static const s_rf_register_t gcs_rf_cfg_ewor[] = {

    {CC1200_IOCFG3,         0x06},                /* Enable IO3 to trigger at the end of the packet (falling edge) */
    {CC1200_IOCFG2,         0x06},                /* Enable IO2 to trigger at the reception of the sync word (rising edge) */
    {CC1200_IOCFG0,         0x00},                /* Enable IO to trigger on Rx-FIFO threshold limit */
    {CC1200_RFEND_CFG0,     0x01},                /* Rx termination on CS */

    {CC1200_IOCFG1,         0xB0},                /* high impendance */
    {CC1200_PKT_CFG0,       RF_PKT_MODE_VAR_LEN}, /* Set variable length mode. */
    {CC1200_PKT_LEN,        RF_MAX_PKT_LEN},      /* Set maximum packet length */
};

static const s_rf_ch_settings_t gcs_rf_ch[E_RF_CHANNEL_MAX] = {
    { gcs_rf_cfg_ch_test, (sizeof(gcs_rf_cfg_ch_test)/sizeof(s_rf_register_t)) },     /* Test channel settings */
    { gcs_rf_cfg_ch_868mhz50bps, (sizeof(gcs_rf_cfg_ch_868mhz50bps)/sizeof(s_rf_register_t)) }, /* 868MHz@50kbps channel settings */
    { gcs_rf_cfg_ch_434mhz50bps, (sizeof(gcs_rf_cfg_ch_434mhz50bps)/sizeof(s_rf_register_t)) }, /* 434MHz@50kbps channel settings */
    { gcs_rf_cfg_ch_434mhz20bps, (sizeof(gcs_rf_cfg_ch_434mhz20bps)/sizeof(s_rf_register_t)) }, /* 434MHz@50kbps channel settings */
};


#if 1   // Used to optimize TX-RX turnaround
static const s_rf_register_t gcs_rf_cfg_ewor2[] = {

    {CC1200_IOCFG2,         0x06},                /* Enable IO2 to trigger at the reception of the sync word (rising edge) */
    {CC1200_IOCFG0,         0x00},                /* Enable IO to trigger on Rx-FIFO threshold limit */
    {CC1200_RFEND_CFG0,     0x01},                /* Rx termination on CS */
    {CC1200_PKT_CFG0,       RF_PKT_MODE_VAR_LEN}, /* Set variable length mode. */
    {CC1200_PKT_LEN,        RF_MAX_PKT_LEN},      /* Set maximum packet length */
};
#endif


#endif /* #ifndef __RF_CFG_H__ */


