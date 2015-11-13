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



#if NETSTK_CFG_RF_CC1120_EN
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
static const s_rf_register_t gcs_rf_cfg_def[] = {

    /* Deault IO Configuration */
    {CC1120_IOCFG3,         0xB0},  /* high impendance */
    {CC1120_IOCFG2,         0xB0},  /* high impendance */
    {CC1120_IOCFG1,         0xB0},  /* high impendance */
    {CC1120_IOCFG0,         0xB0},  /* high impendance */

    {CC1120_IQIC,           0x00},
    {CC1120_MDMCFG0,        0x05},  /* Modem enabled, Data filter disabled, Vitrebi enabled */
    {CC1120_SETTLING_CFG,   0x03},  /* Lock time = 150/60 us, regulator setting time = 60 us */

    {CC1120_PKT_CFG0,       0x20},  /* variable packet length */
    {CC1120_PKT_CFG2,       0x10},  /* CCA -> LBT */
    {CC1120_PKT_LEN,        0xFF},  /* maximum allowed length of a packet */
    {CC1120_IF_MIX_CFG,     0x00},  /* TEST PURPOSE ONLY */

    {CC1120_FS_DIG1,        0x00},  /* TEST PURPOSE ONLY */
    {CC1120_FS_DIG0,        0x5F},  /* highest loop BW for RX and TX */
    {CC1120_FS_CAL1,        0x40},  /* TEST PURPOSE ONLY */
    {CC1120_FS_CAL0,        0x0E},  /* infinite average */
    {CC1120_FS_DIVTWO,      0x03},  /* TEST PURPOSE ONLY */
    {CC1120_FS_DSM0,        0x33},  /* TEST PURPOSE ONLY */
    {CC1120_FS_DVC0,        0x17},  /* TEST PURPOSE ONLY */
    {CC1120_FS_PFD,         0x50},  /* TEST PURPOSE ONLY */
    {CC1120_FS_PRE,         0x6E},  /* TEST PURPOSE ONLY */
    {CC1120_FS_REG_DIV_CML, 0x14},  /* TEST PURPOSE ONLY */
    {CC1120_FS_SPARE,       0xAC},  /* TEST PURPOSE ONLY */
    {CC1120_FS_VCO0,        0xB4},  /* TEST PURPOSE ONLY */
    {CC1120_XOSC5,          0x0E},  /* TEST PURPOSE ONLY */
    {CC1120_XOSC1,          0x03},  /* TEST PURPOSE ONLY, Low phase noise */
};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_test[] = {
    {CC1120_SYNC_CFG1,      0x0B},  /* PQT gating enabled, sync theshold 0x0B */
    {CC1120_DEVIATION_M,    0x48},  /* Deviation = 20.019531 kHz */
    {CC1120_MODCFG_DEV_E,   0x05},  /* Mormal modem mode, 2-FSK, Deviation = 20.019531 kHz */
    {CC1120_DCFILT_CFG,     0x1C},
    {CC1120_PREAMBLE_CFG1,  0x14},
    {CC1120_FREQ_IF_CFG,    0x40},
    {CC1120_CHAN_BW,        0x04},  /* Channel filter enabled, BW = 50kHz */

    /* WOR configuration */
    {CC1120_WOR_CFG0,       0x24},  /* enable clock division, disable Ev2, disable RCOSC calibration, enable RCOSC */
    {CC1120_WOR_EVENT0_MSB, 0x02},  /* tEv1 = 16.67ms */
    {CC1120_WOR_EVENT0_LSB, 0x14},  /* tEv1 = 16.67ms */

    {CC1120_SYMBOL_RATE2,   0x43},  /* Symbol Rate = 1.2ksps */
    {CC1120_SYMBOL_RATE1,   0xA9},  /* Symbol Rate = 1.2ksps */
    {CC1120_SYMBOL_RATE0,   0x2A},  /* Symbol Rate = 1.2ksps */

    {CC1120_AGC_REF,        0x36},
    {CC1120_AGC_CS_THR,     0xFD},  /* AGC Carrier Sense Threshold -105 dBm (+102dB offset!!) */
    {CC1120_AGC_CFG1,       0xA9},
    {CC1120_AGC_CFG0,       0xCF},

    {CC1120_PA_CFG0,        0x7C},

    /* Frequency configuration 868MHz */
    {CC1120_FS_CFG,         0x12},
    {CC1120_FREQOFF_CFG,    0x22},
    {CC1120_FREQ2,          0x6C},
    {CC1120_FREQ1,          0x80},
    {CC1120_FREQ0,          0x00},

};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_868mhz50bps[] = {
    {CC1120_SYNC_CFG1,      0x08},  /* PQT gating enabled, sync theshold 0x08 */
    {CC1120_DEVIATION_M,    0x99},  /* Deviation = 24.963379 kHz */
    {CC1120_MODCFG_DEV_E,   0x0D},  /* Mormal modem mode, 2-GFSK, Deviation = 20.019531 kHz */
    {CC1120_DCFILT_CFG,     0x15},
    {CC1120_PREAMBLE_CFG1,  0x18},
    {CC1120_FREQ_IF_CFG,    0x3A},
    {CC1120_CHAN_BW,        0x02},  /* Channel filter enabled, BW = 100kHz */

    {CC1120_SYMBOL_RATE2,   0x99},  /* Symbol Rate = 50ksps */
    {CC1120_SYMBOL_RATE1,   0x99},  /* Symbol Rate = 50ksps */
    {CC1120_SYMBOL_RATE0,   0x99},  /* Symbol Rate = 50ksps */

    /* WOR configuration */
    {CC1120_WOR_CFG0,       0x24},  /* enable clock division, disable Ev2, disable RCOSC calibration, enable RCOSC */
    {CC1120_WOR_EVENT0_MSB, 0x00},  /* tEv1 = 3.76ms */
    {CC1120_WOR_EVENT0_LSB, 0x78},  /* tEv1 = 3.76ms */

    {CC1120_AGC_REF,        0x3C},
    {CC1120_AGC_CS_THR,     0x00},  /* AGC Carrier Sense Threshold -102 dBm (+102dB offset!!) */
    {CC1120_AGC_CFG1,       0xA9},
    {CC1120_AGC_CFG0,       0xC0},

    {CC1120_PA_CFG0,        0x79},

    /* Frequency configuration 868MHz */
    {CC1120_FS_CFG,         0x12},
    {CC1120_FREQOFF_CFG,    0x20},
    {CC1120_FREQ2,          0x6C},
    {CC1120_FREQ1,          0x80},
    {CC1120_FREQ0,          0x00},
};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_434mhz50bps[] =
{
    {CC1120_SYNC_CFG1,      0x08},  /* PQT gating enabled, sync threshold 0x08 */
    {CC1120_SYNC_CFG0,      0x1B},  /* 1B: 16H bit, 17: 32bit Sync */
    {CC1120_SYNC3,          0x93},
    {CC1120_SYNC2,          0x0B},
    {CC1120_SYNC1,          0x51},
    {CC1120_SYNC0,          0xDE},


    {CC1120_DEVIATION_M,    0x99},  /* Deviation = 24.963379 kHz */
    {CC1120_MODCFG_DEV_E,   0x0D},  /* Normal modem mode, 2-GFSK, Deviation = 20.019531 kHz */
    {CC1120_DCFILT_CFG,     0x15},
    {CC1120_PREAMBLE_CFG1,  0x30},  /* 24byte long preamble */
    {CC1120_FREQ_IF_CFG,    0x3A},
    {CC1120_CHAN_BW,        0x02},  /* Channel filter enabled, BW = 100kHz */

    {CC1120_SYMBOL_RATE2,   0x99},  /* Symbol Rate = 50ksps */
    {CC1120_SYMBOL_RATE1,   0x99},  /* Symbol Rate = 50ksps */
    {CC1120_SYMBOL_RATE0,   0x99},  /* Symbol Rate = 50ksps */

    /* WOR configuration */
    {CC1120_WOR_CFG0,       0x24},  /* enable clock division, disable Ev2, disable RCOSC calibration, enable RCOSC */
    {CC1120_WOR_EVENT0_MSB, 0x00},  /* tEv1 = 3.76ms */
    {CC1120_WOR_EVENT0_LSB, 0x78},  /* tEv1 = 3.76ms */

    {CC1120_AGC_REF,        0x3C},
    {CC1120_AGC_CS_THR,     0x00},  /* AGC Carrier Sense Threshold -102 dBm (+102dB offset!!) */
    {CC1120_AGC_CFG1,       0xA9},
    {CC1120_AGC_CFG0,       0xC0},

    {CC1120_PA_CFG0,        0x79},

    /* Frequency configuration 434MHz */
    {CC1120_FS_CFG,         0x14},
    {CC1120_FREQOFF_CFG,    0x20},
    {CC1120_FREQ2,          0x6C},
    {CC1120_FREQ1,          0x80},
    {CC1120_FREQ0,          0x00},
};

/**
 * \brief   Channel related register values.
 */
static const s_rf_register_t gcs_rf_cfg_ch_434mhz20bps[] = {
    {CC1120_SYNC_CFG1,      0x08},  /* PQT gating enabled, sync threshold 0x08 */
    {CC1120_DEVIATION_M,    0x99},  /* Deviation = 25 kHz */
    {CC1120_MODCFG_DEV_E,   0x0D},  /* Mormal modem mode, 2-GFSK, Deviation = 25 kHz */
    {CC1120_DCFILT_CFG,     0x15},
    {CC1120_PREAMBLE_CFG1,  0x18},
    {CC1120_FREQ_IF_CFG,    0x3A},
    {CC1120_CHAN_BW,        0x02},  /* Channel filter enabled, BW = 100kHz */

    {CC1120_SYMBOL_RATE2,   0x84},  /* Symbol Rate = 20ksps */
    {CC1120_SYMBOL_RATE1,   0x7A},  /* Symbol Rate = 20ksps */
    {CC1120_SYMBOL_RATE0,   0xE1},  /* Symbol Rate = 20ksps */

    /* WOR configuration */
    {CC1120_WOR_CFG0,       0x24},  /* enable clock division, disable Ev2, disable RCOSC calibration, enable RCOSC */
    {CC1120_WOR_EVENT0_MSB, 0x01},  /* tEv1 = 9.38ms */
    {CC1120_WOR_EVENT0_LSB, 0x2C},  /* tEv1 = 9.38ms */

    {CC1120_AGC_REF,        0x3C},
    {CC1120_AGC_CS_THR,     0x02},  /* AGC Carrier Sense Threshold -102 dBm (+102dB offset!!) */
    {CC1120_AGC_CFG1,       0xA9},
    {CC1120_AGC_CFG0,       0xC0},

    {CC1120_PA_CFG0,        0x79},

    /* Frequency configuration 434MHz */
    {CC1120_FS_CFG,         0x14},
    {CC1120_FREQOFF_CFG,    0x20},
    {CC1120_FREQ2,          0x6C},
    {CC1120_FREQ1,          0x80},
    {CC1120_FREQ0,          0x00},
};

/**
 * \brief   Register values used for Tx.
 */
static const s_rf_register_t gcs_rf_cfg_tx[] = {

    {CC1120_IOCFG3,         0x06},  /* Enable IO3 to trigger at the end of the packet (falling edge) */
    {CC1120_IOCFG2,         0xB0},  /* high impendance */
    {CC1120_IOCFG0,         0x02},  /* Enable IO to trigger on Tx-FIFO threshold limit */

    {CC1120_IOCFG1,         0xB0},  /* high impendance */
};

/**
 * \brief   Register values used for Rx.
 */
static const s_rf_register_t gcs_rf_cfg_rx[] = {

    {CC1120_IOCFG3,         0x06},                /* Enable IO3 to trigger at the end of the packet (falling edge) */
    {CC1120_IOCFG2,         0x06},                /* Enable IO2 to trigger at the reception of the sync word (rising edge) */
    {CC1120_IOCFG0,         0x00},                /* Enable IO to trigger on Rx-FIFO threshold limit */
    {CC1120_RFEND_CFG0,     0x00},                /* Disable RX termination */

    {CC1120_IOCFG1,         0xB0},                /* high impendance */
    {CC1120_PKT_CFG0,       RF_PKT_MODE_VAR_LEN}, /* Set variable length mode. */
    {CC1120_PKT_LEN,        RF_MAX_PKT_LEN},      /* Set maximum packet length */
};

/**
 * \brief   Register values used for eWOR.
 */
static const s_rf_register_t gcs_rf_cfg_ewor[] = {

    {CC1120_IOCFG3,         0x06},                /* Enable IO3 to trigger at the end of the packet (falling edge) */
    {CC1120_IOCFG2,         0x06},                /* Enable IO2 to trigger at the reception of the sync word (rising edge) */
    {CC1120_IOCFG0,         0x00},                /* Enable IO to trigger on Rx-FIFO threshold limit */
    {CC1120_RFEND_CFG0,     0x01},                /* Rx termination on CS */

    {CC1120_IOCFG1,         0xB0},                /* high impendance */
    {CC1120_PKT_CFG0,       RF_PKT_MODE_VAR_LEN}, /* Set variable length mode. */
    {CC1120_PKT_LEN,        RF_MAX_PKT_LEN},      /* Set maximum packet length */
};

static const s_rf_ch_settings_t gcs_rf_ch[E_RF_CHANNEL_MAX] = {
    { gcs_rf_cfg_ch_test, (sizeof(gcs_rf_cfg_ch_test)/sizeof(s_rf_register_t)) },     /* Test channel settings */
    { gcs_rf_cfg_ch_868mhz50bps, (sizeof(gcs_rf_cfg_ch_868mhz50bps)/sizeof(s_rf_register_t)) }, /* 868MHz@50kbps channel settings */
    { gcs_rf_cfg_ch_434mhz50bps, (sizeof(gcs_rf_cfg_ch_434mhz50bps)/sizeof(s_rf_register_t)) }, /* 434MHz@50kbps channel settings */
    { gcs_rf_cfg_ch_434mhz20bps, (sizeof(gcs_rf_cfg_ch_434mhz20bps)/sizeof(s_rf_register_t)) }, /* 434MHz@50kbps channel settings */
};


#if 1   // Used to optimize TX-RX turnaround
static const s_rf_register_t gcs_rf_cfg_ewor2[] = {

    {CC1120_IOCFG2,         0x06},                /* Enable IO2 to trigger at the reception of the sync word (rising edge) */
    {CC1120_IOCFG0,         0x00},                /* Enable IO to trigger on Rx-FIFO threshold limit */
    {CC1120_RFEND_CFG0,     0x01},                /* Rx termination on CS */
    {CC1120_PKT_CFG0,       RF_PKT_MODE_VAR_LEN}, /* Set variable length mode. */
    {CC1120_PKT_LEN,        RF_MAX_PKT_LEN},      /* Set maximum packet length */
};
#endif

#endif /* NETSTK_CFG_RF_CC1120_EN */

#endif /* #ifndef __RF_CFG_H__ */
