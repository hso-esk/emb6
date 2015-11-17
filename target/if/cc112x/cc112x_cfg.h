/**
 * @file    CC112X_cfg.h
 * @date    12.11.2015
 * @author  PN
 * @brief   Configuration file of TI transceiver CC112X
 */
#ifndef CC112X_CFG_PRESENT
#define CC112X_CFG_PRESENT

#if NETSTK_CFG_RF_CC112X_EN
/**
 * @brief   IEEE802.15.4g first channel center (channel 0)
 *          See also IEEE802.15.4g-2012, table 68d
 *
 *          Carrier Frequency:  863.125MHz
 *          Modulation format:  MR-FSK
 *          Channel spacing:    200kHz
 *
 *          Symbol rate:        50kbps
 *          Bit rate:           50kbps
 *          RX filter BW:       100kHz
 *          Modulation format:  2-FGSK
 *          Deviation:          25kHz
 *          TX power:           15dBm - maximum
 *          RX Sniff mode:      enabled - eWOR
 *          Preamble length:    24bytes
 *          RX termination:     Carrier Sense with threshold of -90dBm
 */
static const s_regSettings_t rf_cfg_ieee802154g_chan0[] =
{
    {CC112X_IOCFG3,             0x0F},  /* CCA_STATUS */
    {CC112X_IOCFG2,             0x13},
    {CC112X_IOCFG1,             0xB0},
    {CC112X_IOCFG0,             0x06},

    {CC112X_SYNC3,              0x93},
    {CC112X_SYNC2,              0x0B},
    {CC112X_SYNC1,              0x51},
    {CC112X_SYNC0,              0xDE},
    {CC112X_SYNC_CFG1,          0x08},  /* AUTO_CLEAR = 1, enabled; RX_CONFIG_LIMITATION = 0 */
    {CC112X_SYNC_CFG0,          0x1B},  /* 16H bits */

    {CC112X_DEVIATION_M,        0x99},  /* Deviation 25kHz */
    {CC112X_MODCFG_DEV_E,       0x0D},  /* MOD_FORMAT = 001, 2-GFSK; DEV_E = 011, Frequency deviation */
    {CC112X_DCFILT_CFG,         0x15},

    {CC112X_PREAMBLE_CFG1,      0x30},  /* NUM_PREAMBLE = 1100, 24 bytes; PREAMBLE_WORD = 00, 0xAA */
    {CC112X_PREAMBLE_CFG0,      0x8A},  /* PQT_EN = 1, Preamble detection disabled; PQT = 1010, Soft Decision PQT*/

    {CC112X_FREQ_IF_CFG,        0x3A},
    {CC112X_IQIC,               0x00},
    {CC112X_CHAN_BW,            0x02},
    {CC112X_MDMCFG0,            0x05},

    {CC112X_SYMBOL_RATE2,       0x99},  /* Symbol rate: 50kbps */
    {CC112X_SYMBOL_RATE1,       0x99},
    {CC112X_SYMBOL_RATE0,       0x99},

    {CC112X_AGC_REF,            0x3C},
    {CC112X_AGC_CS_THR,         0x0C},
    {CC112X_AGC_CFG1,           0xA0},
    {CC112X_AGC_CFG0,           0xC0},
    {CC112X_FIFO_CFG,           0x80},  /* Automatically flushes when CRC error occurred */
    {CC112X_SETTLING_CFG,       0x03},
    {CC112X_FS_CFG,             0x12},

    {CC112X_WOR_CFG0,           0x20},
    {CC112X_WOR_EVENT0_MSB,     0x00},
    {CC112X_WOR_EVENT0_LSB,     0x78},  /* t_sleep = 3.102ms */

    {CC112X_PKT_CFG2,           0x04},
    {CC112X_PKT_CFG1,           0x05},
    {CC112X_PKT_CFG0,           0x20},

    {CC112X_RFEND_CFG0,         0x09},
    {CC112X_PA_CFG0,            0x79},
    {CC112X_PKT_LEN,            0xFF},
    {CC112X_IF_MIX_CFG,         0x00},
    {CC112X_TOC_CFG,            0x0A},

    {CC112X_FREQ2,              0x6B},    /* Carrier frequency: 863.125MHz */
    {CC112X_FREQ1,              0xE4},
    {CC112X_FREQ0,              0x00},

    /*
     * Miscellaneous, needed
     */
    {CC112X_FS_DIG1,            0x00},
    {CC112X_FS_DIG0,            0x5F},
    {CC112X_FS_CAL1,            0x40},
    {CC112X_FS_CAL0,            0x0E},
    {CC112X_FS_DIVTWO,          0x03},
    {CC112X_FS_DSM0,            0x33},
    {CC112X_FS_DVC0,            0x17},
    {CC112X_FS_PFD,             0x50},
    {CC112X_FS_PRE,             0x6E},
    {CC112X_FS_REG_DIV_CML,     0x14},
    {CC112X_FS_SPARE,           0xAC},
    {CC112X_FS_VCO0,            0xB4},
    {CC112X_XOSC5,              0x0E},
    {CC112X_XOSC2,              0x00},
    {CC112X_XOSC1,              0x03},
};

/**
 * @brief   RF registers with following configurations
 *          Carrier Frequency:  434MHz
 *          Symbol rate:        50kbps
 *          Bit rate:           50kpbs
 *          RX filter BW:       100kHz
 *          Modulation format:  2-FGSK
 *          Deviation:          25kHz
 *          TX power:           14dBm
 *
 *          RX Sniff mode:      enabled (eWOR)
 *          Preamble length:    24bytes
 *          RX termination:     Carrier Sense with threshold of -90dBm
 */
static const s_regSettings_t rf_cfg_ch_434mhz50bps[] =
{
    {CC112X_IOCFG3,             0x0F},  /* CCA_STATUS */
    {CC112X_IOCFG2,             0x13},
    {CC112X_IOCFG1,             0xB0},
    {CC112X_IOCFG0,             0x06},

    {CC112X_SYNC3,              0x93},
    {CC112X_SYNC2,              0x0B},
    {CC112X_SYNC1,              0x51},
    {CC112X_SYNC0,              0xDE},
    {CC112X_SYNC_CFG1,          0x08},  /* PQT gating enabled, sync threshold 0x08 */
    {CC112X_SYNC_CFG0,          0x1B},  /* 1B: 16H bit, 17: 32bit Sync */

    {CC112X_DEVIATION_M,        0x99},  /* Deviation = 24.963379 kHz */
    {CC112X_MODCFG_DEV_E,       0x0D},  /* Normal modem mode, 2-GFSK, Deviation = 20.019531 kHz */
    {CC112X_DCFILT_CFG,         0x15},
    {CC112X_PREAMBLE_CFG1,      0x30},  /* 24byte long preamble */
    {CC112X_FREQ_IF_CFG,        0x3A},
    {CC112X_CHAN_BW,            0x02},  /* Channel filter enabled, BW = 100kHz */

    {CC112X_SYMBOL_RATE2,       0x99},  /* Symbol Rate = 50ksps */
    {CC112X_SYMBOL_RATE1,       0x99},  /* Symbol Rate = 50ksps */
    {CC112X_SYMBOL_RATE0,       0x99},  /* Symbol Rate = 50ksps */

    /* WOR configuration */
    {CC112X_WOR_CFG0,           0x24},  /* enable clock division, disable Ev2, disable RCOSC calibration, enable RCOSC */
    {CC112X_WOR_EVENT0_MSB,     0x00},  /* tEv1 = 3.76ms */
    {CC112X_WOR_EVENT0_LSB,     0x78},  /* tEv1 = 3.76ms */

    {CC112X_AGC_REF,            0x3C},
    {CC112X_AGC_CS_THR,         0x00},  /* AGC Carrier Sense Threshold -102 dBm (+102dB offset!!) */
    {CC112X_AGC_CFG1,           0xA9},
    {CC112X_AGC_CFG0,           0xC0},

    {CC112X_PA_CFG0,            0x79},

    /* Frequency configuration 434MHz */
    {CC112X_FS_CFG,             0x14},
    {CC112X_FREQOFF_CFG,        0x20},
    {CC112X_FREQ2,              0x6C},
    {CC112X_FREQ1,              0x80},
    {CC112X_FREQ0,              0x00},

    {CC112X_FS_DIG1,            0x00},
    {CC112X_FS_DIG0,            0x5F},
    {CC112X_FS_CAL1,            0x40},
    {CC112X_FS_CAL0,            0x0E},
    {CC112X_FS_DIVTWO,          0x03},
    {CC112X_FS_DSM0,            0x33},
    {CC112X_FS_DVC0,            0x17},
    {CC112X_FS_PFD,             0x50},
    {CC112X_FS_PRE,             0x6E},
    {CC112X_FS_REG_DIV_CML,     0x14},
    {CC112X_FS_SPARE,           0xAC},
    {CC112X_FS_VCO0,            0xB4},
    {CC112X_XOSC5,              0x0E},
    {CC112X_XOSC2,              0x00},
    {CC112X_XOSC1,              0x03},
};

#endif

#endif /* CC112X_CFG_PRESENT */
