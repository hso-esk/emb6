/**
 * @file    CC120X_cfg.h
 * @date    12.11.2015
 * @author  PN
 * @brief   Configuration file of TI transceiver CC120X
 */

#ifndef CC120X_CFG_PRESENT
#define CC120X_CFG_PRESENT


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
static const s_regSettings_t cc120x_cfg_ieee802154g_chan0[] =
{
    {CC120X_IOCFG3,             0x0F},  /* CCA_STATUS */
    {CC120X_IOCFG2,             0x13},  /* PKT_CRC_OK       RX_STARTED  */
    {CC120X_IOCFG1,             0xB0},
    {CC120X_IOCFG0,             0x06},  /* PKT_SYNC_RXTX    TX_FINISH   */

    {CC120X_SYNC3,              0x93},
    {CC120X_SYNC2,              0x0B},
    {CC120X_SYNC1,              0x51},
    {CC120X_SYNC0,              0xDE},
    {CC120X_SYNC_CFG1,          0xC8},  /* 16H bits */
    {CC120X_SYNC_CFG0,          0x23},  /* AUTO_CLEAR = 1, enabled; RX_CONFIG_LIMITATION = 0 */

    {CC120X_DEVIATION_M,        0x48},  /* Deviation 25kHz */
    {CC120X_MODCFG_DEV_E,       0x0B},  /* MOD_FORMAT = 001, 2-GFSK; DEV_E = 011, Frequency deviation */
    {CC120X_DCFILT_CFG,         0x4B},

    {CC120X_PREAMBLE_CFG1,      0x30},  /* NUM_PREAMBLE = 1100, 24 bytes; PREAMBLE_WORD = 00, 0xAA */
    {CC120X_PREAMBLE_CFG0,      0x8A},  /* PQT_EN = 1, Preamble detection disabled; PQT = 1010, Soft Decision PQT*/

    {CC120X_IQIC,               0x58},
    {CC120X_CHAN_BW,            0x84},  /* RX filter BW 104kHz*/
    {CC120X_MDMCFG1,            0x42},
    {CC120X_MDMCFG0,            0x05},

    {CC120X_SYMBOL_RATE2,       0x94},  /* Symbol rate: 50kbps */
    {CC120X_SYMBOL_RATE1,       0x7A},
    {CC120X_SYMBOL_RATE0,       0xE1},

    {CC120X_AGC_REF,            0x27},  /* */
    {CC120X_AGC_CS_THR,         0xFF},
    {CC120X_AGC_CFG1,           0x00},
    {CC120X_AGC_CFG0,           0x80},
    {CC120X_FIFO_CFG,           0x80},  /* Automatically flushes when CRC error occurred */
    {CC120X_SETTLING_CFG,       0x03},
    {CC120X_FS_CFG,             0x12},

    {CC120X_WOR_CFG0,           0x20},
    {CC120X_WOR_EVENT0_MSB,     0x00},
    {CC120X_WOR_EVENT0_LSB,     0x96},  /* t_sleep = 3.102ms */

    {CC120X_PKT_CFG2,           0x00},
    {CC120X_PKT_CFG1,           0x03},
    {CC120X_PKT_CFG0,           0x20},

    {CC120X_RFEND_CFG0,         0x09},

    {CC120X_PA_CFG0,            0x53},  /* Power Amplifier */
    {CC120X_PKT_LEN,            0xFF},  /* 128 bytes */

    {CC120X_IF_MIX_CFG,         0x1C},
    {CC120X_TOC_CFG,            0x03},
    {CC120X_MDMCFG2,            0x02},

    {CC120X_FREQ2,              0x56},  /* Carrier frequency: 863.125MHz */
    {CC120X_FREQ1,              0x50},
    {CC120X_FREQ0,              0x00},

    /*
     * Miscellaneous, needed
     */
    {CC120X_IF_ADC1,            0xEE},
    {CC120X_IF_ADC0,            0x10},
    {CC120X_FS_DIG1,            0x07},
    {CC120X_FS_DIG0,            0xA5},
    {CC120X_FS_CAL1,            0x40},
    {CC120X_FS_CAL0,            0x0E},
    {CC120X_FS_DIVTWO,          0x03},
    {CC120X_FS_DSM0,            0x33},
    {CC120X_FS_DVC0,            0x17},
    {CC120X_FS_PFD,             0x00},
    {CC120X_FS_PRE,             0x6E},
    {CC120X_FS_REG_DIV_CML,     0x1C},
    {CC120X_FS_SPARE,           0xAC},
    {CC120X_FS_VCO0,            0xB5},
    {CC120X_IFAMP,              0x09},
    {CC120X_XOSC5,              0x0E},
    {CC120X_XOSC1,              0x03},
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
static const s_regSettings_t cc120x_cfg_ch_434mhz50bps[] =
{
    {CC120X_IOCFG3,         0x0F},  /* CCA_STATUS */
    {CC120X_IOCFG2,         0x13},
    {CC120X_IOCFG1,         0xB0},
    {CC120X_IOCFG0,         0x06},

    {CC120X_SYNC3,          0x93},
    {CC120X_SYNC2,          0x0B},
    {CC120X_SYNC1,          0x51},
    {CC120X_SYNC0,          0xDE},
    {CC120X_SYNC_CFG1,      0xC8},  /* 16H bits; */
    {CC120X_SYNC_CFG0,      0x23},  /* AUTO_CLEAR = 1, enabled; RX_CONFIG_LIMITATION = 0, */

    {CC120X_DEVIATION_M,    0x48},
    {CC120X_MODCFG_DEV_E,   0x0B},  /* MOD_FORMAT = 001, 2-GFSK; DEV_E = 011, Frequency deviation */
    {CC120X_DCFILT_CFG,     0x4B},

    {CC120X_PREAMBLE_CFG1,  0x30},  /* NUM_PREAMBLE = 1100, 24 bytes; PREAMBLE_WORD = 00, 0xAA*/
    {CC120X_PREAMBLE_CFG0,  0x8A},  /* PQT_EN = 1, Preamble detection disabled; PQT = 1010, Soft Decision PQT */

    {CC120X_IQIC,           0x58},
    {CC120X_CHAN_BW,        0x84},  /* RX filter BW 104kHz*/
    {CC120X_MDMCFG1,        0x42},
    {CC120X_MDMCFG0,        0x05},

    {CC120X_SYMBOL_RATE2,   0x94},  /* 434MHz */
    {CC120X_SYMBOL_RATE1,   0x7A},
    {CC120X_SYMBOL_RATE0,   0xE1},

    {CC120X_AGC_REF,        0x27},  /* */
    {CC120X_AGC_CS_THR,     0xF7},
    {CC120X_AGC_CFG1,       0x00},
    {CC120X_AGC_CFG0,       0x80},
    {CC120X_FIFO_CFG,       0x80},  /* Automatically flushes the last packet received if a CRC error occurred */

    {CC120X_SETTLING_CFG,   0x03},
    {CC120X_FS_CFG,         0x14},

    {CC120X_WOR_CFG0,       0x20},
    {CC120X_WOR_EVENT0_MSB, 0x00},
    {CC120X_WOR_EVENT0_LSB, 0x96},  /* t_sleep = 3.102ms */

    {CC120X_PKT_CFG2,       0x00},
    {CC120X_PKT_CFG1,       0x03},
    {CC120X_PKT_CFG0,       0x20},

    {CC120X_RFEND_CFG0,     0x09},
    {CC120X_PA_CFG0,        0x53},  /* Power Amplifier Configuration Reg. 0 */
    {CC120X_PKT_LEN,        0xFF},

    {CC120X_IF_MIX_CFG,     0x1C},
    {CC120X_TOC_CFG,        0x03},
    {CC120X_MDMCFG2,        0x02},

    {CC120X_FREQ2,          0x56},
    {CC120X_FREQ1,          0xCC},
    {CC120X_FREQ0,          0xCC},

    /*
     * Miscellaneous, for testing purposes only
     */
    {CC120X_IF_ADC1,        0xEE},
    {CC120X_IF_ADC0,        0x10},
    {CC120X_FS_DIG1,        0x07},
    {CC120X_FS_DIG0,        0xAF},
    {CC120X_FS_CAL1,        0x40},
    {CC120X_FS_CAL0,        0x0E},
    {CC120X_FS_DIVTWO,      0x03},
    {CC120X_FS_DSM0,        0x33},
    {CC120X_FS_DVC0,        0x17},
    {CC120X_FS_PFD,         0x00},
    {CC120X_FS_PRE,         0x6E},
    {CC120X_FS_REG_DIV_CML, 0x1C},
    {CC120X_FS_SPARE,       0xAC},
    {CC120X_FS_VCO0,        0xB5},
    {CC120X_IFAMP,          0x09},
    {CC120X_XOSC5,          0x0E},
    {CC120X_XOSC1,          0x03},
};

#endif /* CC120X_CFG_PRESENT */
