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
 * @file    CC112X_cfg.h
 * @date    12.11.2015
 * @author  PN
 * @brief   Configuration file of TI transceiver CC112X
 */
#ifndef CC112X_CFG_PRESENT
#define CC112X_CFG_PRESENT


/*
 * Value of channel frequency register is calculated using following equation:
 *      ChannelNumberFrequency = ChannelCenterFreq + ChannelIndex * DeltaFreq
 */
#define CC112X_CSM_DELTA_FREQ               0x000666u   /*!<   0.200 MHz */
#define CC112X_CSM_CHAN_CENTER_FREQ         0x6BE000u   /*!< 863.000 MHz */

#define CC112X_OPMODE1_DELTA_FREQ           0x000666u   /*!<   0.200 MHz */
#define CC112X_OPMODE1_CHAN_CENTER_FREQ     0x6BE400u   /*!< 863.125 MHz */

#define CC112X_OPMODE2_DELTA_FREQ           0x000CCCu   /*!<   0.400 MHz */
#define CC112X_OPMODE2_CHAN_CENTER_FREQ     0x6BE733u   /*!< 863.225 MHz */

#define CC112X_OPMODE3_DELTA_FREQ           0x000CCCu   /*!<   0.400 MHz */
#define CC112X_OPMODE3_CHAN_CENTER_FREQ     0x6BE733u   /*!< 863.225 MHz */


/**
 * @brief   GPIOs configuration for CCA operation
 */
static const s_regSettings_t cc112x_cfg_cca[] = {
    {CC112X_IOCFG3,             0x0F},  /* CCA_DONE, rising */
};


/**
 * @brief   GPIOs configuration for transmission operation
 */
static const s_regSettings_t cc112x_cfg_tx[] = {
    /*
     * Note 1:
     *
     * TX_FIFO_THR, asserted when TX FIFO is filled Above or Equal to
     * (127-FIFO_CFG.FIGO_THR) or the end of packet is reached
     * De-asserted when the TX FIFO is drained below the same threshold.
     * This signal is also available in the MODEM_STATUS0 register.
     *
     * If byte_lefts is equal or above the threshold, an amount of byte equal to
     * threshold is written into TX FIFO for transmission. This then causes
     * TX_FIFO_THR interrupt. Here FIFO is configured to infinite packet length
     * If the byte left is below the threshold, FIFO shall be configured to
     * fixed packet length. Afterwards remaining bytes are written into TX FIFO
     * for transmission. At the end of the transmission PKT_SYCN_RXTX is de-
     * asserted.
     */
    {CC112X_IOCFG0,             0x02},  /* TX_FIFO_THR, rising */
    {CC112X_IOCFG2,             0x06},  /* PKT_SYNC_RXTX, rising */

    /*
     * 0x00: fixed packet length mode
     * 0x20: variable packet length mode. Packet length is configured by the
     * first byte received after sync word;
     * 0x40: infinite packet length mode
     * By default the transceiver is configured to be infinite packet length
     * mode (0x40). Upon number of remaining bytes regardless in transmission or
     * reception, the packet length mode is configured accordingly (see note 1).
     */
    {CC112X_PKT_CFG0,           0x40},
    {CC112X_FIFO_CFG,           0x78},  /* FIFO_THR = 120 */
    /*
     * Packet length
     * In fixed length mode, this field indicates the packet length, and a value
     * of 0 indicates the length to be 256 bytes.
     * In variable packet length mode, this value indicates the maximum allowed
     * packet lengths.
     */
    {CC112X_PKT_LEN,            0xFF},
};

/**
 * @brief   GPIOs configuration for reception operation with eWOR enabled
 */
static const s_regSettings_t cc112x_cfg_rx_wor[] = {
    {CC112X_IOCFG0,             0x00},  /* RXFIFO_THR, rising */
    {CC112X_IOCFG2,             0x06},  /* PKT_SYNC_RXTX, rising */
    {CC112X_IOCFG3,             0x06},  /* PKT_SYNC_RXTX, falling */

    {CC112X_PKT_CFG0,           0x40},  /* packet length mode */
    {CC112X_FIFO_CFG,           0x78},  /* FIFO_THR = 120 */
    {CC112X_PKT_LEN,            0xFF},  /* packet length of 255 bytes */
};


/**
 * @brief   Default register settings for IEEE-802.15.4g
 *          Carrier Frequency:  863.125MHz
 *          Modulation format:  MR-FSK
 *          Channel spacing:    200kHz
 *          Symbol rate:        50kbps
 *          Bit rate:           50kbps
 *          RX filter BW:       100kHz
 *          Modulation format:  2-FSK
 *          Deviation:          25kHz
 *          TX power:           15dBm - maximum
 *          Preamble length:    4bytes with WOR-disabled
 *          RX termination:     Carrier Sense with threshold of -90dBm
 */
static const s_regSettings_t cc112x_cfg_ieee802154g_default[] =
{
    {CC112X_IOCFG0,             0xB0},  /* Impedance */
    {CC112X_IOCFG2,             0xB0},  /* Impedance */
    {CC112X_IOCFG3,             0xB0},  /* Impedance */
    {CC112X_IOCFG1,             0xB0},  /* Impedance */

    {CC112X_SYNC3,              0x72},  /* SFD[15-8] FEC not supported and phyMRFSKSFD = 0 -> SFD = 0x7209 */
    {CC112X_SYNC2,              0x09},  /* SFD[ 7-0] ... */
    {CC112X_SYNC1,              0x51},  /* don't care */
    {CC112X_SYNC0,              0xDE},  /* don't care */
    {CC112X_SYNC_CFG1,          0x08},  /* AUTO_CLEAR = 1, enabled; RX_CONFIG_LIMITATION = 0 */
    {CC112X_SYNC_CFG0,          0x1B},  /* 16H bits SYCN3-SYNC2 */

    {CC112X_DEVIATION_M,        0x99},  /* Deviation 25kHz */
    {CC112X_MODCFG_DEV_E,       0x05},  /* MOD_FORMAT = 2-FSK */
    {CC112X_DCFILT_CFG,         0x15},

    {CC112X_PREAMBLE_CFG1,      0x19},  /* NUM_PREAMBLE = 4 bytes; PREAMBLE_WORD = 0x55 */
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
    {CC112X_AGC_CFG0,           0xC0},  /* RSSI_VALID_CNT = 00b; 0x02 */
    {CC112X_FIFO_CFG,           0x80},  /* Automatically flushes when CRC error occurred */
    {CC112X_SETTLING_CFG,       0x03},
    {CC112X_FS_CFG,             0x12},

    {CC112X_WOR_CFG0,           0x20},
    {CC112X_WOR_EVENT0_MSB,     0x00},
    {CC112X_WOR_EVENT0_LSB,     0x78},  /* t_sleep = 3.102ms */

    {CC112X_PKT_CFG2,           0x04},  /* CCA_MODE=001b, indicating clear channel when RSSI is below threshold */
    {CC112X_PKT_CFG1,           0x00},  /* CRC_CFG = 00; CRC16 is disabled, APPEND_STATUS is disabled */
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
static const s_regSettings_t cc112x_cfg_ch_434mhz50bps[] =
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
