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
 * @file    CC120X_cfg.h
 * @date    12.11.2015
 * @author  PN
 * @brief   Configuration file of TI transceiver CC120X
 */

#ifndef CC120X_CFG_PRESENT
#define CC120X_CFG_PRESENT


/*
 * Value of channel frequency register is calculated using following equation:
 *      ChannelNumberFrequency = ChannelCenterFreq + ChannelIndex * DeltaFreq
 */
#define CC120X_CSM_DELTA_FREQ               0x00051Fu   /*!<   0.200 MHz */
#define CC120X_CSM_CHAN_CENTER_FREQ         0x564CCCu   /*!< 863.000 MHz */

#define CC120X_OPMODE1_DELTA_FREQ           0x00051Fu   /*!<   0.200 MHz */
#define CC120X_OPMODE1_CHAN_CENTER_FREQ     0x565000u   /*!< 863.125 MHz */

#define CC120X_OPMODE2_DELTA_FREQ           0x000A3Eu   /*!<   0.400 MHz */
#define CC120X_OPMODE2_CHAN_CENTER_FREQ     0x56528Fu   /*!< 863.225 MHz */

#define CC120X_OPMODE3_DELTA_FREQ           0x000A3Eu   /*!<   0.400 MHz */
#define CC120X_OPMODE3_CHAN_CENTER_FREQ     0x56528Fu   /*!< 863.225 MHz */


/**
 * @brief   GPIOs configuration for CCA operation
 */
static const s_regSettings_t cc120x_cfg_cca[] = {
    {CC120X_IOCFG3,             0x0F},  /* CCA_DONE, rising */
};


/**
 * @brief   GPIOs configuration for transmission operation
 */
static const s_regSettings_t cc120x_cfg_tx[] = {
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
    {CC120X_IOCFG0,             0x02},  /* TX_FIFO_THR, rising */
    {CC120X_IOCFG2,             0x06},  /* PKT_SYNC_RXTX, rising */

    /*
     * 0x00: fixed packet length mode
     * 0x20: variable packet length mode. Packet length is configured by the
     * first byte received after sync word;
     * 0x40: infinite packet length mode
     * By default the transceiver is configured to be infinite packet length
     * mode (0x40). Upon number of remaining bytes regardless in transmission or
     * reception, the packet length mode is configured accordingly (see note 1).
     */
    {CC120X_PKT_CFG0,           0x40},
    {CC120X_FIFO_CFG,           0x78},  /* FIFO_THR = 120 */
    /*
     * Packet length
     * In fixed length mode, this field indicates the packet length, and a value
     * of 0 indicates the length to be 256 bytes.
     * In variable packet length mode, this value indicates the maximum allowed
     * packet lengths.
     */
    {CC120X_PKT_LEN,            0xFF},
};

/**
 * @brief   GPIOs configuration for reception operation with eWOR enabled
 */
static const s_regSettings_t cc120x_cfg_rx_wor[] = {
    {CC120X_IOCFG0,             0x00},  /* RXFIFO_THR, rising */
    {CC120X_IOCFG2,             0x06},  /* PKT_SYNC_RXTX, rising */
    {CC120X_IOCFG3,             0x06},  /* PKT_SYNC_RXTX, falling */

    {CC120X_PKT_CFG0,           0x40},  /* packet length mode */
    {CC120X_FIFO_CFG,           0x78},  /* FIFO_THR = 120 */
    {CC120X_PKT_LEN,            0xFF},  /* packet length of 255 bytes */
};


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
 *          Modulation format:  2-FSK
 *          Deviation:          25kHz
 *          TX power:           15dBm - maximum
 *          Preamble length:    4bytes with WOR-disabled OR 24 byte with WOR-enabled
 *          RX termination:     Carrier Sense with threshold of -90dBm
 */
static const s_regSettings_t cc120x_cfg_ieee802154g_chan0[] =
{
    {CC120X_IOCFG3,             0x0F},  /* CCA_STATUS */
    {CC120X_IOCFG2,             0x13},  /* PKT_CRC_OK       RX_STARTED  */
    {CC120X_IOCFG1,             0xB0},
    {CC120X_IOCFG0,             0x06},  /* PKT_SYNC_RXTX    TX_FINISH   */

    {CC120X_SYNC3,              0x72},  /* SFD[15-8] FEC not supported and phyMRFSKSFD = 0 -> SFD = 0x7209 */
    {CC120X_SYNC2,              0x09},  /* SFD[ 7-0] ... */
    {CC120X_SYNC1,              0x51},  /* don't care */
    {CC120X_SYNC0,              0xDE},  /* don't care */
    {CC120X_SYNC_CFG1,          0xC8},  /* 16H bits SYCN3-SYNC2 */
    {CC120X_SYNC_CFG0,          0x23},  /* AUTO_CLEAR = 1, enabled; RX_CONFIG_LIMITATION = 0 */

    {CC120X_DEVIATION_M,        0x48},  /* Deviation 25kHz */
    {CC120X_MODCFG_DEV_E,       0x03},  /* MOD_FORMAT = 2-FSK; */
    {CC120X_DCFILT_CFG,         0x4B},

    {CC120X_PREAMBLE_CFG1,      0x19},  /* NUM_PREAMBLE = 4 bytes; PREAMBLE_WORD = 0x55 */
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
    {CC120X_PKT_CFG1,           0x00},    /* CRC_CFG = 00; CRC16 is disabled, APPEND_STATUS is disabled */
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
