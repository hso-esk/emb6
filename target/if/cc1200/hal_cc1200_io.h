/*
 * hal_cc1200_io.h
 *
 *  Created on: Jan 23, 2014
 *      Author: Fesseha
 */

#ifndef HAL_CC1200_IO_H_
#define HAL_CC1200_IO_H_


/*============================================================================*/
/*                                  INCLUDES                                  */
/*============================================================================*/


/*============================================================================*/
/*                                  DEFINES                                   */
/*============================================================================*/
/* configuration registers */
#define CC1200_IOCFG3                   0x0000
#define CC1200_IOCFG2                   0x0001
#define CC1200_IOCFG1                   0x0002
#define CC1200_IOCFG0                   0x0003
#define CC1200_SYNC3                    0x0004
#define CC1200_SYNC2                    0x0005
#define CC1200_SYNC1                    0x0006
#define CC1200_SYNC0                    0x0007
#define CC1200_SYNC_CFG1                0x0008
#define CC1200_SYNC_CFG0                0x0009
#define CC1200_DEVIATION_M              0x000A
#define CC1200_MODCFG_DEV_E             0x000B
#define CC1200_DCFILT_CFG               0x000C
#define CC1200_PREAMBLE_CFG1            0x000D
#define CC1200_PREAMBLE_CFG0            0x000E
#define CC1200_IQIC                     0x000F
#define CC1200_CHAN_BW                  0x0010
#define CC1200_MDMCFG1                  0x0011
#define CC1200_MDMCFG0                  0x0012
#define CC1200_SYMBOL_RATE2             0x0013
#define CC1200_SYMBOL_RATE1             0x0014
#define CC1200_SYMBOL_RATE0             0x0015
#define CC1200_AGC_REF                  0x0016
#define CC1200_AGC_CS_THR               0x0017
#define CC1200_AGC_GAIN_ADJUST          0x0018
#define CC1200_AGC_CFG3                 0x0019
#define CC1200_AGC_CFG2                 0x001A
#define CC1200_AGC_CFG1                 0x001B
#define CC1200_AGC_CFG0                 0x001C
#define CC1200_FIFO_CFG                 0x001D
#define CC1200_DEV_ADDR                 0x001E
#define CC1200_SETTLING_CFG             0x001F
#define CC1200_FS_CFG                   0x0020
#define CC1200_WOR_CFG1                 0x0021
#define CC1200_WOR_CFG0                 0x0022
#define CC1200_WOR_EVENT0_MSB           0x0023
#define CC1200_WOR_EVENT0_LSB           0x0024
#define CC1200_RXDCM_TIME           	0x0025
#define CC1200_PKT_CFG2                 0x0026
#define CC1200_PKT_CFG1                 0x0027
#define CC1200_PKT_CFG0                 0x0028
#define CC1200_RFEND_CFG1               0x0029
#define CC1200_RFEND_CFG0               0x002A
#define CC1200_PA_CFG1                  0x002B
#define CC1200_PA_CFG0                  0x002C
#define CC1200_ASK_CFG                  0x002C
#define CC1200_PKT_LEN                  0x002E

/* Extended Configuration Registers */
#define CC1200_IF_MIX_CFG               0x2F00
#define CC1200_FREQOFF_CFG              0x2F01
#define CC1200_TOC_CFG                  0x2F02
#define CC1200_MARC_SPARE               0x2F03
#define CC1200_ECG_CFG                  0x2F04
#define CC1200_MDMCFG2                  0x2F05
//#define CC1200_CFM_DATA_CFG             0x2F05
#define CC1200_EXT_CTRL                 0x2F06
#define CC1200_RCCAL_FINE               0x2F07
#define CC1200_RCCAL_COARSE             0x2F08
#define CC1200_RCCAL_OFFSET             0x2F09
#define CC1200_FREQOFF1                 0x2F0A
#define CC1200_FREQOFF0                 0x2F0B
#define CC1200_FREQ2                    0x2F0C
#define CC1200_FREQ1                    0x2F0D
#define CC1200_FREQ0                    0x2F0E
#define CC1200_IF_ADC2                  0x2F0F
#define CC1200_IF_ADC1                  0x2F10
#define CC1200_IF_ADC0                  0x2F11
#define CC1200_FS_DIG1                  0x2F12
#define CC1200_FS_DIG0                  0x2F13
#define CC1200_FS_CAL3                  0x2F14
#define CC1200_FS_CAL2                  0x2F15
#define CC1200_FS_CAL1                  0x2F16
#define CC1200_FS_CAL0                  0x2F17
#define CC1200_FS_CHP                   0x2F18
#define CC1200_FS_DIVTWO                0x2F19
#define CC1200_FS_DSM1                  0x2F1A
#define CC1200_FS_DSM0                  0x2F1B
#define CC1200_FS_DVC1                  0x2F1C
#define CC1200_FS_DVC0                  0x2F1D
#define CC1200_FS_LBI                   0x2F1E
#define CC1200_FS_PFD                   0x2F1F
#define CC1200_FS_PRE                   0x2F20
#define CC1200_FS_REG_DIV_CML           0x2F21
#define CC1200_FS_SPARE                 0x2F22
#define CC1200_FS_VCO4                  0x2F23
#define CC1200_FS_VCO3                  0x2F24
#define CC1200_FS_VCO2                  0x2F25
#define CC1200_FS_VCO1                  0x2F26
#define CC1200_FS_VCO0                  0x2F27
#define CC1200_GBIAS6                   0x2F28
#define CC1200_GBIAS5                   0x2F29
#define CC1200_GBIAS4                   0x2F2A
#define CC1200_GBIAS3                   0x2F2B
#define CC1200_GBIAS2                   0x2F2C
#define CC1200_GBIAS1                   0x2F2D
#define CC1200_GBIAS0                   0x2F2E
#define CC1200_IFAMP                    0x2F2F
#define CC1200_LNA                      0x2F30
#define CC1200_RXMIX                    0x2F31
#define CC1200_XOSC5                    0x2F32
#define CC1200_XOSC4                    0x2F33
#define CC1200_XOSC3                    0x2F34
#define CC1200_XOSC2                    0x2F35
#define CC1200_XOSC1                    0x2F36
#define CC1200_XOSC0                    0x2F37
#define CC1200_ANALOG_SPARE             0x2F38
#define CC1200_PA_CFG3                  0x2F39
//#define CC1200_IRQ0M                    0x2F3F
//#define CC1200_IRQ0F                    0x2F40

/* Status Registers */
#define CC112X_WOR_TIME1                0x2F64
#define CC1200_WOR_TIME0                0x2F65
#define CC1200_WOR_CAPTURE1             0x2F66
#define CC1200_WOR_CAPTURE0             0x2F67
#define CC1200_BIST                     0x2F68
#define CC1200_DCFILTOFFSET_I1          0x2F69
#define CC1200_DCFILTOFFSET_I0          0x2F6A
#define CC1200_DCFILTOFFSET_Q1          0x2F6B
#define CC1200_DCFILTOFFSET_Q0          0x2F6C
#define CC1200_IQIE_I1                  0x2F6D
#define CC1200_IQIE_I0                  0x2F6E
#define CC1200_IQIE_Q1                  0x2F6F
#define CC1200_IQIE_Q0                  0x2F70
#define CC1200_RSSI1                    0x2F71
#define CC1200_RSSI0                    0x2F72
#define CC1200_MARCSTATE                0x2F73
#define CC1200_LQI_VAL                  0x2F74
#define CC1200_PQT_SYNC_ERR             0x2F75
#define CC1200_DEM_STATUS               0x2F76
#define CC1200_FREQOFF_EST1             0x2F77
#define CC1200_FREQOFF_EST0             0x2F78
#define CC1200_AGC_GAIN3                0x2F79
#define CC1200_AGC_GAIN2                0x2F7A
#define CC1200_AGC_GAIN1                0x2F7B
#define CC1200_AGC_GAIN0                0x2F7C
#define CC1200_CFM_RX_DATA_OUT          0x2F7D
#define CC1200_CFM_TX_DATA_IN           0x2F7E
#define CC1200_ASK_SOFT_RX_DATA         0x2F7F
#define CC1200_RNDGEN                   0x2F80		/* Random Generator? */
#define CC1200_MAGN2                    0x2F81
#define CC1200_MAGN1                    0x2F82
#define CC1200_MAGN0                    0x2F83
#define CC1200_ANG1                     0x2F84
#define CC1200_ANG0                     0x2F85
#define CC1200_CHFILT_I2                0x2F86
#define CC1200_CHFILT_I1                0x2F87
#define CC1200_CHFILT_I0                0x2F88
#define CC1200_CHFILT_Q2                0x2F89
#define CC1200_CHFILT_Q1                0x2F8A
#define CC1200_CHFILT_Q0                0x2F8B
#define CC1200_GPIO_STATUS              0x2F8C
#define CC1200_FSCAL_CTRL               0x2F8D
#define CC1200_PHASE_ADJUST             0x2F8E
#define CC1200_PARTNUMBER               0x2F8F
#define CC1200_PARTVERSION              0x2F90
#define CC1200_SERIAL_STATUS            0x2F91
#define CC1200_MODEM_STATUS1            0x2F92
#define CC1200_MODEM_STATUS0            0x2F93
#define CC1200_MARC_STATUS1             0x2F94
#define CC1200_MARC_STATUS0             0x2F95
#define CC1200_PA_IFAMP_TEST            0x2F96
#define CC1200_FSRF_TEST                0x2F97
#define CC1200_PRE_TEST                 0x2F98
#define CC1200_PRE_OVR                  0x2F99
#define CC1200_ADC_TEST                 0x2F9A
#define CC1200_DVC_TEST                 0x2F9B
#define CC1200_ATEST                    0x2F9C
#define CC1200_ATEST_LVDS               0x2F9D
#define CC1200_ATEST_MODE               0x2F9E
#define CC1200_XOSC_TEST1               0x2F9F
#define CC1200_XOSC_TEST0               0x2FA0
#define CC1200_AES                  	0x2FA1
#define CC1200_MDM_TEST               	0x2FA2

#define CC1200_RXFIRST                  0x2FD2
#define CC1200_TXFIRST                  0x2FD3
#define CC1200_RXLAST                   0x2FD4
#define CC1200_TXLAST                   0x2FD5
#define CC1200_NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */
#define CC1200_NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define CC1200_FIFO_NUM_TXBYTES         0x2FD8
#define CC1200_FIFO_NUM_RXBYTES         0x2FD9
#define CC1200_RXFIFO_PRE_BUF	        0x2FDA

/* DATA FIFO Access */
#define CC1200_SINGLE_TXFIFO            0x003F      /*  TXFIFO  - Single access to Transmit FIFO */
#define CC1200_BURST_TXFIFO             0x007F      /*  TXFIFO  - Burst access to Transmit FIFO  */
#define CC1200_SINGLE_RXFIFO            0x00BF      /*  RXFIFO  - Single access to Receive FIFO  */
#define CC1200_BURST_RXFIFO             0x00FF      /*  RXFIFO  - Burst access to Receive FIFO  */

#define CC1200_LQI_CRC_OK_BM            0x80
#define CC1200_LQI_EST_BM               0x7F

/* Command strobe registers */
#define CC1200_SRES                     0x30      /*  SRES    - Reset chip. */
#define CC1200_SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define CC1200_SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define CC1200_SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define CC1200_SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define CC1200_STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define CC1200_SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define CC1200_SAFC                     0x37      /*  SAFC    - Automatic Frequency Correction */
#define CC1200_SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define CC1200_SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define CC1200_SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define CC1200_SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define CC1200_SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define CC1200_SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */


/* Chip states returned in status byte */
#define CC1200_STATE_IDLE               0x00
#define CC1200_STATE_RX                 0x10
#define CC1200_STATE_TX                 0x20
#define CC1200_STATE_FSTXON             0x30
#define CC1200_STATE_CALIBRATE          0x40
#define CC1200_STATE_SETTLING           0x50
#define CC1200_STATE_RXFIFO_ERROR       0x60
#define CC1200_STATE_TXFIFO_ERROR       0x70
#define CC1200_STATE_CHIP_RDYn			0x80

/* Defines used for the manual calibration */
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

/* For CC1200 with 32 MHz XOSC. VCO frequency setting. */
static const uint8_t ac_freqConfiguration[5][3] = { { 0x69, 0xF1, 0xFF }, // 169.5125 MHz
    { 0x6C, 0x80, 0x00 }, // 434 MHz
    { 0x6C, 0x80, 0x00 }, // 868 MHz
    { 0x72, 0x60, 0x00 }, // 915 MHz
    { 0x77, 0x60, 0x00 }  // 955 MHz
};

/* Band select settings for the LO divider (RF band selection).
 Out of lock detector enabled (bit 4) */
static const uint8_t ac_cc1200LODivider[5] = { 0x1A, // 169 MHz
    0x14, // 434 MHz
    0x12, // 868 MHz
    0x12, // 915 MHz
    0x12, // 955 MHz
    };

/* Values in this table must be or'ed with the PA_CFG2 register to account for
 * pa_shape_en
 * Formula: paPowerRamp[index] = dec2hex((wanted_dBm_level+18)*2-1)
 */
static const uint8_t ac_paPowerRamp[12] = { 0x3F, /*  14 dBm - highest power      - index 0 */
0x3B, /*  12 dBm - next highest power - index 1 */
0x37, /*  10 dBm -                    - index 2 */
0x33, /*   8 dBm -                    - index 3 */
0x2F, /*   6 dBm -                    - index 4 */
0x2B, /*   4 dBm -                    - index 5 */
0x27, /*   2 dBm -                    - index 6 */
0x23, /*   0 dBm -                    - index 7 */
0x1D, /*  -3 dBm -                    - index 8 */
0x17, /*  -6 dBm -                    - index 9 */
0x0F, /* -10 dBm - next lowest power  - index 10 */
0x03 /*  -16 dBm - lowest power       - index 11 */
};

/*============================================================================*/
/*                                    MACROS                                  */
/*============================================================================*/

/*============================================================================*/
/*                                TYPEDEF ENUMS                               */
/*============================================================================*/

/*============================================================================*/
/*                              TYPEDEF STRUCTURES                            */
/*============================================================================*/
typedef uint8_t trxStatus_t;

typedef struct REGISTERSETTING_T {
  uint16_t i_address;
  uint8_t c_data;
} registerSetting_t;

static const registerSetting_t trxSettings[] = { { CC1200_IOCFG3, 0x00 }, {
CC1200_IOCFG2, 0x30 }, { CC1200_IOCFG1, 0x30 }, { CC1200_IOCFG0, 0x06 } };

// RX filter BW = 50.000000
// Address config = Address check, no broadcast
// Packet length = 255
// Symbol rate = 1.2
// PA ramping = true
// Performance mode = High Performance
// Carrier frequency = 434.000000
// Bit rate = 1.2
// Packet bit length = 0
// Whitening = false
// Manchester enable = false
// Modulation format = 2-FSK
// Packet length mode = Variable
// Device address = 0
// TX power = 15
// Deviation = 20.019531

static const registerSetting_t preferredSettings1[] = {
    { CC1200_IOCFG3, 0xB0 },
    { CC1200_IOCFG2, 0x06 },
    { CC1200_IOCFG1, 0xB0 },
    { CC1200_IOCFG0, 0x40 },
    { CC1200_SYNC_CFG1, 0x0B },
    { CC1200_DEVIATION_M, 0x48 },
    { CC1200_MODCFG_DEV_E, 0x05 },
    { CC1200_DCFILT_CFG, 0x1C },
    { CC1200_IQIC, 0x00 },
    { CC1200_CHAN_BW, 0x04 },
    { CC1200_MDMCFG0, 0x05 },
    { CC1200_AGC_CS_THR, 0x48 }, // Threshold lowered to -30/-45dB
    { CC1200_AGC_CFG1, 0xA0 },
    { CC1200_FIFO_CFG, 0x00 },
    { CC1200_SETTLING_CFG, 0x03 },
    { CC1200_FS_CFG, 0x14 },
    { CC1200_WOR_CFG0, 0x20 },
    { CC1200_WOR_EVENT0_MSB, 0x02 },
    { CC1200_WOR_EVENT0_LSB, 0xEA },
    { CC1200_PKT_CFG1, 0x15 },
    { CC1200_PKT_CFG0, 0x20 }, // Variable packet length mode
    { CC1200_RFEND_CFG0, 0x09 },
    { CC1200_PKT_LEN, 0xFF },
    { CC1200_IF_MIX_CFG, 0x00 },
    { CC1200_FREQOFF_CFG, 0x22 },
    { CC1200_FREQ2, 0x6C },
    { CC1200_FREQ1, 0x80 },
    { CC1200_FS_DIG1, 0x00 },
    { CC1200_FS_DIG0, 0x5F },
    { CC1200_FS_CAL1, 0x40 },
    { CC1200_FS_CAL0, 0x0E },
    { CC1200_FS_DIVTWO, 0x03 },
    { CC1200_FS_DSM0, 0x33 },
    { CC1200_FS_DVC0, 0x17 },
    { CC1200_FS_PFD, 0x50 },
    { CC1200_FS_PRE, 0x6E },
    { CC1200_FS_REG_DIV_CML, 0x14 },
    { CC1200_FS_SPARE, 0xAC },
    { CC1200_FS_VCO0, 0xB4 },
    { CC1200_XOSC5, 0x0E },
    { CC1200_XOSC2, 0x00 }
};

// RX filter BW = 50.000000
// Address config = Address check, no broadcast
// Packet length = 255
// Symbol rate = 2.4
// PA ramping = true
// Performance mode = High Performance
// Carrier frequency = 434.000000
// Bit rate = 2.4
// Packet bit length = 0
// Whitening = false
// Manchester enable = false
// Modulation format = 2-FSK
// Packet length mode = Variable
// Device address = 0
// TX power = 15
// Deviation = 20.019531

static const registerSetting_t preferredSettings2[] = {
    { CC1200_IOCFG3, 0xB0 },
    { CC1200_IOCFG2, 0x06 },
    { CC1200_IOCFG1, 0xB0 },
    { CC1200_IOCFG0, 0x40 },
    { CC1200_SYNC_CFG1, 0x0B },
    { CC1200_DEVIATION_M, 0x48 },
    { CC1200_MODCFG_DEV_E, 0x05 },
    { CC1200_DCFILT_CFG, 0x1C },
    { CC1200_IQIC, 0x00 },
    { CC1200_CHAN_BW, 0x04 },
    { CC1200_MDMCFG0, 0x05 },
    { CC1200_SYMBOL_RATE2, 0x53 },
    { CC1200_AGC_CS_THR, 0x0C },
    { CC1200_AGC_CFG1, 0xA0 },
    { CC1200_FIFO_CFG, 0x00 },
    { CC1200_SETTLING_CFG, 0x03 },
    { CC1200_FS_CFG, 0x14 },
    { CC1200_WOR_CFG0, 0x20 },
    { CC1200_WOR_EVENT0_MSB, 0x03 },
    { CC1200_WOR_EVENT0_LSB, 0x20 },
    { CC1200_PKT_CFG1, 0x15 },
    { CC1200_PKT_CFG0, 0x20 }, // Variable packet length mode
    { CC1200_RFEND_CFG0, 0x09 },
    { CC1200_PA_CFG0, 0x7E },
    { CC1200_PKT_LEN, 0xFF },
    { CC1200_IF_MIX_CFG, 0x00 },
    { CC1200_FREQOFF_CFG, 0x22 },
    { CC1200_FREQ2, 0x6C },
    { CC1200_FREQ1, 0x80 },
    { CC1200_FS_DIG1, 0x00 },
    { CC1200_FS_DIG0, 0x5F },
    { CC1200_FS_CAL1, 0x40 },
    { CC1200_FS_CAL0, 0x0E },
    { CC1200_FS_DIVTWO, 0x03 },
    { CC1200_FS_DSM0, 0x33 },
    { CC1200_FS_DVC0, 0x17 },
    { CC1200_FS_PFD, 0x50 },
    { CC1200_FS_PRE, 0x6E },
    { CC1200_FS_REG_DIV_CML, 0x14 },
    { CC1200_FS_SPARE, 0xAC },
    { CC1200_FS_VCO0, 0xB4 },
    { CC1200_XOSC5, 0x0E },
    { CC1200_XOSC2, 0x00 }
};
/*============================================================================*/
/*                                  FUNCTIONS                                 */
/*============================================================================*/
#ifdef __cplusplus
extern "C"
{
#endif

trxStatus_t cc1200_spiReadReg( uint16_t i_addr, uint8_t *pc_data,
    uint8_t c_len );
trxStatus_t cc1200_spiWriteReg( uint16_t i_addr, uint8_t *pc_data,
    uint8_t c_len );
trxStatus_t cc1200_spiWriteTxFifo( uint8_t *pc_writeData, uint8_t c_len );
trxStatus_t cc1200_spiReadRxFifo( uint8_t *pc_readData, uint8_t c_len );
trxStatus_t cc1200_getTxStatus( void );
trxStatus_t cc1200_getRxStatus( void );
void cc1200_enterIdle( void );

void cc1200_enterWor( void );
void cc1200_enterSleep( void );
void cc1200_enterRx( void );

void cc1200_manualCalibration( void );
void cc1200_calibrateRCOsc( void );


uint8_t cc1200_spiCmdStrobe (uint8_t uc_cmd);
uint8_t cc1200_spi8bitRegAccess( uint8_t uc_accessType, uint8_t uc_addressByte, uint8_t *puc_data, uint8_t uc_len );
uint8_t cc1200_spi16BitRegAccess(uint8_t uc_accessType, uint8_t uc_extendedAddr, uint8_t uc_regAddr, uint8_t *puc_data, uint8_t uc_len );

#ifdef __cplusplus
}
#endif

#endif /* HAL_CC1200_IO_H_ */
