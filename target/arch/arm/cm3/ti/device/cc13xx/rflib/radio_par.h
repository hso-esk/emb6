/******************************************************************************
*  Filename:       radio_par.h
*  Revised:        $ $
*  Revision:       $ $
*
*  Description:    Definition of radio parameter override registers
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*  TI Confidential - NDA Restrictions
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#ifndef __RADIO_PAR_H
#define __RADIO_PAR_H

#ifndef __RFC_STRUCT
#ifdef __GNUC__
#define __RFC_STRUCT __attribute__ ((aligned (4)))
#else
#define __RFC_STRUCT
#endif
#endif

//! \addtogroup rfc
//! @{

//! \addtogroup radio_par
//! @{

#include <stdint.h>

/// Address of the array in RFCORE RAM
#define RFC_CFG_ADDR 0x21000028

typedef struct __RFC_STRUCT rfc_radioPar_s rfc_radioPar_t;
typedef struct __RFC_STRUCT rfc_synthPar_s rfc_synthPar_t;
typedef struct __RFC_STRUCT rfc_frontEndPar_s rfc_frontEndPar_t;
typedef struct __RFC_STRUCT rfc_bleRadioPar_s rfc_bleRadioPar_t;
typedef struct __RFC_STRUCT rfc_ieeeRadioPar_s rfc_ieeeRadioPar_t;
typedef struct __RFC_STRUCT rfc_propRadioPar_s rfc_propRadioPar_t;

//! \addtogroup radioPar
//! @{
#define _POSITION_radioPar_maxFutureTime                        0
#define _TYPE_radioPar_maxFutureTime                            uint16_t
#define _POSITION_radioPar_numSwBits                            2
#define _TYPE_radioPar_numSwBits                                uint8_t
#define _POSITION_radioPar_preamCtrl                            3
#define _TYPE_radioPar_preamCtrl                                uint8_t
#define _POSITION_radioPar_preamPattern                         4
#define _TYPE_radioPar_preamPattern                             uint16_t
#define _POSITION_radioPar_bNoPtrCheck                          6
#define _TYPE_radioPar_bNoPtrCheck                              uint8_t
#define _POSITION_radioPar_rssiOverride                         7
#define _TYPE_radioPar_rssiOverride                             int8_t
#define _POSITION_radioPar_modIsfCfgVal                         8
#define _TYPE_radioPar_modIsfCfgVal                             uint32_t
#define _BITPOS_radioPar_modIsfCfgVal_moduleEn                  0
#define _NBITS_radioPar_modIsfCfgVal_moduleEn                   1
#define _BITPOS_radioPar_modIsfCfgVal_intpFactor                2
#define _NBITS_radioPar_modIsfCfgVal_intpFactor                 2
#define _BITPOS_radioPar_modIsfCfgVal_shapeGain                 4
#define _NBITS_radioPar_modIsfCfgVal_shapeGain                  2
#define _BITPOS_radioPar_modIsfCfgVal_useZU                     6
#define _NBITS_radioPar_modIsfCfgVal_useZU                      1
#define _BITPOS_radioPar_modIsfCfgVal_useShaped154              7
#define _NBITS_radioPar_modIsfCfgVal_useShaped154               1
#define _BITPOS_radioPar_modIsfCfgVal_txOff                     8
#define _NBITS_radioPar_modIsfCfgVal_txOff                      12
#define _BITPOS_radioPar_modIsfCfgVal_bDynamicTxFreq            20
#define _NBITS_radioPar_modIsfCfgVal_bDynamicTxFreq             1
#define _POSITION_radioPar_pModIsfShape                         12
#define _TYPE_radioPar_pModIsfShape                             uint32_t const *
#define _POSITION_radioPar_crcWhConfig                          16
#define _TYPE_radioPar_crcWhConfig                              uint8_t
#define _BITPOS_radioPar_crcWhConfig_bWhEn                      0
#define _NBITS_radioPar_crcWhConfig_bWhEn                       1
#define _BITPOS_radioPar_crcWhConfig_bWhBitRev                  1
#define _NBITS_radioPar_crcWhConfig_bWhBitRev                   1
#define _BITPOS_radioPar_crcWhConfig_bMsbFirst                  2
#define _NBITS_radioPar_crcWhConfig_bMsbFirst                   1
#define _BITPOS_radioPar_crcWhConfig_bNoSwPreamProg             3
#define _NBITS_radioPar_crcWhConfig_bNoSwPreamProg              1
#define _BITPOS_radioPar_crcWhConfig_bSwDepPream                4
#define _NBITS_radioPar_crcWhConfig_bSwDepPream                 1
#define _BITPOS_radioPar_crcWhConfig_bWhBeforeCrc               5
#define _NBITS_radioPar_crcWhConfig_bWhBeforeCrc                1
#define _BITPOS_radioPar_crcWhConfig_dualSwConfig               6
#define _NBITS_radioPar_crcWhConfig_dualSwConfig                2
#define _POSITION_radioPar_numCrcBits                           17
#define _TYPE_radioPar_numCrcBits                               uint8_t
#define _POSITION_radioPar_introduceCrcErr                      18
#define _TYPE_radioPar_introduceCrcErr                          uint16_t
#define _POSITION_radioPar_crcInit                              20
#define _TYPE_radioPar_crcInit                                  uint32_t
#define _POSITION_radioPar_crcXor                               24
#define _TYPE_radioPar_crcXor                                   uint32_t
#define _POSITION_radioPar_whInit                               28
#define _TYPE_radioPar_whInit                                   uint32_t
#define _POSITION_radioPar_startToTxRatOffset                   32
#define _TYPE_radioPar_startToTxRatOffset                       uint16_t
#define _POSITION_radioPar_startToRxRatOffset                   34
#define _TYPE_radioPar_startToRxRatOffset                       uint16_t
#define _POSITION_radioPar_startToSynthRatOffset                36
#define _TYPE_radioPar_startToSynthRatOffset                    uint16_t
#define _POSITION_radioPar_txToRxRatOffset                      38
#define _TYPE_radioPar_txToRxRatOffset                          uint16_t
#define _POSITION_radioPar_rxToTxRatOffset                      40
#define _TYPE_radioPar_rxToTxRatOffset                          uint16_t
#define _POSITION_radioPar_syncTimeAdjust                       42
#define _TYPE_radioPar_syncTimeAdjust                           uint16_t
#define _POSITION_radioPar_rxFifoThrSleep                       44
#define _TYPE_radioPar_rxFifoThrSleep                           uint8_t
#define _POSITION_radioPar_txFifoThrSleep                       45
#define _TYPE_radioPar_txFifoThrSleep                           uint8_t
#define _POSITION_radioPar_ratTicksPerBit                       46
#define _TYPE_radioPar_ratTicksPerBit                           uint16_t
#define _POSITION_radioPar_mdmFreqWordOvr                       48
#define _TYPE_radioPar_mdmFreqWordOvr                           uint16_t
#define _BITPOS_radioPar_mdmFreqWordOvr_freqWordOvr             0
#define _NBITS_radioPar_mdmFreqWordOvr_freqWordOvr              12
#define _BITPOS_radioPar_mdmFreqWordOvr_bOvrMdmRfCh             14
#define _NBITS_radioPar_mdmFreqWordOvr_bOvrMdmRfCh              1
#define _BITPOS_radioPar_mdmFreqWordOvr_bWrMdmRfCh              15
#define _NBITS_radioPar_mdmFreqWordOvr_bWrMdmRfCh               1
#define _POSITION_radioPar_mdmRxIntFreq                         50
#define _TYPE_radioPar_mdmRxIntFreq                             int16_t
#define _POSITION_radioPar_mdmTxIntFreq                         52
#define _TYPE_radioPar_mdmTxIntFreq                             int16_t
#define _POSITION_radioPar_mdmPShift                            54
#define _TYPE_radioPar_mdmPShift                                uint8_t
#define _POSITION_radioPar_mdmQShift                            55
#define _TYPE_radioPar_mdmQShift                                uint8_t
#define _POSITION_radioPar_txFreqBase                           56
#define _TYPE_radioPar_txFreqBase                               uint16_t
#define _POSITION_radioPar_freqOffset                           58
#define _TYPE_radioPar_freqOffset                               int16_t
#define _POSITION_radioPar_bawConfig                            60
#define _TYPE_radioPar_bawConfig                                uint32_t
#define _BITPOS_radioPar_bawConfig_preScale                     0
#define _NBITS_radioPar_bawConfig_preScale                      8
#define _BITPOS_radioPar_bawConfig_rateWord                     8
#define _NBITS_radioPar_bawConfig_rateWord                      21
#define _BITPOS_radioPar_bawConfig_bFsAdjust                    29
#define _NBITS_radioPar_bawConfig_bFsAdjust                     1
#define _BITPOS_radioPar_bawConfig_bSymbolRateAdjust            30
#define _NBITS_radioPar_bawConfig_bSymbolRateAdjust             1
#define _BITPOS_radioPar_bawConfig_bRatAdjust                   31
#define _NBITS_radioPar_bawConfig_bRatAdjust                    1
#define _SIZEOF_radioPar                                        64

struct __RFC_STRUCT rfc_radioPar_s {
   uint16_t maxFutureTime;              //!< \brief If the difference between two radio timer times is greater than
                                        //!<        <code>(maxFutureTime << 16)</code>, the difference is interpreted as being negative
   uint8_t numSwBits;                   //!<        Number of bits in the sync word
   uint8_t preamCtrl;                   //!<        Value of MODPRECTRL register when sending preamble
   uint16_t preamPattern;               //!<        Preamble pattern
   uint8_t bNoPtrCheck;                 //!<        If non-zero, pointers are checked for alignment only; not range
   int8_t rssiOverride;                 //!<        If different from 0x80, report this value as RSSI and maximum RSSI instead of the true value
   struct {
      uint32_t moduleEn:1;
      uint32_t :1;
      uint32_t intpFactor:2;            //!< \brief 0: Make no register accesses to the MOD ISF<br>
                                        //!<        1: Enable MOD ISF when Tx starts
      uint32_t shapeGain:2;             //!< \brief Value to write to INTPFACTOR field of MODISFCFG register:
                                        //!<        Interpolation factor of the shaping filter (1: 16. 2: 32. Others: <i>Reserved</i>)
      uint32_t useZU:1;                 //!< \brief Value to write to SHAPEGAIN field of MODISFCFG register:
                                        //!<        Multiplies the shape by a gain factor. 1/2/4/8.
      uint32_t useShaped154:1;          //!< \brief Value to write to USEZU field of MODISFCFG register:
                                        //!<        Use Zigbee modulator unint (0: No. 1: Yes)
      uint32_t txOff:12;                //!< \brief Value to write to USESHAPED154 field of MODISFCFG register:
                                        //!<        Use shaped IEEE 802.15.4 modulation. Only valid if USEZU is active. (0: No. 1: Yes)
      uint32_t bDynamicTxFreq:1;        //!< \brief Value to write to TXFOFF field of MODISFCFG register if bDynamicTxFreq is 0
                                        //!<        Tx frequency offset
   } modIsfCfgVal;                      //!< \brief 0: Write TXOFF as given in txOff field
                                        //!<        1: Calculate TXOFF based on mdmTxIntFreq and current RF frequency
   uint32_t const * pModIsfShape;       //!<        Pointer to Tx shaping filter coefficients; NULL means no coefficients are written.
   struct {
      uint8_t bWhEn:1;
      uint8_t bWhBitRev:1;              //!< \brief Whitener enabling <br>
                                        //!<        0: Whitener disabled <br>
                                        //!<        1: Whitener enabled
      uint8_t bMsbFirst:1;              //!< \brief Bit reversal of whitener bytes compared to data bytes <br>
                                        //!<        0: Normal operation <br>
                                        //!<        1: Whitener bytes bit reversed (CC2500 compatible)
      uint8_t bNoSwPreamProg:1;         //!< \brief Bit ordering (unless overridden in command) <br>
                                        //!<        0: Least significant bit first over the air<br>
                                        //!<        1: Most significant bit first over the air
      uint8_t bSwDepPream:1;            //!<        If 1, sync word and preamble are not programmed in MCE registers
      uint8_t bWhBeforeCrc:1;           //!<        If 1, invert (1's complement) preamble if sync word starts with 1
      uint8_t dualSwConfig:2;           //!<        If 1, whitening is done before CRC calculation in Tx (and vice versa in Rx)
   } crcWhConfig;                       //!< \brief Configuration of dual sync word if applicable
                                        //!<        0: No register modification
                                        //!<        1: Set correlation threshold high for unused sync word
                                        //!<        2: Program cascade configuration to single or parallel
                                        //!<        3: No register modification or sync word programming
   uint8_t numCrcBits;                  //!<        Number of CRC bits (0-32)
   uint16_t introduceCrcErr;            //!< \brief Vector of CRC error introduction for testing. If LSB is 1, a transmitted packet will have a
                                        //!<        bit error in its CRC and a received packet will always be reported as having a CRC error.
                                        //!<        After a packet has been received or transmitted, the value is shifted right by one bit.
   uint32_t crcInit;                    //!< \brief Initialization value for the CRC, written to the shift register prior to transmission or
                                        //!<        reception
   uint32_t crcXor;                     //!< \brief Value to be XOR-ed with the CRC after it has been calculated, before it is transmitted on
                                        //!<        Tx, and after it has been received and before it is checked on Rx
   uint32_t whInit;                     //!< \brief Initialization value for the whitener, written to the shift register prior to transmission
                                        //!<        or reception
   uint16_t startToTxRatOffset;         //!< \brief Number of RAT ticks from start of a radio operation doing Tx first to start of transmitted
                                        //!<        packet. Must not be too small, as processing needs to be done.
   uint16_t startToRxRatOffset;         //!< \brief Number of RAT ticks from start of a radio operation doing Rx first to start of receive
                                        //!<        operation. Must give time for synth stabilizing after mode change and processing.
   uint16_t startToSynthRatOffset;      //!< \brief Number of RAT ticks from start of a radio operation doing calibrating synth to synth can
                                        //!<        be expected to be in lock and Rx or Tx operation can be started
   uint16_t txToRxRatOffset;            //!< \brief Number of RAT ticks from end of transmitted packet to start of Rx when transitioning from
                                        //!<        Tx to Rx within one radio operation command
   uint16_t rxToTxRatOffset;            //!< \brief Number of RAT ticks from end of received packet to start of Tx when transitioning from
                                        //!<        Rx to Tx within one radio operation command
   uint16_t syncTimeAdjust;             //!< \brief Number of RAT ticks to subtract from the sync found time reported by the modem before
                                        //!<        reporting it further, in order to get a time that indicates the start of the packet
   uint8_t rxFifoThrSleep;              //!<        Maximum FIFO threshold to use when going to sleep in Rx
   uint8_t txFifoThrSleep;              //!<        FIFO threshold to use when going to sleep in Tx
   uint16_t ratTicksPerBit;             //!<        Number of RAT ticks per bit over the air on 12.4 format
   struct {
      uint16_t freqWordOvr:12;
      uint16_t :2;
      uint16_t bOvrMdmRfCh:1;           //!< \brief If non-zero, use this frequency (MHz) when calculating fractional resampling instead of the
                                        //!<        frequency configured in the command.
      uint16_t bWrMdmRfCh:1;            //!<        If 1, apply freqWordOvr if non-zero also if writing frequency to MDMRFCHANNEL
   } mdmFreqWordOvr;                    //!<        If 1, write frequency (whole MHz only) to MDMRFCHANNEL register when programming frequency
   int16_t mdmRxIntFreq;                //!< \brief Intermediate frequency in Rx mode on 4.12 signed fractional format. Note: This does not
                                        //!<        enforce the IF, but assumes it when programming frequency
   int16_t mdmTxIntFreq;                //!< \brief Intermediate frequency in Tx mode on 4.12 signed fractional format. Note: This  enforces
                                        //!<        the IF only if modIsfCfgVal.bDynamicTxFreq is 1, but it i always assumed when programming
                                        //!<        frequency
   uint8_t mdmPShift;                   //!<        Number of right shifts when calculating P in the fractional resampler
   uint8_t mdmQShift;                   //!<        Number of right shifts when calculating Q in the fractional resampler
   uint16_t txFreqBase;                 //!<        If non-zero, use this frequency to calculate DTX gain and TX IF (if dynamic)
   int16_t freqOffset;                  //!<        Relative frequency/rate offset, signed, scaled by 2^-22
   struct {
      uint32_t preScale:8;
      uint32_t rateWord:21;             //!<        Prescaler value to use in modem (also covers alignvalue and reserved fields)
      uint32_t bFsAdjust:1;             //!<        Rate word in modem prior to compensation (16.5 format)
      uint32_t bSymbolRateAdjust:1;     //!<        If 1, adjust synth frequency by freqOffset when programming synth
      uint32_t bRatAdjust:1;            //!<        If 1, adjust symbol rate by freqOffset when starting Rx or Tx
   } bawConfig;                         //!<        If 1, adjust RAT timing by writing 2^21/(1+freqOffset*2^-22) to RATADJ
};

//! @}

//! \addtogroup synthPar
//! @{
#define _POSITION_synthPar_config                               64
#define _TYPE_synthPar_config                                   uint16_t
#define _BITPOS_synthPar_config_b24MHzXtal                      0
#define _NBITS_synthPar_config_b24MHzXtal                       1
#define _BITPOS_synthPar_config_bSkipTdcCalib                   1
#define _NBITS_synthPar_config_bSkipTdcCalib                    1
#define _BITPOS_synthPar_config_bSkipCoarseCalib                2
#define _NBITS_synthPar_config_bSkipCoarseCalib                 1
#define _BITPOS_synthPar_config_bSkipMidCalib                   3
#define _NBITS_synthPar_config_bSkipMidCalib                    1
#define _BITPOS_synthPar_config_bForceTxMode                    4
#define _NBITS_synthPar_config_bForceTxMode                     1
#define _BITPOS_synthPar_config_bOpenLoop1Pt                    5
#define _NBITS_synthPar_config_bOpenLoop1Pt                     1
#define _BITPOS_synthPar_config_bDisableSynthProg               7
#define _NBITS_synthPar_config_bDisableSynthProg                1
#define _BITPOS_synthPar_config_bDisableSynthEnaPoll            8
#define _NBITS_synthPar_config_bDisableSynthEnaPoll             1
#define _BITPOS_synthPar_config_bPhaseErrorDiscard              9
#define _NBITS_synthPar_config_bPhaseErrorDiscard               1
#define _POSITION_synthPar_ktAvgOffset                          66
#define _TYPE_synthPar_ktAvgOffset                              int8_t
#define _POSITION_synthPar_tdcFactor                            67
#define _TYPE_synthPar_tdcFactor                                uint8_t
#define _POSITION_synthPar_tdcOffset1                           68
#define _TYPE_synthPar_tdcOffset1                               int8_t
#define _POSITION_synthPar_tdcOffset2                           69
#define _TYPE_synthPar_tdcOffset2                               int8_t
#define _POSITION_synthPar_peThreshFactor                       70
#define _TYPE_synthPar_peThreshFactor                           uint8_t
#define _POSITION_synthPar_peThreshOffset                       71
#define _TYPE_synthPar_peThreshOffset                           int8_t
#define _POSITION_synthPar_tdcSubtractFactor                    72
#define _TYPE_synthPar_tdcSubtractFactor                        uint8_t
#define _POSITION_synthPar_tdcSubtractOffset                    73
#define _TYPE_synthPar_tdcSubtractOffset                        int8_t
#define _POSITION_synthPar_defaultPreDivRatio24                 74
#define _TYPE_synthPar_defaultPreDivRatio24                     uint8_t
#define _POSITION_synthPar_fXtalInv                             76
#define _TYPE_synthPar_fXtalInv                                 uint32_t
#define _POSITION_synthPar_K1                                   80
#define _TYPE_synthPar_K1                                       uint32_t
#define _POSITION_synthPar_K2BL16                               84
#define _TYPE_synthPar_K2BL16                                   uint32_t
#define _POSITION_synthPar_K2AL16                               88
#define _TYPE_synthPar_K2AL16                                   uint32_t
#define _POSITION_synthPar_K3BL                                 92
#define _TYPE_synthPar_K3BL                                     uint32_t
#define _POSITION_synthPar_K3AL                                 96
#define _TYPE_synthPar_K3AL                                     uint32_t
#define _POSITION_synthPar_K5                                   100
#define _TYPE_synthPar_K5                                       uint32_t
#define _POSITION_synthPar_M2K                                  104
#define _TYPE_synthPar_M2K                                      uint8_t
#define _POSITION_synthPar_M3K                                  105
#define _TYPE_synthPar_M3K                                      uint8_t
#define _POSITION_synthPar_M4K                                  106
#define _TYPE_synthPar_M4K                                      uint8_t
#define _POSITION_synthPar_synthRegAnaDivLsbFsOnly              107
#define _TYPE_synthPar_synthRegAnaDivLsbFsOnly                  uint8_t
#define _POSITION_synthPar_synthRegAnaDivLsbRx                  108
#define _TYPE_synthPar_synthRegAnaDivLsbRx                      uint8_t
#define _POSITION_synthPar_synthRegAnaDivLsbTx                  109
#define _TYPE_synthPar_synthRegAnaDivLsbTx                      uint8_t
#define _POSITION_synthPar_ktPrecal                             110
#define _TYPE_synthPar_ktPrecal                                 uint8_t
#define _POSITION_synthPar_ktPrecal0                            110
#define _TYPE_synthPar_ktPrecal0                                uint8_t
#define _POSITION_synthPar_ktPrecal1                            111
#define _TYPE_synthPar_ktPrecal1                                uint8_t
#define _POSITION_synthPar_tdcPrecal                            112
#define _TYPE_synthPar_tdcPrecal                                uint16_t
#define _POSITION_synthPar_tdcPrecal0                           112
#define _TYPE_synthPar_tdcPrecal0                               uint16_t
#define _POSITION_synthPar_tdcPrecal1                           114
#define _TYPE_synthPar_tdcPrecal1                               uint16_t
#define _POSITION_synthPar_coarsePrecal                         116
#define _TYPE_synthPar_coarsePrecal                             uint8_t
#define _POSITION_synthPar_coarsePrecal0                        116
#define _TYPE_synthPar_coarsePrecal0                            uint8_t
#define _POSITION_synthPar_coarsePrecal1                        117
#define _TYPE_synthPar_coarsePrecal1                            uint8_t
#define _POSITION_synthPar_coarsePrecal2                        118
#define _TYPE_synthPar_coarsePrecal2                            uint8_t
#define _POSITION_synthPar_coarsePrecal3                        119
#define _TYPE_synthPar_coarsePrecal3                            uint8_t
#define _POSITION_synthPar_midPrecal                            120
#define _TYPE_synthPar_midPrecal                                uint8_t
#define _POSITION_synthPar_midPrecal00                          120
#define _TYPE_synthPar_midPrecal00                              uint8_t
#define _POSITION_synthPar_midPrecal01                          121
#define _TYPE_synthPar_midPrecal01                              uint8_t
#define _POSITION_synthPar_midPrecal02                          122
#define _TYPE_synthPar_midPrecal02                              uint8_t
#define _POSITION_synthPar_midPrecal03                          123
#define _TYPE_synthPar_midPrecal03                              uint8_t
#define _POSITION_synthPar_midPrecal04                          124
#define _TYPE_synthPar_midPrecal04                              uint8_t
#define _POSITION_synthPar_midPrecal05                          125
#define _TYPE_synthPar_midPrecal05                              uint8_t
#define _POSITION_synthPar_midPrecal06                          126
#define _TYPE_synthPar_midPrecal06                              uint8_t
#define _POSITION_synthPar_midPrecal07                          127
#define _TYPE_synthPar_midPrecal07                              uint8_t
#define _POSITION_synthPar_midPrecal08                          128
#define _TYPE_synthPar_midPrecal08                              uint8_t
#define _POSITION_synthPar_midPrecal09                          129
#define _TYPE_synthPar_midPrecal09                              uint8_t
#define _POSITION_synthPar_midPrecal10                          130
#define _TYPE_synthPar_midPrecal10                              uint8_t
#define _POSITION_synthPar_midPrecal11                          131
#define _TYPE_synthPar_midPrecal11                              uint8_t
#define _POSITION_synthPar_midPrecal12                          132
#define _TYPE_synthPar_midPrecal12                              uint8_t
#define _POSITION_synthPar_midPrecal13                          133
#define _TYPE_synthPar_midPrecal13                              uint8_t
#define _POSITION_synthPar_midPrecal14                          134
#define _TYPE_synthPar_midPrecal14                              uint8_t
#define _POSITION_synthPar_midPrecal15                          135
#define _TYPE_synthPar_midPrecal15                              uint8_t
#define _SIZEOF_synthPar                                        136

struct __RFC_STRUCT rfc_synthPar_s {
   uint32_t __dummy0;
   uint32_t __dummy1;
   uint32_t __dummy2;
   uint32_t __dummy3;
   uint32_t __dummy4;
   uint32_t __dummy5;
   uint32_t __dummy6;
   uint32_t __dummy7;
   uint32_t __dummy8;
   uint32_t __dummy9;
   uint32_t __dummy10;
   uint32_t __dummy11;
   uint32_t __dummy12;
   uint32_t __dummy13;
   uint32_t __dummy14;
   uint32_t __dummy15;
   struct {
      uint16_t b24MHzXtal:1;
      uint16_t bSkipTdcCalib:1;         //!< \brief Select 24 MHz crystal <br>
                                        //!<        0: 48 MHz crystal <br>
                                        //!<        1: 24 MHz crystal
      uint16_t bSkipCoarseCalib:1;      //!< \brief Skip TDC calibration <br>
                                        //!<        0: Perform calibration <br>
                                        //!<        1: Skip calibration
      uint16_t bSkipMidCalib:1;         //!< \brief Skip coarse calibration <br>
                                        //!<        0: Perform calibration <br>
                                        //!<        1: Skip calibration
      uint16_t bForceTxMode:1;          //!< \brief Skip mid calibration <br>
                                        //!<        0: Perform calibration <br>
                                        //!<        1: Skip calibration
      uint16_t bOpenLoop1Pt:1;          //!< \brief Force use of Tx mode even for Rx <br>
                                        //!<        0: Automatically switch between Rx and Tx mode<br>
                                        //!<        1: Always use Tx mode
      uint16_t :1;
      uint16_t bDisableSynthProg:1;     //!< \brief Set one-point open loop modulation <br>
                                        //!<        0: Standard modulation <br>
                                        //!<        1: One-point open loop modulation in Tx
      uint16_t bDisableSynthEnaPoll:1;  //!< \brief Disable synth programming<br>
                                        //!<        0: Perform synth programming<br>
                                        //!<        1: Skip synth programming
      uint16_t bPhaseErrorDiscard:1;    //!< \brief Disable polling of ADI during synth power-on<br>
                                        //!<        0: Perform polling<br>
                                        //!<        1: Do not poll
   } config;                            //!< \brief Control phase error discard when synth is in Rx or Tx<br>
                                        //!<        0: No automatic control<br>
                                        //!<        1: Enable phase error discard when synth is in Rx or Tx
   int8_t ktAvgOffset;
   uint8_t tdcFactor;
   int8_t tdcOffset1;
   int8_t tdcOffset2;
   uint8_t peThreshFactor;
   int8_t peThreshOffset;
   uint8_t tdcSubtractFactor;
   int8_t tdcSubtractOffset;
   uint8_t defaultPreDivRatio24;        //!<        Default pre-division ratio, assuming 24 MHz crystal
   uint8_t __dummy16;
   uint32_t fXtalInv;
   uint32_t K1;
   uint32_t K2BL16;                     //!<        Coefficient for loop bandwidth before lock (default: 120 kHz)
   uint32_t K2AL16;                     //!<        Coefficient for loop bandwidth after lock (default: 40 kHz)
   uint32_t K3BL;                       //!<        Coefficient for loop bandwidth before lock (default: 120 kHz)
   uint32_t K3AL;                       //!<        Coefficient for loop bandwidth after lock (default: 40 kHz)
   uint32_t K5;
   uint8_t M2K;
   uint8_t M3K;
   uint8_t M4K;
   uint8_t synthRegAnaDivLsbFsOnly;     //!<        Value for LSB of SYNTHREGANADIV when the synth is running with no Rx or Tx
   uint8_t synthRegAnaDivLsbRx;         //!<        Value for LSB of SYNTHREGANADIV when the synth is outputting to Rx
   uint8_t synthRegAnaDivLsbTx;         //!<        Value for LSB of SYNTHREGANADIV when the synth is outputting to Tx
   uint8_t ktPrecal;                    //!<        Array of 2 precalibrated values for KT calibration
   uint8_t ktPrecal1;                   //!<        KT calibration pre-calibration value for 2420-2460 MHz
   uint16_t tdcPrecal;                  //!<        Array of 2 precalibrated values for TDC calibration
   uint16_t tdcPrecal1;                 //!<        TDC calibration pre-calibration value for 2420-2460 MHz
   uint8_t coarsePrecal;                //!<        Array of 4 precalibrated values for coarse calibration
   uint8_t coarsePrecal1;               //!<        Coarse calibration pre-calibration value for 2420-2440 MHz
   uint8_t coarsePrecal2;               //!<        Coarse calibration pre-calibration value for 2440-2460 MHz
   uint8_t coarsePrecal3;               //!<        Coarse calibration pre-calibration value for 2460-2480 MHz
   uint8_t midPrecal;                   //!<        Array of 16 precalibrated values for mid calibration
   uint8_t midPrecal01;                 //!<        Mid calibration pre-calibration value for 2405-2410 MHz
   uint8_t midPrecal02;                 //!<        Mid calibration pre-calibration value for 2410-2415 MHz
   uint8_t midPrecal03;                 //!<        Mid calibration pre-calibration value for 2415-2420 MHz
   uint8_t midPrecal04;                 //!<        Mid calibration pre-calibration value for 2420-2425 MHz
   uint8_t midPrecal05;                 //!<        Mid calibration pre-calibration value for 2425-2430 MHz
   uint8_t midPrecal06;                 //!<        Mid calibration pre-calibration value for 2430-2435 MHz
   uint8_t midPrecal07;                 //!<        Mid calibration pre-calibration value for 2435-2440 MHz
   uint8_t midPrecal08;                 //!<        Mid calibration pre-calibration value for 2440-2445 MHz
   uint8_t midPrecal09;                 //!<        Mid calibration pre-calibration value for 2445-2450 MHz
   uint8_t midPrecal10;                 //!<        Mid calibration pre-calibration value for 2450-2455 MHz
   uint8_t midPrecal11;                 //!<        Mid calibration pre-calibration value for 2455-2460 MHz
   uint8_t midPrecal12;                 //!<        Mid calibration pre-calibration value for 2460-2465 MHz
   uint8_t midPrecal13;                 //!<        Mid calibration pre-calibration value for 2465-2470 MHz
   uint8_t midPrecal14;                 //!<        Mid calibration pre-calibration value for 2470-2475 MHz
   uint8_t midPrecal15;                 //!<        Mid calibration pre-calibration value for 2475-2480 MHz
};

//! @}

//! \addtogroup frontEndPar
//! @{
#define _POSITION_frontEndPar_lnaIbOffset                       136
#define _TYPE_frontEndPar_lnaIbOffset                           int8_t
#define _POSITION_frontEndPar_ifAmpTrimOffset                   137
#define _TYPE_frontEndPar_ifAmpTrimOffset                       int8_t
#define _POSITION_frontEndPar_rssiOffset                        138
#define _TYPE_frontEndPar_rssiOffset                            int8_t
#define _POSITION_frontEndPar_gpoControl                        139
#define _TYPE_frontEndPar_gpoControl                            uint8_t
#define _BITPOS_frontEndPar_gpoControl_bOutputLnaEn             0
#define _NBITS_frontEndPar_gpoControl_bOutputLnaEn              1
#define _BITPOS_frontEndPar_gpoControl_bOutputPaEn              1
#define _NBITS_frontEndPar_gpoControl_bOutputPaEn               1
#define _BITPOS_frontEndPar_gpoControl_bOutputSynthCal          2
#define _NBITS_frontEndPar_gpoControl_bOutputSynthCal           1
#define _BITPOS_frontEndPar_gpoControl_bOutputSyncFound         3
#define _NBITS_frontEndPar_gpoControl_bOutputSyncFound          1
#define _BITPOS_frontEndPar_gpoControl_rfeGpoCtrlMask           4
#define _NBITS_frontEndPar_gpoControl_rfeGpoCtrlMask            4
#define _SIZEOF_frontEndPar                                     140

struct __RFC_STRUCT rfc_frontEndPar_s {
   uint32_t __dummy0;
   uint32_t __dummy1;
   uint32_t __dummy2;
   uint32_t __dummy3;
   uint32_t __dummy4;
   uint32_t __dummy5;
   uint32_t __dummy6;
   uint32_t __dummy7;
   uint32_t __dummy8;
   uint32_t __dummy9;
   uint32_t __dummy10;
   uint32_t __dummy11;
   uint32_t __dummy12;
   uint32_t __dummy13;
   uint32_t __dummy14;
   uint32_t __dummy15;
   uint32_t __dummy16;
   uint32_t __dummy17;
   uint32_t __dummy18;
   uint32_t __dummy19;
   uint32_t __dummy20;
   uint32_t __dummy21;
   uint32_t __dummy22;
   uint32_t __dummy23;
   uint32_t __dummy24;
   uint32_t __dummy25;
   uint32_t __dummy26;
   uint32_t __dummy27;
   uint32_t __dummy28;
   uint32_t __dummy29;
   uint32_t __dummy30;
   uint32_t __dummy31;
   uint32_t __dummy32;
   uint32_t __dummy33;
   int8_t lnaIbOffset;                  //!<        Signed value to add to the trimmed LNA_IB for the given front-end configuration
   int8_t ifAmpTrimOffset;              //!<        Signed value to add to the trimmed IFAMP_TRIM for the given front-end configuration
   int8_t rssiOffset;                   //!<        Signed value to add to the trimmed RSSI offset for the given front-end configuration
   struct {
      uint8_t bOutputLnaEn:1;
      uint8_t bOutputPaEn:1;            //!<        Output LNA enable on CPEGPO0 if 1
      uint8_t bOutputSynthCal:1;        //!<        Output PA enable signal on CPEGPO1 if 1
      uint8_t bOutputSyncFound:1;       //!<        Output synth calibration signal on CPEGPO2 if 1
      uint8_t rfeGpoCtrlMask:4;         //!<        Output sync found signal on RATGPO1 if 1
   } gpoControl;                        //!<        Bit mask with 1 for the CPEGPOx bits that the RFE is allowed to control
};

//! @}

//! \addtogroup bleRadioPar
//! @{
#define _POSITION_bleRadioPar_advSyncWord                       144
#define _TYPE_bleRadioPar_advSyncWord                           uint32_t
#define _POSITION_bleRadioPar_maxCrcErr                         148
#define _TYPE_bleRadioPar_maxCrcErr                             uint8_t
#define _POSITION_bleRadioPar_numDfBytes                        149
#define _TYPE_bleRadioPar_numDfBytes                            uint8_t
#define _POSITION_bleRadioPar_dataHeaderAnd                     150
#define _TYPE_bleRadioPar_dataHeaderAnd                         uint8_t
#define _POSITION_bleRadioPar_dataAutoEmptyHdr                  151
#define _TYPE_bleRadioPar_dataAutoEmptyHdr                      uint8_t
#define _POSITION_bleRadioPar_advIndHdr                         152
#define _TYPE_bleRadioPar_advIndHdr                             uint8_t
#define _POSITION_bleRadioPar_advDirIndHdr                      153
#define _TYPE_bleRadioPar_advDirIndHdr                          uint8_t
#define _POSITION_bleRadioPar_advNcIndHdr                       154
#define _TYPE_bleRadioPar_advNcIndHdr                           uint8_t
#define _POSITION_bleRadioPar_advScanIndHdr                     155
#define _TYPE_bleRadioPar_advScanIndHdr                         uint8_t
#define _POSITION_bleRadioPar_scanReqHdr                        156
#define _TYPE_bleRadioPar_scanReqHdr                            uint8_t
#define _POSITION_bleRadioPar_connectReqHdr                     157
#define _TYPE_bleRadioPar_connectReqHdr                         uint8_t
#define _POSITION_bleRadioPar_scanRspHdr                        158
#define _TYPE_bleRadioPar_scanRspHdr                            uint8_t
#define _POSITION_bleRadioPar_dataLengthOr                      160
#define _TYPE_bleRadioPar_dataLengthOr                          uint8_t
#define _POSITION_bleRadioPar_advLengthOr                       161
#define _TYPE_bleRadioPar_advLengthOr                           uint8_t
#define _POSITION_bleRadioPar_dataLenMask                       162
#define _TYPE_bleRadioPar_dataLenMask                           uint8_t
#define _POSITION_bleRadioPar_maxDataLen                        163
#define _TYPE_bleRadioPar_maxDataLen                            uint8_t
#define _POSITION_bleRadioPar_advLenMask                        164
#define _TYPE_bleRadioPar_advLenMask                            uint8_t
#define _POSITION_bleRadioPar_maxAdvLen                         165
#define _TYPE_bleRadioPar_maxAdvLen                             uint8_t
#define _POSITION_bleRadioPar_rxIfsTimeout                      166
#define _TYPE_bleRadioPar_rxIfsTimeout                          uint16_t
#define _POSITION_bleRadioPar_minLogUpperLimit                  168
#define _TYPE_bleRadioPar_minLogUpperLimit                      uint8_t
#define _POSITION_bleRadioPar_maxLogUpperLimit                  169
#define _TYPE_bleRadioPar_maxLogUpperLimit                      uint8_t
#define _POSITION_bleRadioPar_winOffsetMargin                   170
#define _TYPE_bleRadioPar_winOffsetMargin                       uint16_t
#define _POSITION_bleRadioPar_extraWinSzLimit                   172
#define _TYPE_bleRadioPar_extraWinSzLimit                       uint16_t
#define _POSITION_bleRadioPar_minWinSz                          174
#define _TYPE_bleRadioPar_minWinSz                              uint8_t
#define _POSITION_bleRadioPar_bWinOffsetNotZero                 175
#define _TYPE_bleRadioPar_bWinOffsetNotZero                     uint8_t
#define _POSITION_bleRadioPar_ticksPerConnIntStep               176
#define _TYPE_bleRadioPar_ticksPerConnIntStep                   uint16_t
#define _SIZEOF_bleRadioPar                                     178

struct __RFC_STRUCT rfc_bleRadioPar_s {
   uint32_t __dummy0;
   uint32_t __dummy1;
   uint32_t __dummy2;
   uint32_t __dummy3;
   uint32_t __dummy4;
   uint32_t __dummy5;
   uint32_t __dummy6;
   uint32_t __dummy7;
   uint32_t __dummy8;
   uint32_t __dummy9;
   uint32_t __dummy10;
   uint32_t __dummy11;
   uint32_t __dummy12;
   uint32_t __dummy13;
   uint32_t __dummy14;
   uint32_t __dummy15;
   uint32_t __dummy16;
   uint32_t __dummy17;
   uint32_t __dummy18;
   uint32_t __dummy19;
   uint32_t __dummy20;
   uint32_t __dummy21;
   uint32_t __dummy22;
   uint32_t __dummy23;
   uint32_t __dummy24;
   uint32_t __dummy25;
   uint32_t __dummy26;
   uint32_t __dummy27;
   uint32_t __dummy28;
   uint32_t __dummy29;
   uint32_t __dummy30;
   uint32_t __dummy31;
   uint32_t __dummy32;
   uint32_t __dummy33;
   uint32_t __dummy34;
   uint32_t __dummy35;
   uint32_t advSyncWord;                //!< \brief Advertising channel sync word. Used as access address for advertiser, scanner and initiator
                                        //!<        operation. 1's complement of the value is used as access address for CMD_BLE_TX_TEST.
   uint8_t maxCrcErr;                   //!< \brief Number of subsequent CRC errors in a master or slave operation that causes the operation to
                                        //!<        end.
   uint8_t numDfBytes;                  //!< \brief Number of bytes in the end of an ADV*_IND packet that are supposed to be anti-whitened. Used
                                        //!<        for locationing
   uint8_t dataHeaderAnd;               //!<        Value to be AND'ed with the header of a packet sent by a master or sleve operation
   uint8_t dataAutoEmptyHdr;            //!< \brief Value to use for the header of an auto-empty packet, prior to automatic NESN, SN, and MD
                                        //!<        insertion
   uint8_t advIndHdr;                   //!<        Header value of an ADV_IND message prior to automatic address type bit insertion
   uint8_t advDirIndHdr;                //!<        Header value of an ADV_DIRECT_IND message prior to automatic address type bit insertion
   uint8_t advNcIndHdr;                 //!<        Header value of an ADV_NONCONN_IND message prior to automatic address type bit insertion
   uint8_t advScanIndHdr;               //!<        Header value of an ADV_SCAN_IND message prior to automatic address type bit insertion
   uint8_t scanReqHdr;                  //!<        Header value of a SCAN_REQ message prior to automatic address type bit insertion
   uint8_t connectReqHdr;               //!<        Header value of a CONNECT_REQ message prior to automatic address type bit insertion
   uint8_t scanRspHdr;                  //!<        Header value of a SCAN_RSP message prior to automatic address type bit insertion
   uint8_t __dummy36;
   uint8_t dataLengthOr;                //!<        Value to be OR'ed with the length byte of a data channel packet prior to transmission
   uint8_t advLengthOr;                 //!<        Value to be OR'ed with the length byte of an advertising channel packet prior to transmission
   uint8_t dataLenMask;                 //!< \brief Value to be AND'ed with the received length byte of a data channel packet prior to
                                        //!<        interpretation
   uint8_t maxDataLen;                  //!<        Maximum allowed length of a received data channel packet; if larger, sync search is resumed
   uint8_t advLenMask;                  //!< \brief Value to be AND'ed with the received length byte of an advertising channel packet prior to
                                        //!<        interpretation
   uint8_t maxAdvLen;                   //!< \brief Maximum allowed length of a received advertising channel packet; if larger, sync search is
                                        //!<        resumed
   uint16_t rxIfsTimeout;               //!<        Number of RAT cycles to listen for sync after T_IFS before giving up
   uint8_t minLogUpperLimit;            //!<        Minumum value of logUpperLimit
   uint8_t maxLogUpperLimit;            //!<        Maximum value of logUpperLimit
   uint16_t winOffsetMargin;            //!< \brief Number of RAT cycles from the end of the transmitted CONNECT_REQ packet to the nominal first
                                        //!<        possible start of the first master packet. This number includes the 1.25 ms specified in the
                                        //!<        BLE spec between the CONNECT_REQ message and the transmit window
   uint16_t extraWinSzLimit;            //!< \brief Minimum number of RAT cycles from the the nominal first possible start of the first master
                                        //!<        packet to the configured transmit time before the reported WinSize is incremented by one.
   uint8_t minWinSz;                    //!< \brief Minimum reported WinSize; the transmitted WinSize in a CONNECT_REQ message will be this
                                        //!<        value or one more
   uint8_t bWinOffsetNotZero;           //!< \brief If this value is zero, WinOffset will be in the range [0, connectInterval-1]. If the value is
                                        //!<        non-zero, WinOffset will be in the range [1, connectInterval].
   uint16_t ticksPerConnIntStep;        //!<        Number of RAT ticks per connection interval step as given in the CONNECT_REQ message
};

//! @}

//! \addtogroup ieeeRadioPar
//! @{
#define _POSITION_ieeeRadioPar_numPreambleNibbles               144
#define _TYPE_ieeeRadioPar_numPreambleNibbles                   uint8_t
#define _POSITION_ieeeRadioPar_phyLenMask                       145
#define _TYPE_ieeeRadioPar_phyLenMask                           uint8_t
#define _POSITION_ieeeRadioPar_preambleWord                     146
#define _TYPE_ieeeRadioPar_preambleWord                         uint16_t
#define _POSITION_ieeeRadioPar_ackMacHdr                        148
#define _TYPE_ieeeRadioPar_ackMacHdr                            uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAnd                     150
#define _TYPE_ieeeRadioPar_frameCtrlAnd                         uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndBeacon               150
#define _TYPE_ieeeRadioPar_frameCtrlAndBeacon                   uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndData                 152
#define _TYPE_ieeeRadioPar_frameCtrlAndData                     uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndAck                  154
#define _TYPE_ieeeRadioPar_frameCtrlAndAck                      uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndMacCmd               156
#define _TYPE_ieeeRadioPar_frameCtrlAndMacCmd                   uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndFt4                  158
#define _TYPE_ieeeRadioPar_frameCtrlAndFt4                      uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndFt5                  160
#define _TYPE_ieeeRadioPar_frameCtrlAndFt5                      uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndFt6                  162
#define _TYPE_ieeeRadioPar_frameCtrlAndFt6                      uint16_t
#define _POSITION_ieeeRadioPar_frameCtrlAndFt7                  164
#define _TYPE_ieeeRadioPar_frameCtrlAndFt7                      uint16_t
#define _POSITION_ieeeRadioPar_rxFrameEndOffset                 166
#define _TYPE_ieeeRadioPar_rxFrameEndOffset                     int16_t
#define _POSITION_ieeeRadioPar_txFrameEndOffset                 168
#define _TYPE_ieeeRadioPar_txFrameEndOffset                     int16_t
#define _POSITION_ieeeRadioPar_backoffPeriod                    170
#define _TYPE_ieeeRadioPar_backoffPeriod                        uint16_t
#define _POSITION_ieeeRadioPar_csmaSlottedWaitBeforeCca         172
#define _TYPE_ieeeRadioPar_csmaSlottedWaitBeforeCca             uint16_t
#define _POSITION_ieeeRadioPar_csmaUnslottedWaitBeforeCca       174
#define _TYPE_ieeeRadioPar_csmaUnslottedWaitBeforeCca           uint16_t
#define _POSITION_ieeeRadioPar_csmaUnslottedWaitAfterCca        176
#define _TYPE_ieeeRadioPar_csmaUnslottedWaitAfterCca            uint16_t
#define _POSITION_ieeeRadioPar_csmaUnslottedWait0Cca            178
#define _TYPE_ieeeRadioPar_csmaUnslottedWait0Cca                uint16_t
#define _POSITION_ieeeRadioPar_csmaRxWakeupTime                 180
#define _TYPE_ieeeRadioPar_csmaRxWakeupTime                     uint16_t
#define _POSITION_ieeeRadioPar_csmaRxWakeupMargin               182
#define _TYPE_ieeeRadioPar_csmaRxWakeupMargin                   uint16_t
#define _POSITION_ieeeRadioPar_ccaCorrWindow                    184
#define _TYPE_ieeeRadioPar_ccaCorrWindow                        uint16_t
#define _POSITION_ieeeRadioPar_rxCcaCorrValidTime               186
#define _TYPE_ieeeRadioPar_rxCcaCorrValidTime                   uint16_t
#define _POSITION_ieeeRadioPar_broadcastAckMode                 188
#define _TYPE_ieeeRadioPar_broadcastAckMode                     uint8_t
#define _SIZEOF_ieeeRadioPar                                    189

struct __RFC_STRUCT rfc_ieeeRadioPar_s {
   uint32_t __dummy0;
   uint32_t __dummy1;
   uint32_t __dummy2;
   uint32_t __dummy3;
   uint32_t __dummy4;
   uint32_t __dummy5;
   uint32_t __dummy6;
   uint32_t __dummy7;
   uint32_t __dummy8;
   uint32_t __dummy9;
   uint32_t __dummy10;
   uint32_t __dummy11;
   uint32_t __dummy12;
   uint32_t __dummy13;
   uint32_t __dummy14;
   uint32_t __dummy15;
   uint32_t __dummy16;
   uint32_t __dummy17;
   uint32_t __dummy18;
   uint32_t __dummy19;
   uint32_t __dummy20;
   uint32_t __dummy21;
   uint32_t __dummy22;
   uint32_t __dummy23;
   uint32_t __dummy24;
   uint32_t __dummy25;
   uint32_t __dummy26;
   uint32_t __dummy27;
   uint32_t __dummy28;
   uint32_t __dummy29;
   uint32_t __dummy30;
   uint32_t __dummy31;
   uint32_t __dummy32;
   uint32_t __dummy33;
   uint32_t __dummy34;
   uint32_t __dummy35;
   uint8_t numPreambleNibbles;          //!<        Number of zero symbols (nibbles) transmitted as preamble
   uint8_t phyLenMask;                  //!<        Value to be AND'ed with the received PHY header byte before evaluating frame length
   uint16_t preambleWord;               //!<        16-bit value of transmitted preamble - repeated (does not change receiver)
   uint16_t ackMacHdr;                  //!<        MAC header FCF to use when sending auto ACK
   uint16_t frameCtrlAnd;               //!< \brief Array of 8 half-words to AND with the FCF of the MAC header prior to interpretation,
                                        //!<        depending on frame type
   uint16_t frameCtrlAndData;           //!<        Value to AND with the FCF of the MAC header prior to interpretation for data frame
   uint16_t frameCtrlAndAck;            //!<        Value to AND with the FCF of the MAC header prior to interpretation for ACK frame
   uint16_t frameCtrlAndMacCmd;         //!<        Value to AND with the FCF of the MAC header prior to interpretation for MAC CMD frame
   uint16_t frameCtrlAndFt4;            //!<        Value to AND with the FCF of the MAC header prior to interpretation for frame type 4
   uint16_t frameCtrlAndFt5;            //!<        Value to AND with the FCF of the MAC header prior to interpretation for frame type 5
   uint16_t frameCtrlAndFt6;            //!<        Value to AND with the FCF of the MAC header prior to interpretation for frame type 6
   uint16_t frameCtrlAndFt7;            //!<        Value to AND with the FCF of the MAC header prior to interpretation for frame type 7
   int16_t rxFrameEndOffset;            //!<        Number of RAT cycles to add to calculated end of received frame for use with CCA
   int16_t txFrameEndOffset;            //!<        Number of RAT cycles to add to calculated end of transmitted frame
   uint16_t backoffPeriod;              //!<        Number of RAT cycles in a backoff period
   uint16_t csmaSlottedWaitBeforeCca;   //!<        Number of RAT cycles to wait before reading CCA state in slotted CSMA-CA
   uint16_t csmaUnslottedWaitBeforeCca; //!<        Number of RAT cycles to wait before reading CCA state in unslotted CSMA-CA
   uint16_t csmaUnslottedWaitAfterCca;  //!<        Number of RAT cycles to wait after reading CCA state in unslotted CSMA-CA
   uint16_t csmaUnslottedWait0Cca;      //!<        Number of extra RAT cycles to wait if 0 wait periods was drawn in CSMA-CA
   uint16_t csmaRxWakeupTime;           //!< \brief Time needed to start Rx not included synth calibration, used to find time to restart Rx
                                        //!<        when suspending Rx during CSMA-CA
   uint16_t csmaRxWakeupMargin;         //!<        Margin used to evaluate if Rx can be turned off when suspending Rx during CSMA-CA
   uint16_t ccaCorrWindow;              //!<        Size of window used to look for correlation peaks in CCA
   uint16_t rxCcaCorrValidTime;         //!<        Time from start of demodulator before correlation for CCA is valid
   uint8_t broadcastAckMode;            //!< \brief Handling of ACK of broadcast frames<br>
                                        //!<        0: Never ACK frames if destination PAN ID is 0xFFFF and destination address is short,
                                        //!<        or if short destination address is 0xFFFF<br>
                                        //!<        1: Never ACK frames if destination PAN ID is 0xFFFF or if short destination address is 0xFFFF<br>
                                        //!<        2: Never ACK frames if short destination address is 0xFFFF<br>
                                        //!<        3: ACK frames regardless of broadcast PAN ID or address (as CC253x)
};

//! @}

//! \addtogroup propRadioPar
//! @{
#define _POSITION_propRadioPar_syncWord                         144
#define _TYPE_propRadioPar_syncWord                             uint32_t
#define _POSITION_propRadioPar_crcInit1                         148
#define _TYPE_propRadioPar_crcInit1                             uint32_t
#define _POSITION_propRadioPar_crcXor1                          152
#define _TYPE_propRadioPar_crcXor1                              uint32_t
#define _POSITION_propRadioPar_crcPoly1                         156
#define _TYPE_propRadioPar_crcPoly1                             uint32_t
#define _POSITION_propRadioPar_numCrcBits1                      164
#define _TYPE_propRadioPar_numCrcBits1                          uint8_t
#define _POSITION_propRadioPar_modeSwitchBitpos                 165
#define _TYPE_propRadioPar_modeSwitchBitpos                     uint8_t
#define _POSITION_propRadioPar_fcsTypeBitpos                    166
#define _TYPE_propRadioPar_fcsTypeBitpos                        uint8_t
#define _POSITION_propRadioPar_dwBitpos                         167
#define _TYPE_propRadioPar_dwBitpos                             uint8_t
#define _POSITION_propRadioPar_crcMinLen                        168
#define _TYPE_propRadioPar_crcMinLen                            uint8_t
#define _BITPOS_propRadioPar_crcMinLen_minLen0                  0
#define _NBITS_propRadioPar_crcMinLen_minLen0                   4
#define _BITPOS_propRadioPar_crcMinLen_minLen1                  4
#define _NBITS_propRadioPar_crcMinLen_minLen1                   4
#define _POSITION_propRadioPar_addressMask                      172
#define _TYPE_propRadioPar_addressMask                          uint32_t
#define _SIZEOF_propRadioPar                                    176

struct __RFC_STRUCT rfc_propRadioPar_s {
   uint32_t __dummy0;
   uint32_t __dummy1;
   uint32_t __dummy2;
   uint32_t __dummy3;
   uint32_t __dummy4;
   uint32_t __dummy5;
   uint32_t __dummy6;
   uint32_t __dummy7;
   uint32_t __dummy8;
   uint32_t __dummy9;
   uint32_t __dummy10;
   uint32_t __dummy11;
   uint32_t __dummy12;
   uint32_t __dummy13;
   uint32_t __dummy14;
   uint32_t __dummy15;
   uint32_t __dummy16;
   uint32_t __dummy17;
   uint32_t __dummy18;
   uint32_t __dummy19;
   uint32_t __dummy20;
   uint32_t __dummy21;
   uint32_t __dummy22;
   uint32_t __dummy23;
   uint32_t __dummy24;
   uint32_t __dummy25;
   uint32_t __dummy26;
   uint32_t __dummy27;
   uint32_t __dummy28;
   uint32_t __dummy29;
   uint32_t __dummy30;
   uint32_t __dummy31;
   uint32_t __dummy32;
   uint32_t __dummy33;
   uint32_t __dummy34;
   uint32_t __dummy35;
   uint32_t syncWord;                   //!<        Sync word to use for modes where it is not otherwise configured
   uint32_t crcInit1;                   //!< \brief Initialization value for the CRC used when the CRC select bit is 1, written to the shift
                                        //!<        register prior to transmission or reception
   uint32_t crcXor1;                    //!< \brief Value to be XOR-ed with the CRC after it has been calculated, before it is transmitted in
                                        //!<        Tx, and after it has been received and before it is checked on Rx
   uint32_t crcPoly1;                   //!<        Polynomial to substitute when CRC select bit is 1
   uint32_t __dummy36;
   uint8_t numCrcBits1;                 //!<        Number of CRC bits when the CRC select bit is 1
   uint8_t modeSwitchBitpos;            //!< \brief Position (counted from LSB as transmitted) of mode switch bit of header.
                                        //!<        0xFF: No 15.4g specific support. 0xFE: No mode switch bit
   uint8_t fcsTypeBitpos;               //!<        Position (counted from LSB as transmitted) of FCS type bit of header. 0xFF: No FCS type bit
   uint8_t dwBitpos;                    //!< \brief Position (counted from LSB as transmitted) of data whitening bit of header. 0xFF: No DW bit.
                                        //!<        0x80-0x9F: Invert polarity
   struct {
      uint8_t minLen0:4;
      uint8_t minLen1:4;                //!<        Minimum length of payload for calculating CRC of type 0; pad if smaller
   } crcMinLen;                         //!<        Minimum length of payload for calculating CRC of type 1; pad if smaller
   uint8_t __dummy37;
   uint16_t __dummy38;
   uint32_t addressMask;                //!< \brief Mask to apply (logical AND) to addresses in CMD_PROP_RX and CMD_PROP_RX_ADV before address
                                        //!<        filtering. For addresses with more than 32 bits, the mask is applied to the least significant
                                        //!<        word, and the most significant word of the mask is all 1's
};

//! @}

//! @}
//! @}
#endif
