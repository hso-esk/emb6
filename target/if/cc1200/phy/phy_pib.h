#ifndef __PHY_PIB_H__
#define __PHY_PIB_H__
#ifndef __DECL_PHY_PIB_H__
#define __DECL_PHY_PIB_H__    extern
#endif /* #ifndef __DECL_PHY_PIB_H__ */

/*============================================================================*/
/**
 * \file    phy_pib.h
 *
 * \author  Manuel Schappacher
 *
 * \brief   Physical layer PIB implementation.
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/

#include <stdint.h>
#include <stdbool.h>

#if !defined(MAC_TDD_ENABLED)
#include "rf.h"
#endif /* MAC_TDD_ENABLED */

/** \addtogroup phy IEEE802.15.4 PHY
 *  @{
 */

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** \brief  Maximum number of channels per page */
#define PHY_MAX_CHANNELS_PER_PAGE       1

/** \brief Maximum size of a packet */
#define PHY_MAX_PACKET_SIZE             127

/** \brief Default configured transmit power */
#define PHY_PIB_TXPOWER_DEFAULT         0



/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/** \addtogroup phy_pib PHY Information Base (PIB)
 *  @{
 */

/**
 * \brief PHY Channel pages.
 */
typedef enum E_PHY_CHPAGE_T
{
  /** Testpage */
  E_PHY_CHPAGE_TEST,
  /** 868MHZ@50KBPS */
  E_PHY_CHPAGE_868MHZ_50KBPS,
  /** 434MHZ@50KBPS */
  E_PHY_CHPAGE_434MHZ_50KBPS,
  /** 434MHZ@20KBPS */
  E_PHY_CHPAGE_434MHZ_20KBPS,

  E_PHY_CHPAGE_MAX

} e_phy_chpage_t;

/**
 * \brief PHY TX power tolerance.
 */
typedef enum E_PHY_TXPOW_TOLERANCE_T
{
  /** 1dBm */
  E_PHY_TXPOW_TOLERANCE_1DB,
  /** 3db */
  E_PHY_TXPOW_TOLERANCE_3DB,
  /** 3db */
  E_PHY_TXPOW_TOLERANCE_6DB,

  E_PHY_TXPOW_TOLERANCE_MAX

} e_phy_txpow_tolerance_t;

/**
 * \brief PHY CCA modes.
 */
typedef enum E_PHY_CCA_MODE_T
{
  /** energy above threshold */
  E_PHY_CCA_MODE_ED_THRESHOLD,
  /** carrier sense only */
  E_PHY_CCA_MODE_CARRIER_SENSE,
  /** carrier sense or energy above threshols */
  E_PHY_CCA_MODE_CARRIER_OR_ED,
  /** aloha (always true) */
  E_PHY_CCA_MODE_ALOHA,
  /** LBT */
  E_PHY_CCA_MODE_LBT,

  E_PHY_CCA_MODE_MAX

} e_phy_cca_mode_t;

/**
 * \brief PHY Sniff modes.
 */
typedef enum E_PHY_PREAMBLE_LEN_T
{
  /** 31 bits */
  E_PHY_PREAMBLE_LEN_31 = 0,
  /** 127 bits */
  E_PHY_PREAMBLE_LEN_127,
  /** 191 bits */
  E_PHY_PREAMBLE_LEN_191,

  E_PHY_PREAMBLE_LEN_MAX,

} e_phy_preamble_len_t;

/**
 * \brief PHY Sniff modes.
 */
typedef enum E_PHY_SNIFFMODE_T
{
  /** No sniffmode, normal operation */
  E_PHY_SNIFFMODE_OFF,
  /** Sniffmode with carrier sensing */
  E_PHY_SNIFFMODE_CS

} e_phy_sniffmode_t;

/**
 * \brief PHY information base attributes.
 */
typedef enum E_PHY_PIB_ATTR_T
{

  E_PHY_PIB_phyCurrentChannel,
//  E_PHY_PIB_phyChannelsSupported,
  E_PHY_PIB_phyTXPowerTolerance,
  E_PHY_PIB_phyTXPower,
  E_PHY_PIB_phyCCAMode,
  E_PHY_PIB_phyCurrentPage,
//  E_PHY_PIB_phyMaxFrameDuration,
//  E_PHY_PIB_phySHRDuration,
//  E_PHY_PIB_phySymbolsPerOctet,
//  E_PHY_PIB_phySymbolsPerOctet,
  E_PHY_PIB_phyPreambleSymbolLength,
//  E_PHY_PIB_phyUWBDataRatesSupported,
//  E_PHY_PIB_phyCSSLowDataRateSupported,
//  E_PHY_PIB_phyUWBCoUSupported,
//  E_PHY_PIB_phyUWBCSSupported,
//  E_PHY_PIB_phyUWBLCPSupported,
//  E_PHY_PIB_phyUWBCurrentPulseShape,
//  E_PHY_PIB_phyUWBCoUpulse,
//  E_PHY_PIB_phyUWBCSpulse,
//  E_PHY_PIB_phyUWBLCPWeight1,
//  E_PHY_PIB_phyUWBLCPWeight2,
//  E_PHY_PIB_phyUWBLCPWeight3,
//  E_PHY_PIB_phyUWBLCPWeight4,
//  E_PHY_PIB_phyUWBLCPDelay2,
//  E_PHY_PIB_phyUWBLCPDelay3,
//  E_PHY_PIB_phyUWBLCPDelay4,
//  E_PHY_PIB_phyRanging,
//  E_PHY_PIB_phyRangingCrystalOffset,
//  E_PHY_PIB_phyRangingDPS,
//  E_PHY_PIB_phyCurrentCode,
//  E_PHY_PIB_phyNativePRF,
//  E_PHY_PIB_phyUWBScanBinsPerChannel,
//  E_PHY_PIB_phyUWBInsertedPreambleInterval,
//  E_PHY_PIB_phyTXRMARKEROffset,
//  E_PHY_PIB_phyRXRMARKEROffset,
//  E_PHY_PIB_phyRFRAMEProcessingTime,
//  E_PHY_PIB_phyCCADuration,
//
//  E_PHY_PIB_phyReserved0,
//  E_PHY_PIB_phyReserved1,
//  E_PHY_PIB_phyReserved2,
//  E_PHY_PIB_phyReserved3,
//  E_PHY_PIB_phyReserved4,
//
  E_PHY_PIB_phySniffMode,

  E_PHY_PIB_MAX

} e_phy_pib_attr_t;
/** @}*/


/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/

/** \addtogroup phy_pib PHY Information Base (PIB)
 *  @{
 */
/**
 * \brief PHY information base.
 */
typedef struct S_PHY_PIB_T
{
  /** The RF channel to use for all following transmissions and receptions, 8.1.2. */
  uint8_t phyCurrentChannel;
  /** Each entry in the list consists of a channel page and a list of channel
   * numbers supported for that channel page. */
//  phyChannelsSupported;
  /** The tolerance on the transmit power setting, plus or
   * minus the indicated value. */
  e_phy_txpow_tolerance_t phyTXPowerTolerance;
  /** The transmit power of the device in dBm. */
  int8_t phyTXPower;
  /** The CCA mode, as defined in 8.2.7. */
  e_phy_cca_mode_t phyCCAMode;
  /** This is the current PHY channel page. This is used in conjunction with
      phyCurrentChannel to uniquely identify the channel currently
      being used.*/
  e_phy_chpage_t phyCurrentPage;
  /** The maximum number ofsymbols in a frame, as defined in 9.4. */
//  phyMaxFrameDuration;
  /** The duration of the synchronization header (SHR) in symbols for
      the current PHY. */
//  phySHRDuration;
  /** The number of symbols per octet for the current PHY. For the UWB PHY this
      is defined in 14.2.3. For the CSS PHY, 1.3 corresponds to 1 Mb/s while
      5.3 corresponds to 250 kb/s. */
//  phySymbolsPerOctet;

  /** Zero indicates preamble symbol length is 31,  one indicates that
     length is 127 two indicates that length 191 symbol is used. Present for
     UWB PHY and Low-Power extension. */
 e_phy_preamble_len_t phyPreambleSymbolLength;
  /** A list of the data rates available in the operating channel as defined
      in Table 105. */
//  phyUWBDataRatesSupported;
  /** A value of TRUE indicates that 250 kb/s is supported. Present for CSS PHY. */
//  phyCSSLowDataRateSupported;
  /** TRUE if CoU pulses are supported, FALSE otherwise. */
//  phyUWBCoUSupported;
 /** TRUE if CSpulses are supported, FALSE otherwise. */
//  phyUWBCSSupported;
  /** TRUE if LCP pulses are supported, FALSE otherwise. */
//  phyUWBLCPSupported;
  /** Indicates the current pulse shape setting of the UWB PHY. The mandatory
      pulse is described in 14.4.5. Optional pulse shapes include CoU, as
      defined in 14.5.1, CS, as defined in 14.5.2, and LCP, as
      defined in 14.5.3. */
//  phyUWBCurrentPulseShape;
 /** Defines the slope of the frequency chirp and bandwidth of pulse.
     CCh.3–CCh.6 are valid only for wideband UWB channels, e.g., 4, 7, 11, or 15,
     as defined in 14.5.1. */
//  phyUWBCoUpulse;
  /** Defines the group delay of the continuous spectrum filter. No.3–No.6 are
      valid only for wideband UWB channels, e.g., 4, 7, 11, or 15, as described
      in 14.5.2. */
//  phyUWBCSpulse;
  /** The weights are represented in twos-complement form. A value of
      0x80 represents –1 while a value of 0x7F represents 1. */
//  phyUWBLCPWeight1;
  /** The weights are represented in twos-complement form. A value of
      0x80 represents –1 while a value of 0x7F represents 1. */
//  phyUWBLCPWeight2;
  /** The weights are represented in twos-complement form. A value of
      0x80 represents –1 while a value of 0x7F represents 1. */
//  phyUWBLCPWeight3;
  /** The weights are represented in twos-complement form. A value of
      0x80 represents –1 while a value of 0x7F represents 1. */
//  phyUWBLCPWeight4;
  /** The range is from 0 to 4 ns with a resolution is 4/255 = 15.625 ps. For
      example, a value of 0x00 represents 0 while 0x02 represents 31.25 ps, as
      defined in 14.5.3. */
//  phyUWBLCPDelay2;
  /** The range is from 0 to 4 ns with a resolution is 4/255 = 15.625 ps. For
      example, a value of 0x00 represents 0 while 0x02 represents 31.25 ps, as
      defined in 14.5.3. */
//  phyUWBLCPDelay3;
  /** The range is from 0 to 4 ns with a resolution is 4/255 = 15.625 ps. For
      example, a value of 0x00 represents 0 while 0x02 represents 31.25 ps, as
      defined in 14.5.3. */
//  phyUWBLCPDelay4;
  /** TRUE if ranging is supported, FALSE otherwise. */
//  phyRanging;
  /** TRUE if crystal offset characterization is supported, FALSE otherwise. */
//  phyRangingCrystalOffset;
  /** TRUE if DPS is supported, FALSE otherwise. */
//  phyRangingDPS;
  /** This value is zero for PHYs other than UWB or CSS. For UWB PHYs, this
      represents the current preamble code index in use by the transmitter, as
      defined in Table 102 and Table 103. For the CSS
      PHY, the value indicates the subchirp, as defined
      in 13.3. */
//  phyCurrentCode;
  /** For UWB PHYs, the native PRF. Zero is for nonUWB PHYs; one is for PRF of
      4; two is for a PRF of 16; and three is for PHYs that have no preference. */
//  phyNativePRF;
  /** Number of frequency intervals used to scan each UWB channel
      (scan resolution). Set to zero for non-UWB PHYs.  */
//  phyUWBScanBinsPerChannel;
  /** The time interval between two neighboring inserted preamble symbols in
      the data portion, as defined in 14.6, for UWB PHYs operating with
      CCA mode 6. The resolution is a data symbol duration at a data rate of
      850 kb/s for all channels. Set to four for UWB PHY in CCA mode 6;
      otherwise, set to zero. */
//  phyUWBInsertedPreambleInterval;
  /** A count of the propagation time from the ranging counter to the transmit
      antenna. The LSB of a time value represents 1/128 of a chip time at the
      mandatory chipping rate of 499.2 MHz. */
//  phyTXRMARKEROffset;
  /** A count of the propagation time from the receive antenna to the ranging
      counter. The LSB of a time value represents 1/128 of a chip time at the
      mandatory chipping rate of 499.2 MHz. */
//  phyRXRMARKEROffset;
  /** A count of the processing time required by the PHY to handle an arriving
      RFRAME. The LSB represents 2 ms. The meaning of the value is that if a
      sequence of RFRAMEs arrive separated by phyRFRAMEProcessingTime, then the
      PHY can keep up with the processing indefinitely. */
//  phyRFRAMEProcessingTime;
  /** The duration for CCA, specified in symbols. This attribute shall only be
      implemented with PHYs operating in the 950 MHz band. */
//  phyCCADuration;

  /** RESERVED */
//  phyReserved0;
  /** RESERVED */
//  phyReserved1;
  /** RESERVED */
//  phyReserved2;
  /** RESERVED */
//  phyReserved3;
  /** RESERVED */
//  phyReserved4;

  /** Sniffmode */
  e_phy_sniffmode_t phySniffMode; /* sniff mode */

} s_phy_pib_t;

/**
 * \brief Table element of the PIB table.
 */
typedef struct S_PHY_PIB_TABLE_T
{
  /** offset of the element in the PIB */
  uint8_t uc_offset;
  /** length of the value */
  uint8_t uc_len;
  /** maximum value */
  int16_t i_min;
  /** minimum value */
  int16_t i_max;
  /* read only attribute */
  bool b_rOnly;

} s_phy_pib_table_t;
/** @}*/


/** \addtogroup phy_pcb PHY Constants Base (PCB)
 *  @{
 */
/**
 * \brief Structure of PHY constants.
 */
typedef struct S_PHY_CONSTS_T
{
  /** The maximum PSDU size (in octets) the PHY shall be able to receive. */
  uint8_t aMaxPHYPacketSize;

} s_phy_consts_t;
/** @}*/


/*============================================================================*/
/*                                CONSTANTS                                   */
/*============================================================================*/
#if !defined(MAC_TDD_ENABLED)

/** \addtogroup phy_pcb PHY Constants Base (PCB)
 *  @{
 */
/** The PHY constants */
__DECL_PHY_PIB_H__ const s_phy_consts_t gcs_phy_consts;
/** @}*/

/** \addtogroup phy_pib PHY Information Base (PIB)
 *  @{
 */
/** PIB default values. */
__DECL_PHY_PIB_H__ const s_phy_pib_t gcs_phy_pibDefaults;

/** The PHY channel pages including channels. */
__DECL_PHY_PIB_H__ const e_rf_channel_t gcpe_phy_channels[E_PHY_CHPAGE_MAX][PHY_MAX_CHANNELS_PER_PAGE];

/** Tx power tolerance map */
__DECL_PHY_PIB_H__ const uint8_t gcpuc_phy_txtolerance_table[E_PHY_TXPOW_TOLERANCE_MAX];

/** CCA map */
__DECL_PHY_PIB_H__ const e_rf_cca_mode_t gcpe_phy_cca_table[E_PHY_CCA_MODE_MAX];

/** Preamble map */
__DECL_PHY_PIB_H__ const e_rf_preamble_t gcpe_phy_preamble_table[E_PHY_PREAMBLE_LEN_MAX];
/** @}*/

#endif /* MAC_TDD_ENABLED */

/*============================================================================*/
/*                            VARIABLES                                       */
/*============================================================================*/

/** \addtogroup phy_pib PHY Information Base (PIB)
 *  @{
 */
/** The PHY information base table. */
__DECL_PHY_PIB_H__ s_phy_pib_table_t gps_phy_pibTable[];

/** The PHY Information base. */
__DECL_PHY_PIB_H__ s_phy_pib_t gs_phy_pib;
/** @}*/

/** @}*/
#endif /* #ifndef __PHY_PIB_H__ */


