

//*********************************************************************************
// These settings have been generated for use with TI-RTOS and cc13xxware
//
// Generated by SmartRF Studio version 2.4.2 (build #23)
// Tested for TI-RTOS version tirtos_simplelink_2_18_00
// Device: CC1310 Rev. 2.1
// 
//*********************************************************************************


//*********************************************************************************
// Parameter summary
// Address: aa-bb 
// Frequency: 869.52499 MHz
// Data Format: Serial mode disable 
// Deviation: 25.000 kHz
// Packet Length Config: Variable 
// Max Packet Length: 125 
// Packet Length: 30 
// RX Filter BW: 155 kHz
// Symbol Rate: 50.00000 kBaud
// Sync Word Length: 16 Bits 
// TX Power: 14 dBm (requires define CCFG_FORCE_VDDR_HH = 1 in ccfg.c, see CC13xx/CC26xx Technical Reference Manual)
// Whitening: No whitening 

#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_prop_cmd.h>
#include "sf_cc13xx_869_525.h"

// Overrides for CMD_PROP_RADIO_DIV_SETUP
uint32_t RF_869_525_pOverrides[] =
{
    // override_use_patch_prop_genfsk.xml
    // PHY: Use MCE ROM bank 4, RFE RAM patch
    MCE_RFE_OVERRIDE(0,4,0,1,0,0),
    // override_synth_prop_863_930_div5.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x0A480583,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x7AB80603,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout
    (uint32_t)0x04B00243,
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 3
    (uint32_t)0x00038883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x1a.xml
    // Tx: Enable PA ramping (0x41). Rx: Set AGC reference level to 0x1A.
    HW_REG_OVERRIDE(0x6088,0x411A),
    // Tx: Configure PA ramping setting
    HW_REG_OVERRIDE(0x608C,0x8213),
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_frontend_xd.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +5 dB
    (uint32_t)0x00FB88A3,
    // TX power override
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
    (uint32_t)0xFFFFFFFF,
};

// CMD_PROP_RADIO_DIV_SETUP
rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_869_525_cmdPropRadioDivSetup =
{
    .commandNo = 0x3807,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x1,
    .modulation.deviation = 0x64,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x8000,
    .rxBw = 0x26,
    .preamConf.nPreamBytes = 0x2,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x10,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x1,
    .formatConf.fecMode = 0x0,
    .formatConf.whitenMode = 0x0,
    .config.frontEndMode = 0x0,
    .config.biasMode = 0x1,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0xA73F,
    .pRegOverride = RF_869_525_pOverrides,
    .centerFreq = 0x0365,
    .intFreq = 0x8000,
    .loDivider = 0x05,
};

// CMD_FS
rfc_CMD_FS_t RF_869_525_cmdFs =
{
    .commandNo = 0x0803,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .frequency = 0x0365,
    .fractFreq = 0x8666,
    .synthConf.bTxMode = 0x0,
    .synthConf.refFreq = 0x0,
    .__dummy0 = 0x00,
    .__dummy1 = 0x00,
    .__dummy2 = 0x00,
    .__dummy3 = 0x0000,
};

// CMD_PROP_TX_ADV
rfc_CMD_PROP_TX_ADV_t RF_869_525_cmdPropTxAdv =
{
    .commandNo = 0x3803,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bUseCrc = 0x0,
    .pktConf.bCrcIncSw = 0x0,
    .pktConf.bCrcIncHdr = 0x0,
    .numHdrBits = 0x00,
    .pktLen = 0x0000,
    .startConf.bExtTxTrig = 0x0,
    .startConf.inputMode = 0x0,
    .startConf.source = 0x0,
    .preTrigger.triggerType = 0x0,
    .preTrigger.bEnaCmd = 0x0,
    .preTrigger.triggerNo = 0x0,
    .preTrigger.pastTrig = 0x0,
    .preTime = 0x00000000,
    .syncWord = 0x543D54CD,
    .pPkt = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};

// CMD_PROP_RX_ADV
rfc_CMD_PROP_RX_ADV_t RF_869_525_cmdPropRxAdv =
{
    .commandNo = 0x3804,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bRepeatOk = 0x0,
    .pktConf.bRepeatNok = 0x0,
    .pktConf.bUseCrc = 0x0,
    .pktConf.bCrcIncSw = 0x0,
    .pktConf.bCrcIncHdr = 0x0,
    .pktConf.endType = 0x0,
    .pktConf.filterOp = 0x0,
    .rxConf.bAutoFlushIgnored = 0x0,
    .rxConf.bAutoFlushCrcErr = 0x0,
    .rxConf.bIncludeHdr = 0x0,
    .rxConf.bIncludeCrc = 0x0,
    .rxConf.bAppendRssi = 0x0,
    .rxConf.bAppendTimestamp = 0x0,
    .rxConf.bAppendStatus = 0x0,
    .syncWord0 = 0x0000543D,
    .syncWord1 = 0x00000000,
    .maxPktLen = 0x0000,
    .hdrConf.numHdrBits = 0x0,
    .hdrConf.lenPos = 0x0,
    .hdrConf.numLenBits = 0x0,
    .addrConf.addrType = 0x0,
    .addrConf.addrSize = 0x0,
    .addrConf.addrPos = 0x0,
    .addrConf.numAddr = 0x0,
    .lenOffset = 0x00,
    .endTrigger.triggerType = 0x0,
    .endTrigger.bEnaCmd = 0x0,
    .endTrigger.triggerNo = 0x0,
    .endTrigger.pastTrig = 0x0,
    .endTime = 0x00000000,
    .pAddr = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .pQueue = 0, // INSERT APPLICABLE POINTER: (dataQueue_t*)&xxx
    .pOutput = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};


