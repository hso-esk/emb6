/*
 * hal_cc1200_io.c
 *
 *  Created on: Jan 23, 2014
 *      Author: Fesseha
 */

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include "bsp_lums.h"
#include "hal_cc1200_io.h"
#include "hal_types.h"
#include "io.h"
#include "spi.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/
/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDY      (uint8_t)( 0x80 )

#define RADIO_BURST_ACCESS   (uint8_t)( 0x40 )
#define RADIO_SINGLE_ACCESS  (uint8_t)( 0x00 )
#define RADIO_READ_ACCESS    (uint8_t)( 0x80 )
#define RADIO_WRITE_ACCESS   (uint8_t)( 0x00 )


/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/
/*=============================================================================
 * cc1200_spiReadReg()
 *============================================================================*/
trxStatus_t cc1200_spiReadReg( uint16_t ui_addr, uint8_t *pc_data,
    uint8_t uc_len )
{
  uint8_t uc_chipStatus = 0U;
  uint8_t uc_tempExt = (uint8_t) (ui_addr >> 8);
  uint8_t uc_tempAddr = (uint8_t) (ui_addr & 0x00FF);

  /* Checking if this is a FIFO access -> returns chip not ready  */
  if ((CC1200_SINGLE_TXFIFO <= uc_tempAddr) && (uc_tempExt == 0))
    return STATUS_CHIP_RDY;

  /* Decide what register space is accessed */
  if (!uc_tempExt)
  {
    uc_chipStatus = cc1200_spi8bitRegAccess((RADIO_BURST_ACCESS | RADIO_READ_ACCESS),
        uc_tempAddr, pc_data, uc_len);
  } else if (uc_tempExt == 0x2F)				// Extended address space
  {
    uc_chipStatus = cc1200_spi16BitRegAccess((RADIO_BURST_ACCESS | RADIO_READ_ACCESS),
        uc_tempExt, uc_tempAddr, pc_data, uc_len);
  }

  return (uc_chipStatus);
}

/*=============================================================================
 * cc1200_spiWriteReg()
 *============================================================================*/
trxStatus_t cc1200_spiWriteReg( uint16_t ui_addr, uint8_t *pc_data,
    uint8_t uc_len )
{
  uint8_t uc_chipStatus;
  uint8_t uc_tempExt = (uint8_t) (ui_addr >> 8);
  uint8_t uc_tempAddr = (uint8_t) (ui_addr & 0x00FF);

  /* Checking if this is a FIFO access - returns chip not ready */
  if ((CC1200_SINGLE_TXFIFO <= uc_tempAddr) && (uc_tempExt == 0))
    return STATUS_CHIP_RDY;

  /* Decide what register space is accessed */
  if (!uc_tempExt)
  {
    uc_chipStatus = cc1200_spi8bitRegAccess((RADIO_BURST_ACCESS | RADIO_WRITE_ACCESS),
        uc_tempAddr, pc_data, uc_len);
  } else if (uc_tempExt == 0x2F)
  {
    uc_chipStatus = cc1200_spi16BitRegAccess((RADIO_BURST_ACCESS | RADIO_WRITE_ACCESS),
        uc_tempExt, uc_tempAddr, pc_data, uc_len);
  }
  return (uc_chipStatus);
}

/*=============================================================================
 * cc1200_spiWriteTxFifo()
 *============================================================================*/
trxStatus_t cc1200_spiWriteTxFifo( uint8_t *pc_writeData, uint8_t uc_len )
{
  uint8_t uc_chipStatus;
    uc_chipStatus = cc1200_spi8bitRegAccess(0x00, CC1200_BURST_TXFIFO, pc_writeData,
      uc_len);
  return (uc_chipStatus);
}

/*=============================================================================
 * cc1200_spiReadRxFifo()
 *============================================================================*/
trxStatus_t cc1200_spiReadRxFifo( uint8_t *pc_readData, uint8_t uc_len )
{
  uint8_t uc_chipStatus;
    uc_chipStatus = cc1200_spi8bitRegAccess(0x00, CC1200_BURST_RXFIFO, pc_readData,
      uc_len);
  return (uc_chipStatus);
}

/*=============================================================================
 * cc1200_getTxStatus()
 *============================================================================*/
trxStatus_t cc1200_getTxStatus( void )
{
  /* Send SNOP and receive CS. */
  return cc1200_spiCmdStrobe(CC1200_SNOP);
}

/*=============================================================================
 * cc1200_getRxStatus()
 *============================================================================*/
trxStatus_t cc1200_getRxStatus( void )
{
  return cc1200_spiCmdStrobe(CC1200_SNOP | RADIO_READ_ACCESS);
}

/*=============================================================================
 * cc1200_enterIdle()
 *============================================================================*/
void cc1200_enterIdle( void )
{
  /* Strobe to idle state. */
  cc1200_spiCmdStrobe(CC1200_SIDLE);
  /* Wait until CHIP_RDYn goes low and chip in IDLE state. */
  while (cc1200_spiCmdStrobe(CC1200_SNOP) & 0xF0)
    ;
  /* Flush the Receive FIFO */
  cc1200_spiCmdStrobe(CC1200_SFRX);
  /* Flush the Transmit FIFO */
  cc1200_spiCmdStrobe(CC1200_SFTX);
}

/*=============================================================================
 * cc1200_enterWor()
 *============================================================================*/
void cc1200_enterWor( void )
{
  cc1200_enterIdle();
  cc1200_spiCmdStrobe(CC1200_SWOR);
}

/*=============================================================================
 * cc1200_enterSleep()
 *============================================================================*/
void cc1200_enterSleep( void )
{
  cc1200_enterIdle();
  cc1200_spiCmdStrobe(CC1200_SPWD);
}

/*=============================================================================
 * cc1200_enterRx()
 *============================================================================*/
void cc1200_enterRx( void )
{
  cc1200_enterIdle();
  cc1200_spiCmdStrobe(CC1200_SRX);
}

/*=============================================================================
 * cc1200_manualCalibration()
 *============================================================================*/
void cc1200_manualCalibration( void )
{
  uint8_t uc_original_fs_cal2;
  uint8_t uc_calResults_for_vcdac_start_high[3];
  uint8_t uc_calResults_for_vcdac_start_mid[3];
  uint8_t uc_marcState;
  uint8_t uc_writeByte;

  /* 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00) */
  uc_writeByte = 0x00;
  cc1200_spiWriteReg(CC1200_FS_VCO2, &uc_writeByte, 1);

  /* 2) Start with high VCDAC (original VCDAC_START + 2) */
  cc1200_spiReadReg(CC1200_FS_CAL2, &uc_original_fs_cal2, 1);
  uc_writeByte = uc_original_fs_cal2 + VCDAC_START_OFFSET;
  cc1200_spiWriteReg(CC1200_FS_CAL2, &uc_writeByte, 1);

  /* 3) Calibrate and wait for calibration to be done (radio back in IDLE state) */
  cc1200_spiCmdStrobe(CC1200_SCAL);

  do
  {
    cc1200_spiReadReg(CC1200_MARCSTATE, &uc_marcState, 1);
  } while (uc_marcState != 0x41);

  /* 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value */
  cc1200_spiReadReg(CC1200_FS_VCO2,
      &uc_calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
  cc1200_spiReadReg(CC1200_FS_VCO4,
      &uc_calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
  cc1200_spiReadReg(CC1200_FS_CHP,
      &uc_calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

  /* 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00) */
  uc_writeByte = 0x00;
  cc1200_spiWriteReg(CC1200_FS_VCO2, &uc_writeByte, 1);

  /* 6) Continue with mid VCDAC (original VCDAC_START) */
  uc_writeByte = uc_original_fs_cal2;
  cc1200_spiWriteReg(CC1200_FS_CAL2, &uc_writeByte, 1);

  /* 7) Calibrate and wait for calibration to be done (radio back in IDLE state) */
  cc1200_spiCmdStrobe(CC1200_SCAL);

  do
  {
    cc1200_spiReadReg(CC1200_MARCSTATE, &uc_marcState, 1);
  } while (uc_marcState != 0x41);

  /* 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value */
  cc1200_spiReadReg(CC1200_FS_VCO2,
      &uc_calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
  cc1200_spiReadReg(CC1200_FS_VCO4,
      &uc_calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
  cc1200_spiReadReg(CC1200_FS_CHP,
      &uc_calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

  /* 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result */
  if (uc_calResults_for_vcdac_start_high[FS_VCO2_INDEX]
      > uc_calResults_for_vcdac_start_mid[FS_VCO2_INDEX])
  {
    uc_writeByte = uc_calResults_for_vcdac_start_high[FS_VCO2_INDEX];
    cc1200_spiWriteReg(CC1200_FS_VCO2, &uc_writeByte, 1);
    uc_writeByte = uc_calResults_for_vcdac_start_high[FS_VCO4_INDEX];
    cc1200_spiWriteReg(CC1200_FS_VCO4, &uc_writeByte, 1);
    uc_writeByte = uc_calResults_for_vcdac_start_high[FS_CHP_INDEX];
    cc1200_spiWriteReg(CC1200_FS_CHP, &uc_writeByte, 1);
  } else
  {
    uc_writeByte = uc_calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
    cc1200_spiWriteReg(CC1200_FS_VCO2, &uc_writeByte, 1);
    uc_writeByte = uc_calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
    cc1200_spiWriteReg(CC1200_FS_VCO4, &uc_writeByte, 1);
    uc_writeByte = uc_calResults_for_vcdac_start_mid[FS_CHP_INDEX];
    cc1200_spiWriteReg(CC1200_FS_CHP, &uc_writeByte, 1);
  }

}

/*=============================================================================
 * cc1200_calibrateRCOsc()
 *============================================================================*/
void cc1200_calibrateRCOsc( void )
{
  uint8_t uc_writeByte;

  cc1200_spiReadReg(CC1200_WOR_CFG0, &uc_writeByte, 1);
  /* Mask RC_MODE (2:1) bitfields and write new values to enable RC caliberation. */
  uc_writeByte = (uc_writeByte & 0xF9) | (0x02 << 1);
  cc1200_spiWriteReg(CC1200_WOR_CFG0, &uc_writeByte, 1);

  /* Strobe idle to calibrate RCOsc */
  cc1200_spiCmdStrobe(CC1200_SIDLE);
  /* Disable RC calibration */
  uc_writeByte = (uc_writeByte & 0xF9) | (0x00 << 1);
  cc1200_spiWriteReg(CC1200_WOR_CFG0, &uc_writeByte, 1);
}


/**
 *
 * @param uc_cmd
 * @return
 */
uint8_t cc1200_spiCmdStrobe(uint8_t cmd)
{
    uint8_t chip_status;


    hal_spiSelect(NULL);
    hal_spiTxRx(&cmd, &chip_status, 1);
    hal_spiDeSelect(NULL);
    return chip_status;
}


/**
 *
 * @param uc_accessType
 * @param uc_addressByte
 * @param puc_data
 * @param uc_len
 * @return
 */
uint8_t cc1200_spi8bitRegAccess(uint8_t     access,
                                uint8_t     addr,
                                uint8_t    *p_data,
                                uint8_t     len)
{
    uint8_t chip_status;
    uint8_t data;


    hal_spiSelect(NULL);
    {
        /* Transmit access command */
        data = access | addr;
        hal_spiTxRx(&data, &chip_status, 1);

        /* Read/Write data */
        if (data & RADIO_READ_ACCESS) {
            if (data & RADIO_BURST_ACCESS) {        /* Burst  Read Access   */
                hal_spiRead(p_data, len);
            } else {                                /* Single Read Access   */
                hal_spiRead(p_data, 1);
            }
        } else {
            if (data & RADIO_BURST_ACCESS) {        /* Burst Write Access   */
                hal_spiWrite(p_data, len);
            } else {                                /* Single Write Access  */
                hal_spiWrite(p_data, 1);
            }
        }
    }
    hal_spiDeSelect(NULL);

    return (chip_status);
}


/**
 *
 * @param uc_accessType
 * @param uc_extendedAddr
 * @param uc_regAddr
 * @param puc_data
 * @param uc_len
 * @return
 */
uint8_t cc1200_spi16BitRegAccess(uint8_t    access,
                                 uint8_t    ext_addr,
                                 uint8_t    reg_addr,
                                 uint8_t   *p_data,
                                 uint8_t    len)
{
    uint8_t chip_status;
    uint8_t data;


    hal_spiSelect(NULL);
    {
        /* Transmit access command */
        data = access | ext_addr;
        hal_spiTxRx(&data, &chip_status, 1);
        hal_spiTxRx(&reg_addr, &chip_status, 1);

        /* Read/Write data */
        if (data & RADIO_READ_ACCESS) {
            if (data & RADIO_BURST_ACCESS) {        /* Burst  Read Access   */
                hal_spiRead(p_data, len);
            } else {                                /* Single Read Access   */
                hal_spiRead(p_data, 1);
            }
        } else {
            if (data & RADIO_BURST_ACCESS) {        /* Burst Write Access   */
                hal_spiWrite(p_data, len);
            } else {                                /* Single Write Access  */
                hal_spiWrite(p_data, 1);
            }
        }
    }
    hal_spiDeSelect(NULL);

    return chip_status;
}
