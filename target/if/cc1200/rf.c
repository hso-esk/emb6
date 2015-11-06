/*============================================================================*/
/**
 * \file    rf.c
 *
 * \author  Manuel Schappacher
 *
 * \brief   Low Level RF-Chip access.
 *
 *          This module provides low level access to the RF Chip.
 */
/*============================================================================*/

#ifdef __cplusplus
extern "C"
{
#endif

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define __DECL_RF_H__
#include "rf.h"
#include "rf_cfg.h"
#include "hal_cc1200_io.h"
#include "bsp_lums.h"

#include "lib_port.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Size of the RX FIFO */
#define RF_FIFO_LEN                   128
/** RX FIFO threshold */
#define RF_RX_FIFO_THR                50
/** RX FIFO threshold */
#define RF_TX_FIFO_THR                50
/** RF busy watchdog */
#define RF_BUSY_WD_MAX                1000

/** value to check for chip compatibility */
#define RF_CHIP_COMPATIBILITY         (0x20)
/** RX FIFO error in marc status */
#define RF_MARC_STATUS_RXFIFOERR      (0x11)
/** TX FIFO error in marc status */
#define RF_MARC_STATUS_TXFIFOERR      (0x0B)

/** MASK of the CCA in the register */
#define RF_PKT_CFG2_CCA_MSK           (0x1C)

/** CCA fail bit in the MARC status register */
#define RF_PKT_MARC_STATUS0_CCA_FAIL  (0x04)

/** RSSI valid bit */
#define RF_RSSI_VALID_MSK             (0x01)
/** Offset of the lower RSSI bits */
#define RF_RSSI_LOW_OFST              (3)
/** Length of the lower RSSI bits */
#define RF_RSSI_LOW_LEN               (4)
/** Mask of the lower RSSI bits */
#define RF_RSSI_LOW_MSK               (0x78)
/** RSSI resolution */
#define RF_RSSI_RES                   (0.0625)
/** RSSI offset */
#define RF_RSSI_OFFSET                (102)
/** number of maximum CCA attempts */
#define RF_CCA_ATTEMPTS_MAX           (3)

#ifndef RF_CCA_ON_TX
#define RF_CCA_ON_TX                  TRUE
#endif /* #ifndef RF_CCA_ON_TX */

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/

/* Read the chip status */
#define RF_STATUS()               cc1200_spiCmdStrobe( CC1200_SNOP )

/** Write register in the RF Chip */
#define RF_REGWR( addr, data )    {uint8_t wr = (data); cc1200_spiWriteReg( (addr), &wr, 1);}
/** Read register from the RF Chip */
#define RF_REGRD( addr, data )    cc1200_spiReadReg( (addr), &(data), 1)

/** Flush the RX FIFO */
#define RF_RXFIFOFLUSH()          cc1200_spiCmdStrobe( CC1200_SFRX )
/** Read from the RX FIFO */
#define RF_RXFIFORD( data, len)   cc1200_spiReadRxFifo((data), (len))

/** Flush the TX FIFO */
#define RF_TXFIFOFLUSH()          cc1200_spiCmdStrobe( CC1200_SFTX )
/** Write to the TX FIFO */
#define RF_TXFIFOWR( data, len)   cc1200_spiWriteTxFifo((data), (len))
/** Start transmission */
#define RF_TXSTART()              cc1200_spiCmdStrobe( CC1200_STX )

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 *  Available RF states.
 */
typedef enum E_RF_STATE_T
{
  /* uninitialized */
  E_RF_STATE_NINIT = 0,
  /* chip off */
  E_RF_STATE_OFF,
  /* Idle state */
  E_RF_STATE_IDLE,
  /* RX state */
  E_RF_STATE_RX,
  /* TX start state */
  E_RF_STATE_TXSTART,
  /* TX busy state */
  E_RF_STATE_TXBUSY,
  /* TX busy state */
  E_RF_STATE_TXFIN,
  /* RX busy state */
  E_RF_STATE_RXBUSY,
  /* RX busy state */
  E_RF_STATE_RXFIN

} e_rf_state_t;


/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/


/**
 * Context of a RX operation.
 */
typedef struct S_RF_RX_CTX_T
{
  /* buffer for the Tx data */
  uint8_t puc_buf[RF_MAX_PKT_LEN];
  /* length of the received data */
  uint16_t ui_len;
  /* number of bytes remaining */
  uint16_t ui_rem;
  /* RSSI */
  uint8_t uc_rssi;
  /* Status */
  uint8_t uc_status;


} s_rf_rx_ctx_t;

/**
 * Context of a TX operation.
 */
typedef struct S_RF_TX_CTX_T
{
  /* buffer for the Tx data */
  uint8_t puc_buf[RF_MAX_PKT_LEN];
  /* length of the data to transmit */
  uint16_t ui_len;
  /* number of bytes remaining */
  uint16_t ui_rem;

  /* CCA Done */
  uint8_t uc_ccaDone;

} s_rf_tx_ctx_t;

/**
 * RF Context
 */
typedef struct S_RF_CTX_T
{
  /* state of the RF module */
  e_rf_state_t e_state;
  /* mode of the RF module */
  e_rf_mode_t e_mode;
  /* channel of the RF module */
  e_rf_channel_t e_ch;
  /* CCA mode of the RF module */
  e_rf_cca_mode_t e_cca;
  /* RX context */
  s_rf_rx_ctx_t s_rx_ctx;
  /* TX context */
  s_rf_tx_ctx_t s_tx_ctx;

  /* data handlers */
  s_rf_handler_t s_hndlr;

  /* Busy WD */
  uint32_t ul_busyWD;

} s_rf_ctx_t;

/*============================================================================*/
/*                                  CONSTANTS                                 */
/*============================================================================*/

/**
 *  Tx preamble configurations for the different channels.
 */
static const uint8_t gcpuc_rf_premble_cfg[E_RF_CHANNEL_MAX][E_RF_PREAMBLE_MAX] = {

    { 0x18, 0x2C, 0x30 }, /* 4B, 12B 24B, E_RF_CHANNEL_TEST  */
    { 0x18, 0x2C, 0x30 }, /* 4B, 12B 24B, E_RF_CHANNEL_868MHz@50kbps */
    { 0x18, 0x2C, 0x30 }, /* 4B, 12B 24B, E_RF_CHANNEL_434MHz@50kbps */
    { 0x18, 0x2C, 0x30 }, /* 4B, 12B 24B, E_RF_CHANNEL_434MHz@20kbps */
};


/**
 *  CCA register configurations.
 */
static const uint8_t gcpuc_rf_cca_cfg[E_RF_CCA_MODE_MAX] = {
    0x04, /* E_RF_CCA_MODE_ED_THRESHOLD */
    0x08, /* E_RF_CCA_MODE_CARRIER_SENSE */
    0x0C, /* E_RF_CCA_MODE_CARRIER_OR_ED */
    0x00, /* E_RF_CCA_MODE_ALOHA */
    0x10  /* E_RF_CCA_MODE_CARRIER_LBT */
};


/*============================================================================*/
/*                                LOCAL VARIABLES                             */
/*============================================================================*/

/* local RF context */
static s_rf_ctx_t gs_rf_ctx = {
    .e_state = E_RF_STATE_NINIT,          /* uninitialized */
    .e_mode = E_RF_MODE_TRXOFF,           /* TRX off by default */
    .e_ch = E_RF_CHANNEL_TEST,            /* test channel */
    .e_cca = E_RF_CCA_MODE_ED_THRESHOLD,  /* ED threshold per default */

    .s_hndlr = {
        .pf_rx = NULL,
    }
};


/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/* write a configuration */
static int _rf_cfgWr( const s_rf_register_t* ps_cfg, uint16_t ui_num );

/* calibrate RF */
static int _rf_calib( void );

/* reset RF */
static int _rf_reset( void );

/* reset RF */
static int _rf_setMode( e_rf_mode_t e_mode );

/* set RF to sleep mode */
static int _rf_gotoSleep( void );

/* set RF to TRx Off mode */
static int _rf_gotoTrxOff( void );

/* set RF to idle mode */
static int _rf_gotoIdle( void );

/* set RF to idle mode */
static int _rf_gotoRx( void );

/* set RF to idle mode */
static int _rf_gotoWor( void );

/* restore mode */
static void _rf_restoreMode( void );

/* receive data */
static int _rf_rx( void );

/* transmit data */
static int _rf_tx( void );

/* set Tx power */
static int _rf_setTxPower( int8_t c_power );

/* Run CCA */
static uint8_t _rf_ccaRun( void );

/* Run ED */
static int16_t _rf_edRun( void );

/* Read RSSI */
static int16_t _rf_readRSSI( void );

/*============================================================================*/
/*                                ISR PROTOTYPES                              */
/*============================================================================*/

/* Rx FIFO value has reached upper threshold */
static void _rf_isrRxFifoAbThr( void *p_arg);

/* Rx Started */
static void _rf_isrRxStart( void *p_arg);

/* Rx finished */
static void _rf_isrRxFin( void *p_arg);

/* Tx FIFO value has reached lower threshold */
static void _rf_isrTxFifoBelThr( void *p_arg);

/* Tx finished */
static void _rf_isrTxFin( void *p_arg);

/* CCA Done */
static void _rf_isrCCADone( void *p_arg);

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/*============================================================================*/
/**
 * \brief   Write a configuration set to the RF Chip.
 *
 *          This function writes the given configuration to the RF chip
 *          and performs a validation of the written values.
 *
 * \param   ps_cfg    Configuration to write.
 * \param   ui_num    Number of configuration values to write.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_cfgWr( const s_rf_register_t* ps_cfg, uint16_t ui_num )
{
  int i_ret = 0;
  int i = 0;

  if( ps_cfg == NULL )
    i_ret = -1;
  else
  {
    for( i = 0; i < ui_num; i++ )
    {
      uint8_t uc_verify = 0;

      /* write the configuration */
      RF_REGWR( ps_cfg[i].ui_addr, ps_cfg[i].uc_data );

      /* read written value and check against the configuration */
      RF_REGRD( ps_cfg[i].ui_addr, uc_verify );
      if( uc_verify != ps_cfg[i].uc_data )
      {
        /* write error */
        i_ret = -1;
        break;
      }
    }
  }

  return i_ret;
}

/*============================================================================*/
/**
 * \brief   Calibrate the RF Module.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_calib( void )
{
  int i_ret = 0;

  /* run calibration */
  cc1200_manualCalibration();

  return i_ret;
}

/*============================================================================*/
/**
 * \brief   Reset the RF Module.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_reset( void )
{
  int i_ret = 0;

  /* reset the RF chip */
  cc1200_spiCmdStrobe( CC1200_SRES );
  while( 0 != (cc1200_spiCmdStrobe( CC1200_SNOP ) & CC1200_STATE_CHIP_RDYn) );

  return i_ret;
}


/*============================================================================*/
/**
 * \brief   Set RF mode.
 *
 * \param   e_mode    Mode to set.
 */
/*============================================================================*/
int _rf_setMode( e_rf_mode_t e_mode )
{
  int i_ret = -1;

  if( gs_rf_ctx.e_state == E_RF_STATE_NINIT )
  {
    /* the RF module has't been initialized */
    i_ret = -1;
  }
  else
  {
    switch( e_mode )
    {
      /*
       * In TRXOFF Mode all RF based operations will be disabled. Neither
       * transmitting nor receiving will be possible.
       */
      case E_RF_MODE_TRXOFF:
      {
        /* set state */
        gs_rf_ctx.e_state = E_RF_STATE_OFF;

        /* disable radio */
        _rf_gotoTrxOff();
        break;
      }

      /*
       * This is the normal operation mode. Within this mode the RF chip is
       * in RX mode unless a packet shall be transmitted.
      */
      case E_RF_MODE_NORMAL:
      {
        /* write configuration to the chip */
        if( _rf_cfgWr( gcs_rf_cfg_def, (sizeof(gcs_rf_cfg_def)/sizeof(s_rf_register_t)) ) == -1 )
          /* configuration has not been written to the RF chip */
          i_ret = -1;
        else
        {
          /* set Mode */
          gs_rf_ctx.e_state = E_RF_STATE_RX;

          /* re-calibrate RF chip */
          _rf_calib();

          /* set chip to RX state */
          _rf_gotoRx();
          i_ret = 0;
        }
        break;
      }

      /*
       * Power saving Mode.
       */
      case E_RF_MODE_PWRSAVE:
      {
        /* write configuration to the chip */
        if( _rf_cfgWr( gcs_rf_cfg_def, (sizeof(gcs_rf_cfg_def)/sizeof(s_rf_register_t)) ) == -1 )
          /* configuration has not been written to the RF chip */
          i_ret = -1;
        else
        {
          /* set state */;
          gs_rf_ctx.e_state = E_RF_STATE_IDLE;

          /* re-calibrate RF chip */
          _rf_calib();

          /* set chip to WOR state */
          _rf_gotoWor();
          i_ret = 0;
        }
        break;
      }
    }
  }

  if( i_ret == -1 )
  {
    /* An error occurred. Reset RF and disable radio */
    _rf_gotoSleep();
    _rf_gotoSleep();

    gs_rf_ctx.e_mode = E_RF_MODE_TRXOFF;
    gs_rf_ctx.e_state = E_RF_STATE_OFF;
  }

  return i_ret;

}


/*============================================================================*/
/**
 * \brief   Set the RF Module to sleep mode.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_gotoSleep( void )
{
  int i_ret = 0;

  /* go to sleep mode */
  cc1200_enterSleep();

  return i_ret;
}

/*============================================================================*/
/**
 * \brief   Set the RF Module to TRx Off mode.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_gotoTrxOff( void )
{
  int i_ret = 0;

  /* go to sleep mode */
  cc1200_enterSleep();

  return i_ret;
}


/*============================================================================*/
/**
 * \brief   Set the RF Module to Idle mode.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_gotoIdle( void )
{
  int i_ret = 0;

  /* disable IRQs */
  bsp_extIntDisable( E_TARGET_EXT_INT_0 );
  bsp_extIntDisable( E_TARGET_EXT_INT_1 );
  bsp_extIntDisable( E_TARGET_EXT_INT_2 );

  /* go to idle state */
  cc1200_enterIdle();
  while( CC1200_STATE_IDLE == (cc1200_spiCmdStrobe( CC1200_SNOP ) ) );

  return i_ret;
}


/*============================================================================*/
/**
 * \brief   Set the RF Module to Rx Mode.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_gotoRx( void )
{
  int i_ret = 0;

  /* go idle first */
  _rf_gotoIdle();

    /* write configuration to the chip */
  if( _rf_cfgWr( gcs_rf_cfg_rx, (sizeof(gcs_rf_cfg_rx)/sizeof(s_rf_register_t)) ) == -1 )
    /* configuration has not been written to the RF chip */
    i_ret = -1;
  else
  {
    {
      /* read CCA settings */
      uint8_t uc_reg = 0;
      RF_REGRD( CC1200_PKT_CFG2, uc_reg );

      /* apply configured CCA settings */
      uc_reg &= ~RF_PKT_CFG2_CCA_MSK;
      uc_reg |= gcpuc_rf_cca_cfg[gs_rf_ctx.e_cca];
      RF_REGWR( CC1200_PKT_CFG2, uc_reg );
    }

    /* Flush FIFO set the first RXFIFO Threshold to 1*/
    RF_RXFIFOFLUSH();
    RF_REGWR( CC1200_FIFO_CFG, 1 );

    /* go to RX state */
    cc1200_enterRx();
    /* wait for chip to enter RX state */
    while( 0 == (cc1200_spiCmdStrobe( CC1200_SNOP ) & CC1200_STATE_RX) )
      cc1200_enterRx();

    /* connect and enable RX interrupts */
    bsp_extIntEnable( E_TARGET_EXT_INT_0, E_TARGET_INT_EDGE_RISING, &_rf_isrRxFifoAbThr);
    bsp_extIntEnable( E_TARGET_EXT_INT_1, E_TARGET_INT_EDGE_RISING, &_rf_isrRxStart);
    bsp_extIntEnable( E_TARGET_EXT_INT_2, E_TARGET_INT_EDGE_FALLING, &_rf_isrRxFin);

    /* clear pending flags */
    bsp_extIntClear( E_TARGET_EXT_INT_0 );
    bsp_extIntClear( E_TARGET_EXT_INT_1 );
    bsp_extIntClear( E_TARGET_EXT_INT_2 );
  }

  return i_ret;
}

/*============================================================================*/
/**
 * \brief   Set the RF Module to WOR Mode.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_gotoWor( void )
{
  int i_ret = 0;

  /* go idle first */
  _rf_gotoIdle();


  /* write configuration to the chip */
  if( _rf_cfgWr( gcs_rf_cfg_ewor, (sizeof(gcs_rf_cfg_ewor)/sizeof(s_rf_register_t)) ) == -1 )
    /* configuration has not been written to the RF chip */
    i_ret = -1;
  else
  {
    /* Flush FIFO set the first RXFIFO Threshold to 1*/
    RF_RXFIFOFLUSH();
    RF_REGWR( CC1200_FIFO_CFG, 1 );

    /* goto WOR state */
    cc1200_enterWor();

    /* Connect ISRs */
    bsp_extIntEnable( E_TARGET_EXT_INT_0, E_TARGET_INT_EDGE_RISING, &_rf_isrRxFifoAbThr);
    bsp_extIntEnable( E_TARGET_EXT_INT_1, E_TARGET_INT_EDGE_RISING, &_rf_isrRxStart);
    bsp_extIntEnable( E_TARGET_EXT_INT_2, E_TARGET_INT_EDGE_FALLING, &_rf_isrRxFin);

    /* clear pending flags */
    bsp_extIntClear( E_TARGET_EXT_INT_0 );
    bsp_extIntClear( E_TARGET_EXT_INT_1 );
    bsp_extIntClear( E_TARGET_EXT_INT_2 );
  }
  return i_ret;
}


/*============================================================================*/
/**
 * \brief   Restore configured mode.
 *
 */
/*============================================================================*/
static void _rf_restoreMode( void )
{
  /* switch RF back to configured mode */
  if( gs_rf_ctx.e_mode == E_RF_MODE_TRXOFF )
  {
    /* TRx Off Configured. */
    gs_rf_ctx.e_state = E_RF_STATE_OFF;
    _rf_gotoTrxOff();
  }
  else if( gs_rf_ctx.e_mode == E_RF_MODE_PWRSAVE )
  {
    gs_rf_ctx.e_state = E_RF_STATE_IDLE;
    _rf_gotoWor();
  }
  else
  {
    /* Normal mode configured. Switch back to RX mode */
    gs_rf_ctx.e_state = E_RF_STATE_RX;
    _rf_gotoRx();
  }
}

/*============================================================================*/
/**
 * \brief   Set the RF Module to Rx Mode.
 *
 * \return  number of received bytes on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_rx( void )
{
  uint16_t i_ret = -1;
  uint16_t ui_ftr = RF_RX_FIFO_THR;

  /* if this was the first read the length of the rx
   * procedure has to be set. */
  if( 0 == gs_rf_ctx.s_rx_ctx.ui_len )
  {
    uint8_t uc_len = 0;

    /* read single byte from FIFO */
    RF_RXFIFORD( &uc_len, 1 );

    /* set the length to the first byte read */
    gs_rf_ctx.s_rx_ctx.ui_len = uc_len + 2;
    gs_rf_ctx.s_rx_ctx.ui_rem = gs_rf_ctx.s_rx_ctx.ui_len;

    /* decrease fifo reads */
    ui_ftr--;
    i_ret = 1;

    /* update FIFO threshold */
    RF_REGWR( CC1200_FIFO_CFG, RF_RX_FIFO_THR );
  }
  else
  {
    if( gs_rf_ctx.s_rx_ctx.ui_rem < ui_ftr )
      ui_ftr = gs_rf_ctx.s_rx_ctx.ui_rem;

    if( ui_ftr )
    {
      int16_t i_pos = gs_rf_ctx.s_rx_ctx.ui_len - gs_rf_ctx.s_rx_ctx.ui_rem;

      if( i_pos < 0 )
        /* an error occured */
        i_ret = -1;
      else
      {
        /* read remaining bytes from FIFO and update context*/
        RF_RXFIFORD( &gs_rf_ctx.s_rx_ctx.puc_buf[i_pos], ui_ftr );
        gs_rf_ctx.s_rx_ctx.ui_rem -= ui_ftr;

        i_ret = ui_ftr;
      }
    }
  }
  return i_ret;
}

/*============================================================================*/
/**
 * \brief   Transmit data.
 *
 * \return  number of received transmitted on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_tx( void )
{
  uint16_t i_ret = -1;
  uint16_t ui_snd = 0;
  int16_t i_pos;

  if (gs_rf_ctx.s_tx_ctx.ui_rem == 0 )
  {
    /* all the data was transmitted */
    i_ret = 0;
  }
  else
  {
    if (gs_rf_ctx.s_tx_ctx.ui_rem < (RF_FIFO_LEN - RF_TX_FIFO_THR - 1) )
    {
      /* update context ... all data fits into the FIFO */
      ui_snd = gs_rf_ctx.s_tx_ctx.ui_rem;
    }
    else
    {
      /* write as much data to FIFO as possible */
      ui_snd = (RF_FIFO_LEN - RF_TX_FIFO_THR - 1);
    }

    /* calculate position in the TX buffer */
    i_pos = gs_rf_ctx.s_tx_ctx.ui_len - gs_rf_ctx.s_tx_ctx.ui_rem;

    if( (i_pos < 0) || (i_pos > RF_MAX_PKT_LEN) )
    {
      /* error in position calculation */
      i_ret = -1;
    }
    else
    {
      /* calculate remaining bytes */
      gs_rf_ctx.s_tx_ctx.ui_rem -= ui_snd;

      /* Write the frame into the FIFO. */
      RF_TXFIFOWR( &gs_rf_ctx.s_tx_ctx.puc_buf[i_pos], ui_snd );

      if( i_pos == 0 )
        /* Start transmission */
        RF_TXSTART();

      i_ret = ui_snd;
    }
  }

  return i_ret;
}


/*============================================================================*/
/**
 * \brief   Set the Tx power.
 *
 * \return  0 on success, -1 on failure.
 */
/*============================================================================*/
static int _rf_setTxPower( int8_t c_power )
{
  int i_ret = -1;
  uint8_t uc_reg = 0;

  if( (c_power < RF_TXPOWER_MIN) || (c_power > RF_TXPOWER_MAX) )
  {
    /* invalid tx power value */
    i_ret = -1;
  }
  else
  {
    /* calculate the power value */
    uc_reg = (((c_power + 18) * 2) - 1) & 0x3F;
    /* write the Tx power */
    RF_REGWR( CC1200_PA_CFG1, uc_reg );
    i_ret = 0;
  }
  return i_ret;
}


/*============================================================================*/
/**
 * \brief   Run CCA.
 *
 * \return  0 if channel is busy, 1 if channel is idle.
 */
/*============================================================================*/
static uint8_t _rf_ccaRun( void )
{
  uint8_t uc_ret = 0;
  uint8_t uc_state = 0;
  uint8_t uc_ccaAttmpt = 0;

  /* go to RX state */
  _rf_gotoRx();

  /* connect IRQs */
  bsp_extIntEnable( E_TARGET_EXT_INT_0, E_TARGET_INT_EDGE_FALLING, &_rf_isrTxFifoBelThr);
  bsp_extIntEnable( E_TARGET_EXT_INT_1, E_TARGET_INT_EDGE_RISING, &_rf_isrCCADone);
  bsp_extIntEnable( E_TARGET_EXT_INT_2, E_TARGET_INT_EDGE_FALLING, &_rf_isrTxFin);

  /* set IRQ to CCA done */
  RF_REGWR( CC1200_IOCFG2, 0x0F );

  while( (uc_ret == 0) && (uc_ccaAttmpt++ < RF_CCA_ATTEMPTS_MAX) )
  {
    /* we try to switch to FSTX to check CCA state */
    gs_rf_ctx.s_tx_ctx.uc_ccaDone = 0;
    cc1200_spiCmdStrobe( CC1200_SFSTXON );

    /* wait until CCA completed */
    while( gs_rf_ctx.s_tx_ctx.uc_ccaDone == 0);

    /* get MARC state */
    RF_REGRD( CC1200_MARC_STATUS0, uc_state );

    if( uc_state & RF_PKT_MARC_STATUS0_CCA_FAIL )
    {
      /* CCA failed */
      uc_ret = 0;
    }
    else
      /* CCA succeeded */
      uc_ret = 1;
  }

  return uc_ret;
}

/*============================================================================*/
/**
 * \brief   Run ED.
 *
 * \return  Detected energy in dBm.
 */
/*============================================================================*/
static int16_t _rf_edRun( void )
{
  int16_t i_ret = 0;
  uint8_t status;

  /* go to RX state */
  _rf_gotoRx();

  /* disable IRQs */
  bsp_extIntDisable( E_TARGET_EXT_INT_0 );
  bsp_extIntDisable( E_TARGET_EXT_INT_1 );
  bsp_extIntDisable( E_TARGET_EXT_INT_2 );

  /* read the RSSI register and wait for a valid value */
  do
  {
    RF_REGRD( CC1200_RSSI0, status );
  }while( (status & RF_RSSI_VALID_MSK) == 0 );

  /* read RSSI value */
  i_ret = _rf_readRSSI();

  return i_ret;
}

/*============================================================================*/
/**
 * \brief   Read RSSI value from registers.
 *
 * \return  Read RSSI value.
 */
/*============================================================================*/
static int16_t _rf_readRSSI( void )
{
  int16_t i_ret = 0;
  uint8_t rssi2complMSB;
  uint8_t rssi2complLSB;


  /* read the RSSI register and wait for a valid value */
  do
  {
    RF_REGRD( CC1200_RSSI0, rssi2complLSB );
  }while( (rssi2complLSB & RF_RSSI_VALID_MSK) == 0 );

  /* read 2nd RSSI register and add to value */
  RF_REGRD( CC1200_RSSI1, rssi2complMSB );
  i_ret = ((int8_t)(rssi2complMSB) << RF_RSSI_LOW_LEN) |
    ((int8_t)(rssi2complLSB) >> RF_RSSI_LOW_OFST);

  /* get real RSSI from offset and resolution */
  i_ret = (int16_t)((i_ret * RF_RSSI_RES) - RF_RSSI_OFFSET);

  return i_ret;
}

/*============================================================================*/
/*                                LOCAL ISRs                                  */
/*============================================================================*/

/*============================================================================*/
/**
 * \brief   Rx FIFO value has reached upper threshold
 */
/*============================================================================*/
static void _rf_isrRxFifoAbThr( void *p_arg)
{
  if( gs_rf_ctx.e_state == E_RF_STATE_RXBUSY )
  {
    /* Read status */
    uint8_t uc_status = 0;
    uc_status = RF_STATUS();
    RF_REGRD( CC1200_MARCSTATE, uc_status );

    /* Read data from FIFO */
    if( _rf_rx() == -1 )
    {
      /* An error occurred. Abort Receive operation. */
      gs_rf_ctx.e_state = E_RF_STATE_RXFIN;
    }
  }
}

/*============================================================================*/
/**
 * \brief   Rx Started
 */
/*============================================================================*/
static void _rf_isrRxStart( void *p_arg)
{
  if( (gs_rf_ctx.e_state == E_RF_STATE_RX) ||
    (gs_rf_ctx.e_state == E_RF_STATE_IDLE) )
  {
    LED_RX_ON();

    gs_rf_ctx.e_state = E_RF_STATE_RXBUSY;

    /* Read status */
    uint8_t uc_status = 0;
    uc_status = RF_STATUS();
    RF_REGRD( CC1200_MARCSTATE, uc_status );

    /* reset RX Context */
    memset( (void*)&gs_rf_ctx.s_rx_ctx, 0, sizeof( gs_rf_ctx.s_rx_ctx ) );
    /* receive started. */
    gs_rf_ctx.ul_busyWD = 0;
  }
}

/*============================================================================*/
/**
 * \brief   Rx finished
 */
/*============================================================================*/
static void _rf_isrRxFin( void *p_arg)
{
  if( gs_rf_ctx.e_state == E_RF_STATE_RXBUSY )
  {
    /* Read status */
    uint8_t uc_status = 0;
    uc_status = RF_STATUS();
    RF_REGRD( CC1200_MARCSTATE, uc_status );

    /* receive remaining data */
    _rf_rx();
    while (gs_rf_ctx.s_rx_ctx.ui_rem) {
        _rf_rx();
    }

    /* copy RSSI and status */
    gs_rf_ctx.s_rx_ctx.ui_len -= 2;
    gs_rf_ctx.s_rx_ctx.uc_rssi = gs_rf_ctx.s_rx_ctx.puc_buf[gs_rf_ctx.s_rx_ctx.ui_len];
    gs_rf_ctx.s_rx_ctx.uc_status = gs_rf_ctx.s_rx_ctx.puc_buf[gs_rf_ctx.s_rx_ctx.ui_len + 1];

    gs_rf_ctx.e_state = E_RF_STATE_RXFIN;
    LED_RX_OFF();
  }
}

/*============================================================================*/
/**
 * \brief   Tx FIFO value has reached lower threshold.
 */
/*============================================================================*/
static void _rf_isrTxFifoBelThr( void *p_arg)
{
  if( gs_rf_ctx.e_state == E_RF_STATE_TXBUSY )
  {
    /* TX is busy. Transmit further data */
    _rf_tx();
  }
}

/*============================================================================*/
/**
 * \brief   Tx finished
 */
/*============================================================================*/
static void _rf_isrTxFin( void *p_arg)
{
  if( gs_rf_ctx.e_state == E_RF_STATE_TXBUSY )
  {
    /* TX has finished.*/
    gs_rf_ctx.e_state = E_RF_STATE_TXFIN;
  }
}


/*============================================================================*/
/**
 * \brief   CCA Done
 */
/*============================================================================*/
static void _rf_isrCCADone( void *p_arg)
{
  /* set CCA Done Flag */
  gs_rf_ctx.s_tx_ctx.uc_ccaDone = 1;
}



/*============================================================================*/
/*                          FUNCTIONS OF THE API                              */
/*============================================================================*/

/*=============================================================================
 * rf_init()
 *============================================================================*/
int rf_init( void )
{
  int i_ret = -1;
  uint8_t uc_pn;

    /* reset the RF chip */
  _rf_reset();

  /* Read Partnumber and check*/
  RF_REGRD( CC1200_PARTNUMBER, uc_pn );

  if( uc_pn  != RF_CHIP_COMPATIBILITY )
  {
    /* chip i snot supported */
    i_ret = -1;
  }
  else
  {
    i_ret = 0;
    /* disable radio */
    _rf_gotoTrxOff();

    gs_rf_ctx.e_state = E_RF_STATE_OFF;
    gs_rf_ctx.e_mode = E_RF_MODE_TRXOFF;
    gs_rf_ctx.e_ch = E_RF_CHANNEL_TEST;
    gs_rf_ctx.e_cca = E_RF_CCA_MODE_ED_THRESHOLD;

    gs_rf_ctx.s_hndlr.pf_rx = NULL;
  }

  return i_ret;

} /* rf_init() */


/*=============================================================================
 * rf_registerHandler()
 *============================================================================*/
int rf_registerHandler( s_rf_handler_t* ps_hndl )
{
  int i_ret = -1;

  if( ps_hndl == NULL )
    /* invalid parameter */
    i_ret = -1;
  else
  {
    /* copy handlers */
    gs_rf_ctx.s_hndlr = (*ps_hndl);
  }

  return i_ret;

} /* rf_registerHandler() */


/*=============================================================================
 * rf_entry()
 *============================================================================*/
int rf_entry( void )
{
  int i_ret = 0;
  uint8_t uc_rxInd = 0;

  switch( gs_rf_ctx.e_state )
  {
    case E_RF_STATE_NINIT:
      break;

    case E_RF_STATE_OFF:
      break;

    case E_RF_STATE_IDLE:
      break;

    case E_RF_STATE_RX:
      break;

    case E_RF_STATE_TXSTART:
      break;

    case E_RF_STATE_TXBUSY:
      break;

    case E_RF_STATE_RXBUSY:
      if( (gs_rf_ctx.ul_busyWD++) > RF_BUSY_WD_MAX )
        /* restore the configured mode */
        _rf_restoreMode();
      break;

    case E_RF_STATE_TXFIN:
    case E_RF_STATE_RXFIN:
    {
      if( (gs_rf_ctx.e_state == E_RF_STATE_RXFIN) &&
          (gs_rf_ctx.s_hndlr.pf_rx != NULL ) )
      {
        /* indicate RX packet */
        uc_rxInd = 1;
      }

      if( uc_rxInd )
      {
        if( gs_rf_ctx.s_rx_ctx.uc_status & CC1200_LQI_CRC_OK_BM ) {
          /* call the Rx handler with values from the Rx context */
          CPU_ENTER_CRITICAL();
          gs_rf_ctx.s_hndlr.pf_rx( gs_rf_ctx.s_rx_ctx.puc_buf, gs_rf_ctx.s_rx_ctx.ui_len );
          CPU_EXIT_CRITICAL();
        }
      }

      // here: RF in IDLE mode already
      if (gs_rf_ctx.e_state == E_RF_STATE_TXFIN) {
          /* write configuration to the chip */
          _rf_cfgWr( gcs_rf_cfg_ewor2, (sizeof(gcs_rf_cfg_ewor2)/sizeof(s_rf_register_t)));
      }

      if (gs_rf_ctx.e_state == E_RF_STATE_RXFIN) {
          /* Flush FIFO set the first RXFIFO Threshold to 1*/
          RF_RXFIFOFLUSH();
          RF_REGWR( CC1200_FIFO_CFG, 1 );
      }

      /* goto WOR state */
      cc1200_spiCmdStrobe(CC1200_SWOR);

      /* Connect ISRs */
      bsp_extIntEnable( E_TARGET_EXT_INT_0, E_TARGET_INT_EDGE_RISING, &_rf_isrRxFifoAbThr);
      bsp_extIntEnable( E_TARGET_EXT_INT_1, E_TARGET_INT_EDGE_RISING, &_rf_isrRxStart);
      bsp_extIntEnable( E_TARGET_EXT_INT_2, E_TARGET_INT_EDGE_FALLING, &_rf_isrRxFin);

      /* clear pending flags */
      bsp_extIntClear( E_TARGET_EXT_INT_0 );
      bsp_extIntClear( E_TARGET_EXT_INT_1 );
      bsp_extIntClear( E_TARGET_EXT_INT_2 );
      
      gs_rf_ctx.e_state = E_RF_STATE_IDLE;
      break;
    }
  }

  return i_ret;

} /* rf_entry() */


/*=============================================================================
 * rf_ready()
 *============================================================================*/
uint8_t rf_ready( void )
{
  uint8_t uc_ret = RF_BUSY;

  if( (gs_rf_ctx.e_state == E_RF_STATE_IDLE) ||
      (gs_rf_ctx.e_state == E_RF_STATE_RX) ||
      (gs_rf_ctx.e_state == E_RF_STATE_OFF) )
  {
    /* RF is ready */
    uc_ret = RF_READY;
  }
  return uc_ret;
} /* rf_ready() */


/*=============================================================================
 * rf_setMode()
 *============================================================================*/
int rf_setMode( e_rf_mode_t e_mode )
{
  int i_ret = -1;


  bsp_enterCritical();
  if( rf_ready() == RF_READY )
  {

    if( gs_rf_ctx.e_state == E_RF_STATE_NINIT )
    {
      /* the RF module has't been initialized */
      i_ret = -1;
    }
    else
    {
      if( e_mode != gs_rf_ctx.e_mode )
      {
        /* reset chip */
        _rf_reset();

        /* set mode */
        gs_rf_ctx.e_mode = e_mode;

        /* set channel (also sets the mode) */
        i_ret = rf_setChannel( gs_rf_ctx.e_ch );
      }
      else
      {
        i_ret = 0;
      }
    }
  }
  bsp_exitCritical();
  

  return i_ret;

} /* rf_setMode() */


/*=============================================================================
 * rf_setChannel()
 *============================================================================*/
int rf_setChannel( e_rf_channel_t e_ch )
{
  int i_ret = -1;

  if( (gs_rf_ctx.e_state != E_RF_STATE_IDLE) &&
     (gs_rf_ctx.e_state != E_RF_STATE_RX) &&
      (gs_rf_ctx.e_state != E_RF_STATE_OFF) )
  {
    /* not allowed within this state */
    i_ret = -1;
  }
  else
  {
    if( e_ch >= E_RF_CHANNEL_MAX )
    {
      /* invalid channel selection */
      i_ret = -1;
    }
    else
    {
      /* goto idle state */
      _rf_gotoIdle();

      /* set the according channel */
      if( _rf_cfgWr( gcs_rf_ch[e_ch].ps_reg, gcs_rf_ch[e_ch].ui_num ) == -1 )
        /* configuration has not been written to the RF chip */
        i_ret = -1;
      else
      {        
        gs_rf_ctx.e_ch = e_ch;

        /* we must reset the mode */
        if( _rf_setMode( gs_rf_ctx.e_mode ) == -1 )
          /* mode could not be set */
          i_ret = -1;
        else
          i_ret = 0;
      }
    }
  }

  return i_ret;

} /* rf_setChannel() */


/*=============================================================================
 * rf_setCCAMode()
 *============================================================================*/
int rf_setCCAMode( e_rf_cca_mode_t e_cca )
{
  int i_ret = -1;

  if( e_cca >= E_RF_CCA_MODE_MAX )
  {
    /* invalid channel selection */
    i_ret = -1;
  }
  else
  {
    /* set the configuration in the context */
    gs_rf_ctx.e_cca = e_cca;

    if( gs_rf_ctx.e_state == E_RF_STATE_RX )
      /* we must reenter RX to apply the settings */
      _rf_gotoRx();

    i_ret = 0;
  }
  return i_ret;

} /* rf_setCCAMode() */

/*=============================================================================
 * rf_tx()
 *============================================================================*/
int rf_tx( uint8_t* puc_data, uint16_t ui_len, e_rf_preamble_t e_preamble,
    int8_t c_txPower )
{
  int i_ret = RF_TX_ERROR;

  if( (gs_rf_ctx.e_state != E_RF_STATE_IDLE) &&
     (gs_rf_ctx.e_state != E_RF_STATE_RX) )
  {
    /* RF is not ready to send data */
    i_ret = RF_TX_ERROR;
  }
  else
  {
    /* set according state */
    gs_rf_ctx.e_state = E_RF_STATE_TXSTART;

    /**
     * \todo  Register Interrupt Handler and enable TX interrupts.
     */
    if( ui_len > RF_MAX_PKT_LEN )
      /* packet length is too large */
      i_ret = RF_TX_ERROR;
    else
    {
#if RF_CCA_ON_TX
      /* check for an idle channel */
      if( rf_cca() == 0 )
      {
        /* channel is busy */
        i_ret = RF_TX_COLLISION;
      }
      else
#endif /* RF_CCA_ON_TX */
      {
        /* set RF to idle */
        _rf_gotoIdle();

        /* set the preamble configuration */
        RF_REGWR(CC1200_PREAMBLE_CFG1, gcpuc_rf_premble_cfg[gs_rf_ctx.e_ch][e_preamble]);

        if( _rf_setTxPower( c_txPower ) == -1 )
        {
          /* Tx power could not be set */
          i_ret = RF_TX_POWERR;
        }
        else
        {
          /* write configuration to the chip */
          if( _rf_cfgWr( gcs_rf_cfg_tx, (sizeof(gcs_rf_cfg_tx)/sizeof(s_rf_register_t)) ) == -1 )
            /* configuration has not been written to the RF chip */
            i_ret = RF_TX_ERROR;
          else
          {
            uint8_t uc_writeByte = 0;
            RF_TXFIFOFLUSH();

            /* set the TX context and copy data*/
            gs_rf_ctx.s_tx_ctx.ui_len = ui_len;
            gs_rf_ctx.s_tx_ctx.ui_rem = gs_rf_ctx.s_tx_ctx.ui_len;
            memcpy( gs_rf_ctx.s_tx_ctx.puc_buf, puc_data, ui_len );

            /* set according state */
            gs_rf_ctx.e_state = E_RF_STATE_TXBUSY;

            /* connect IRQs */
            bsp_extIntEnable( E_TARGET_EXT_INT_0, E_TARGET_INT_EDGE_FALLING, &_rf_isrTxFifoBelThr);
            bsp_extIntEnable( E_TARGET_EXT_INT_1, E_TARGET_INT_EDGE_RISING, &_rf_isrCCADone);
            bsp_extIntEnable( E_TARGET_EXT_INT_2, E_TARGET_INT_EDGE_FALLING, &_rf_isrTxFin);
            
            /* Do a normal FIFO write with fixed length */
            RF_REGWR( CC1200_PKT_CFG0, RF_PKT_MODE_FIXED_LEN );

            /* Set FIFO threshold. */
            RF_REGWR( CC1200_FIFO_CFG, (127 - RF_TX_FIFO_THR) );

            /* Configure the PKT_LEN register. */
            RF_REGWR( CC1200_PKT_LEN, (ui_len + 1) );

            /* Write the length field */
            uc_writeByte = ui_len;
            RF_TXFIFOWR( &uc_writeByte, 1 );

            /* start data transmission */
            _rf_tx();

            i_ret = RF_TX_SUCCESS;
          }
        }
      }
    }

    if( i_ret != RF_TX_SUCCESS )
      /* restore mode */
      _rf_restoreMode();
  }

  return i_ret;

} /* rf_tx() */

/*=============================================================================
 * rf_tx()
 *============================================================================*/
uint8_t rf_cca( void )
{
  uint8_t uc_ret = 0;

  e_rf_state_t e_stateSave = gs_rf_ctx.e_state;

  if( (gs_rf_ctx.e_state != E_RF_STATE_IDLE) &&
     (gs_rf_ctx.e_state != E_RF_STATE_RX) &&
     (gs_rf_ctx.e_state != E_RF_STATE_TXSTART))
  {
    /* invalid state to perform CCA */
    uc_ret = 0;
  }
  else
  {
    /* perform CCA */
    uc_ret = _rf_ccaRun();
    /* restore the configured mode */
    _rf_restoreMode();

    if( e_stateSave == E_RF_STATE_TXSTART )
      gs_rf_ctx.e_state = E_RF_STATE_TXSTART;
  }

  return uc_ret;
} /* rf_cca() */


/*=============================================================================
 * rf_ed()
 *============================================================================*/
int rf_ed( int16_t* pi_ed )
{
  int i_ret = -1;

  if( pi_ed != NULL )
  {
    if( (gs_rf_ctx.e_state != E_RF_STATE_IDLE) &&
       (gs_rf_ctx.e_state != E_RF_STATE_RX))
    {
      /* invalid state to perform ED */
      i_ret = -1;
    }
    else
    {
      /* perform ED */
      *pi_ed = _rf_edRun();
      /* restore the configured mode */
      _rf_restoreMode();
      i_ret = 0;
    }
  }

  return i_ret;
} /* rf_ed() */


#if 1 // additional functions
uint8_t rf_is_rx(void)
{
    uint8_t uc_ret = 0;

    if( (gs_rf_ctx.e_state == E_RF_STATE_RX) ||
        (gs_rf_ctx.e_state == E_RF_STATE_RXBUSY) ||
        (gs_rf_ctx.e_state == E_RF_STATE_RXFIN))
    {
      uc_ret = 1;
    }

    return uc_ret;
}


uint8_t rf_is_tx(void)
{
    uint8_t uc_ret = 0;

    if( (gs_rf_ctx.e_state == E_RF_STATE_TXSTART) ||
        (gs_rf_ctx.e_state == E_RF_STATE_TXBUSY) ||
        (gs_rf_ctx.e_state == E_RF_STATE_TXFIN) )
    {
      uc_ret = 1;
    }
    return uc_ret;
}
#endif


uint8_t rf_is_channel_busy (void)
{
    uint8_t cca;

    cca = rf_cca();

    if (cca == RF_BUSY) {
        return 1;
    } else {
        return 0;
    }
}

#ifdef __cplusplus
}
#endif