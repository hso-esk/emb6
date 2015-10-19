/*============================================================================*/
/**
 * \file    phy.c
 *
 * \author  Manuel Schappacher
 *
 * \brief   Physical layer implementation.
 *
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
#include <stddef.h>
#include <string.h>

#define __DECL_PHY_H__
#include "phy.h"

#if !defined(MAC_TDD_ENABLED)
#include "rf.h"


/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/

/** wait until RF is ready */
#define PHY_WAIT_RF_READY()             {while( rf_ready() != RF_READY ) rf_entry();}



/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/* RX event */
static void _phy_rxEvt( uint8_t* puc_data, uint16_t ui_len );

/* reset PIB */
static int8_t _phy_pibReset( void );


/*============================================================================*/
/*                              LOCAL VARIABLES                               */
/*============================================================================*/

/** stores last active RF mode */
static e_rf_mode_t ge_phy_prevRFMode;

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/*============================================================================*/
/**
 * \brief   Rx event callback from RF.
 *
 *          This function shall be called if an RX event is received by
 *          the RF module.
 */
/*============================================================================*/
static void _phy_rxEvt( uint8_t* puc_data, uint16_t ui_len )
{
  if( (puc_data != NULL) && (ui_len != 0) &&
      (ui_len <= gcs_phy_consts.aMaxPHYPacketSize) )
  {
    /* parameters and frame size are valid. Forward the received
     * data frame */
    PHY_PdDataInd( puc_data, ui_len );
  }
}

/*============================================================================*/
/**
 * \brief   Reset PIB.
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on general error
 */
/*============================================================================*/
static int8_t _phy_pibReset( void )
{
  int i_ret = PHY_SUCCESS;

  /* reset the PIB to default values */
  gs_phy_pib = gcs_phy_pibDefaults;

  return i_ret;
}

/*============================================================================*/
/*                          FUNCTIONS OF THE API                              */
/*============================================================================*/

/*=============================================================================
 * phy_init()
 *============================================================================*/
int8_t phy_init( void )
{
  int i_ret = PHY_SUCCESS;

  /* create RF callback structure */
  s_rf_handler_t s_evt = {
      .pf_rx = _phy_rxEvt
  };

  /* reset the PIB */
  _phy_pibReset();

  /* initialize the RF Module */
  if( rf_init() == -1 )
  {
    /* error during RF initialization */
    i_ret = PHY_ERROR;
  }
  else
  {
    /* register event structure at the RF driver */
    rf_registerHandler( &s_evt );

    /* Set the RF Module to Normal-Mode */
    rf_setMode( E_RF_MODE_NORMAL );
    ge_phy_prevRFMode = E_RF_MODE_NORMAL;

    /* set channel */
    rf_setChannel( E_RF_CHANNEL_TEST );
  }

  return i_ret;

} /* phy_init() */


/*=============================================================================
 * phy_entry()
 *============================================================================*/
int8_t phy_entry( void )
{
  int i_ret = PHY_SUCCESS;

  /* call entry function of the RF module */
  rf_entry();

  return i_ret;

} /* phy_entry() */


/*=============================================================================
 * PHY_PdDataReq()
 *============================================================================*/
int8_t PHY_PdDataReq( uint8_t* puc_data, uint16_t ui_len )
{
  int8_t i_ret = PHY_ERROR;

  if( (puc_data == NULL) || (ui_len == 0) )
  {
    /* invalid parameters */
    i_ret = PHY_ERROR;
  }
  else
  {
    if( ui_len > gcs_phy_consts.aMaxPHYPacketSize )
    {
      /* packet is to large */
      i_ret = PHY_PKTSIZE_INVALID;
    }
    else
    {
      /* get preamble from PIB */
      e_rf_preamble_t e_preamble =
          gcpe_phy_preamble_table[gs_phy_pib.phyPreambleSymbolLength];


      switch( rf_tx( puc_data, ui_len, e_preamble, gs_phy_pib.phyTXPower ) )
      {
        case RF_TX_ERROR:
        case RF_TX_POWERR:
          /* TX error */
          i_ret = PHY_ERROR;
          break;

        case RF_TX_COLLISION:
          /* collision detected */
          i_ret = PHY_COLL;
          break;

        default:
          /* frame transmission started successfully.
           * Wait until RF is ready */
          PHY_WAIT_RF_READY();
          i_ret = PHY_SUCCESS;
          break;
      }
    }
  }

  return i_ret;

} /* PHY_PdDataReq() */

/*=============================================================================
 * PHY_PlmeGetReq()
 *============================================================================*/
int8_t PHY_PlmeGetReq( e_phy_pib_attr_t e_attr, void* p_val )
{
  int8_t i_ret = PHY_ERROR;

  if( (p_val == NULL) )
  {
    /* invalid parameters */
    i_ret = PHY_ERROR;
  }
  else
  {
    if( e_attr >= E_PHY_PIB_MAX )
    {
      /* invalid attribute */
      i_ret = PHY_ATTRIBUTE_UNSUPPORTED;
    }
    else
    {
      /* return the value */
      memcpy( p_val, ((uint8_t*)&gs_phy_pib + gps_phy_pibTable[e_attr].uc_offset),
          gps_phy_pibTable[e_attr].uc_len );

      i_ret = PHY_SUCCESS;
    }
  }

  return i_ret;

} /* PHY_PlmeGetReq() */

/*=============================================================================
 * PHY_PlmeSetReq()
 *============================================================================*/
int8_t PHY_PlmeSetReq( e_phy_pib_attr_t e_attr, void* p_val )
{
  int8_t i_ret = PHY_ERROR;
  uint8_t uc_pibSet = 1;

  if( (p_val == NULL) )
  {
    /* invalid parameters */
    i_ret = PHY_ERROR;
  }
  else
  {
    if( e_attr >= E_PHY_PIB_MAX )
    {
      /* invalid attribute */
      i_ret = PHY_ATTRIBUTE_UNSUPPORTED;
    }
    else
    {
      if( gps_phy_pibTable[e_attr].b_rOnly == true )
      {
        /* read only value */
        i_ret = PHY_ATTRIBUTE_RDONLY;
      }
      else
      {
        /*
         * \todo: value check!
         */

        switch( e_attr )
        {
          /* set the channel within current page */
          case E_PHY_PIB_phyCurrentChannel:
            /* set the according channel */
            if( rf_setChannel( gcpe_phy_channels[gs_phy_pib.phyCurrentPage][(*(uint8_t*)p_val)] ) == -1 )
              i_ret = PHY_ERROR;
            else
              i_ret = PHY_SUCCESS;
            break;

          /* set the tx power tolerance */
          case E_PHY_PIB_phyTXPowerTolerance:
            /* success */
            i_ret = PHY_SUCCESS;
            break;

          /* set the tx power */
          case E_PHY_PIB_phyTXPower:
          {
            int8_t c_txPower = (*(uint8_t*)p_val);

            if( c_txPower < RF_TXPOWER_MIN )
            {
              /* check with power tolerance */
              c_txPower += gcpuc_phy_txtolerance_table[gs_phy_pib.phyTXPowerTolerance];

              if( c_txPower < RF_TXPOWER_MIN )
                /* still out of range */
                i_ret = PHY_ERROR;
              else
                /* now within range */
                i_ret = PHY_SUCCESS;
            }
            else if( c_txPower > RF_TXPOWER_MAX )
            {
              /* check with power tolerance */
              c_txPower -= gcpuc_phy_txtolerance_table[gs_phy_pib.phyTXPowerTolerance];

              if( c_txPower > RF_TXPOWER_MAX )
                /* still out of range */
                i_ret = PHY_ERROR;
              else
                /* now within range */
                i_ret = PHY_ERROR;
            }
            else
            {
              /* value is valid and can be taken as it is */
              i_ret = PHY_SUCCESS;
            }

            if( i_ret == PHY_SUCCESS )
            {
              /* set value in the PIB and clear the set flag */
              memcpy( ((uint8_t*)&gs_phy_pib + gps_phy_pibTable[e_attr].uc_offset), &c_txPower,
                        gps_phy_pibTable[e_attr].uc_len );
              uc_pibSet = 0;
            }

            break;
          }

          /* set the channel within current page */
          case E_PHY_PIB_phyCCAMode:
          {
            /* set the according channel */
            if( rf_setCCAMode( gcpe_phy_cca_table[(*(uint8_t*)p_val)] ) == -1 )
              i_ret = PHY_ERROR;
            else
              i_ret = PHY_SUCCESS;
            break;
          }


          /* set the page */
          case E_PHY_PIB_phyCurrentPage:
            /* set the according channel */
            if( rf_setChannel( gcpe_phy_channels[(*(uint8_t*)p_val)][gs_phy_pib.phyCurrentChannel] ) == -1 )
              i_ret = PHY_ERROR;
            else
              i_ret = PHY_SUCCESS;
            break;

          case E_PHY_PIB_phyPreambleSymbolLength:
            i_ret = PHY_SUCCESS;
            break;

          /* set sniff mode */
          case E_PHY_PIB_phySniffMode:
          {
            e_rf_mode_t e_rfMode;

            if( *((e_phy_sniffmode_t*)p_val) == E_PHY_SNIFFMODE_OFF )
              e_rfMode = E_RF_MODE_NORMAL;
            else
              e_rfMode = E_RF_MODE_PWRSAVE;

            /* set RF operation mode */
            if( rf_setMode( e_rfMode ) == -1 )
              i_ret = PHY_ERROR;
            else
            {
              ge_phy_prevRFMode = e_rfMode;
              i_ret = PHY_SUCCESS;
            }
            break;
          }

          default:
            /* success */
            i_ret = PHY_SUCCESS;
            break;
        }

        if( (i_ret == PHY_SUCCESS) && uc_pibSet )
        {
          /* operation was successful. Set the value in the PIB */
          memcpy( ((uint8_t*)&gs_phy_pib + gps_phy_pibTable[e_attr].uc_offset), p_val,
                    gps_phy_pibTable[e_attr].uc_len );
        }
      }
    }
  }

  return i_ret;

} /* PHY_PlmeSetReq() */

/*=============================================================================
 * PHY_PlmeCcaReq()
 *============================================================================*/
int8_t PHY_PlmeCcaReq( void )
{
  int8_t c_ret = PHY_CCA_BUSY;

  if( rf_cca() == 0 )
    c_ret = PHY_CCA_BUSY;
  else
    c_ret = PHY_CCA_IDLE;

  return c_ret;

}/* PHY_PlmeCcaReq() */


/*=============================================================================
 * PHY_PlmeEdReq()
 *============================================================================*/
int8_t PHY_PlmeEdReq( int16_t* pi_ed )
{
  int8_t c_ret = PHY_ERROR;

  if( pi_ed != NULL )
  {
    if( rf_ed( pi_ed ) == 0 )
      c_ret = PHY_SUCCESS;
  }

  return c_ret;

}/* PHY_PlmeEdReq() */



/*=============================================================================
 * PHY_PlmeRfOn()
 *============================================================================*/
int8_t PHY_PlmeRfOn( uint8_t uc_on )
{
  int8_t c_ret = PHY_ERROR;

  if( rf_ready() != RF_READY )
  {
    /* rf is not idle */
    c_ret = PHY_ERROR;
  }
  else
  {
    if( uc_on == true )
      /* turning RF on by setting rfMode to its previous value */
      c_ret = rf_setMode(ge_phy_prevRFMode);
    else
      /* set RF mode to TRXOFF */
      c_ret = rf_setMode(E_RF_MODE_TRXOFF);

    if( c_ret != -1 )
      c_ret = PHY_SUCCESS;
  }
  return c_ret;
}/* PHY_PlmeRfOn() */


#endif /* MAC_TDD_ENABLED */

#ifdef __cplusplus
}
#endif
