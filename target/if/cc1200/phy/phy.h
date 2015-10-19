#ifndef __PHY_H__
#define __PHY_H__
#ifndef __DECL_PHY_H__
#define __DECL_PHY_H__
#endif /* #ifndef __DECL_PHY_H__ */

/*============================================================================*/
/**
 * \file    phy.h
 *
 * \author  Manuel Schappacher
 *
 * \brief   Physical layer implementation.
 *
 */
/*============================================================================*/

/** \addtogroup phy IEEE802.15.4 PHY
 *  @{
 */

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/

#include <stdint.h>
#include <stdbool.h>
#include "phy_pib.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** \addtogroup phy_status  General Status and Return Values
 *  @{
 */
/** \brief  Successful operation */
#define PHY_SUCCESS                         (0)
/** \brief  General error */
#define PHY_ERROR                           (-1)
/** \brief  Collision detected */
#define PHY_COLL                            (-2)
/** \brief  PIB attribute is not supported */
#define PHY_ATTRIBUTE_UNSUPPORTED           (-2)
/** \brief  PIB attribute is read only */
#define PHY_ATTRIBUTE_RDONLY                (-3)
/** \brief  PIB value is invalid */
#define PHY_ATTRIBUTE_INVALID               (-4)
/** \brief  Packet size is invalid */
#define PHY_PKTSIZE_INVALID                 (-5)

/** \brief  CCA reported BUSY channel */
#define PHY_CCA_BUSY                        (0)
/** \brief  CCA reported IDLE channel */
#define PHY_CCA_IDLE                        (1)
/** @}*/

/*============================================================================*/
/*                    FUNCTION PROTOTYPES OF THE API                          */
/*============================================================================*/


/*============================================================================*/
/**
 * \brief   Initialize PHY.
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on general error
 */
/*============================================================================*/
int8_t phy_init( void );


/*============================================================================*/
/**
 * \brief   Entry function of the PHY module.
 *
 *          This function must be called periodically as often as possible
 *          to provide a proper behavior of the PHY module.
 *
 * \return  The status of the indication reception as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on general error
 */
/*============================================================================*/
int8_t phy_entry( void );


/*============================================================================*/
/**
 * \brief   Transmit data using the PHY data SAP.
 *
 *
 * \param   puc_data    Data to transmit.
 * \param   ui_len      Length of the data to transmit.
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on general error
 *          \ref PHY_COLL if a collision was detected
 */
/*============================================================================*/
int8_t PHY_PdDataReq( uint8_t* puc_data, uint16_t ui_len );


/*============================================================================*/
/**
 * \brief   Data indication from the PHY data SAP.
 *
 *          This function will be called every time a data frame was
 *          received by the physical layer. Therefore it must be implemented
 *          by a higher module.
 *
 * \param   puc_data    Data received.
 * \param   ui_len      Length of the received data.
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on general error
 */
/*============================================================================*/
int8_t PHY_PdDataInd( uint8_t* puc_data, uint16_t ui_len );


/*============================================================================*/
/**
 * \brief   Retrieve attribute value from PIB.
 *
 * \param   e_attr    Attribute to retrieve.
 * \param   p_val     Destination to write value to.
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on general error
 *          \ref PHY_ATTRIBUTE_UNSUPPORTED if attribute was not found.
 */
/*============================================================================*/
int8_t PHY_PlmeGetReq( e_phy_pib_attr_t e_attr, void* p_val );


/*============================================================================*/
/**
 * \brief   Set attribute value in PIB.
 *
 * \param   e_attr    Attribute to retrieve.
 * \param   p_val     Destination to write value to.
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on general error
 *          \ref PHY_ATTRIBUTE_UNSUPPORTED if attribute was not found.
 *          \ref PHY_ATTRIBUTE_RDONLY if attribute is read only.
 *          \ref PHY_ATTRIBUTE_INVALID if value is not valid.
 */
/*============================================================================*/
int8_t PHY_PlmeSetReq( e_phy_pib_attr_t e_attr, void* p_val );


/*============================================================================*/
/**
 * \brief   Perform a Clear Channel Assessment.
 *
 *
 * \return  The status of the request as follows:
 *          \ref PHY_CCA_BUSY if the channel is busy.
 *          \ref PHY_CCA_IDLE if the channel is idle.
 */
/*============================================================================*/
int8_t PHY_PlmeCcaReq( void );


/*============================================================================*/
/**
 * \brief   Perform an Energy Detection.
 *
 * \param   pi_ed   Buffer to write the detected value to (in dBm).
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on error (e.g. invalid state to check ED)
 */
/*============================================================================*/
int8_t PHY_PlmeEdReq( int16_t* pi_ed );


/*============================================================================*/
/**
 * \brief   Perform an Energy Detection.
 *
 * \param   pi_ed   Buffer to write the detected value to (in dBm).
 *
 * \return  The status of the request as follows:
 *          \ref PHY_SUCCESS on success
 *          \ref PHY_ERROR on error (e.g. invalid state to check ED)
 */
/*============================================================================*/
int8_t PHY_PlmeRfOn( uint8_t uc_on );



/** @}*/



#endif /* #ifndef __PHY_H__ */


