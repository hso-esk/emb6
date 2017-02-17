/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_mcu_sleep.h
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      Low power implementation.
*/

#ifndef __SF_MCU_SLEEP_H__
#define __SF_MCU_SLEEP_H__
#ifndef __DECL_SF_MCU_SLEEP_H__
#define __DECL_SF_MCU_SLEEP_H__ extern
#endif /* __DECL_SF_MCU_SLEEP_H__ */

/******************************************************************************/
/** @defgroup SF_MCU_SLEEP STACKFORCE Example description
  @{  */

/******************************************************************************/
 /** @defgroup SF_MCU_SLEEP_CALLBACKS
     @ingroup  SF_MCU_SLEEP
   This section describes the callbacks for the STACKFORCE Example implementation.
 */
/******************************************************************************/

/******************************************************************************/
 /** @defgroup SF_MCU_SLEEP_API API
     @ingroup  SF_MCU_SLEEP
   This section describes the API for the STACKFORCE Example implementation.
 */
/******************************************************************************/

/******************************************************************************/
 /*! @defgroup SF_MCU_SLEEP_ENUMS SF MCU SLEEP Enumerations
     @ingroup  SF_MCU_SLEEP
      This section defines the enumerations of the mcu sleep module.
  */
/******************************************************************************/

/**@} end of SF_MCU_SLEEP */
/******************************************************************************/

/*==============================================================================
                            TYPEDEF ENUMS
==============================================================================*/
/******************************************************************************/
/*! \addtogroup SF_MCU_SLEEP_ENUMS
 *  @{ */
/*! Enumeration of possible low power modi. */
typedef enum
{
  /*! It is assumed that the mcu will enter a lpm state in which the RAM content
      will be kept. The radio will be set to power down mode. The "tmr_tick"
      counter will not be called */
  E_MCU_SLEEP_MODE_SLEEP,
  /*! It is assumed that the mcu will enter a lpm state in which the RAM content
      will not be kept. The radio will be set to power down mode. The "tmr_tick"
      counter will not be called */
  E_MCU_SLEEP_MODE_DEEPSLEEP,
  /* Currently no sleep mode selected */
  E_MCU_SLEEP_MODE_NONE
} E_MCU_SLEEP_MODE_t;
/*!@} end of SF_MCU_SLEEP_ENUMS */
/******************************************************************************/

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/

/******************************************************************************/
/** \addtogroup SF_MCU_SLEEP_API
 *  @{ */

/*============================================================================*/
/*!
 * \brief  Initialize the mcu sleep function
 *
 */
/*============================================================================*/
void sf_mcu_sleep_init(void);

/*============================================================================*/
/*!
 * \brief  Set the MCU in a sleep mode
 *
 * \param e_sleepMode  Sleep mode to use
 */
/*============================================================================*/
void sf_mcu_sleep(E_MCU_SLEEP_MODE_t e_sleepMode);

/*============================================================================*/
/*!
 * \brief  Get the current MCU sleep mode
 *
 * \return e_sleepMode  Sleep mode in use
 */
/*============================================================================*/
E_MCU_SLEEP_MODE_t sf_mcu_sleep_getMode(void);

/*============================================================================*/
/*!
 * \brief  Wake up the mcu
 *
 */
/*============================================================================*/
void sf_mcu_sleep_wakeUp(void);

 /**@} end of SF_MCU_SLEEP_API */
/******************************************************************************/

#endif /* __SF_MCU_SLEEP_H__ */
