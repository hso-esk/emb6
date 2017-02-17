/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_mcu_timerRtos.h
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      Header for the RTOS timer implementation.
*/
#ifndef __SF_MCU_TIMER_RTOS_H__
#define __SF_MCU_TIMER_RTOS_H__
#ifndef __DECL_SF_MCU_TIMER_RTOS_H__
#define __DECL_SF_MCU_TIMER_RTOS_H__ extern
#endif /* __DECL_SF_MCU_TIMER_RTOS_H__ */

/******************************************************************************/
/*! @defgroup SF_MCU_TIMER_RTOS STACKFORCE Example description
  @{  */

/******************************************************************************/
 /*! @defgroup SF_MCU_TIMER_RTOS_API API
     @ingroup  SF_MCU_TIMER_RTOS
   This section describes the API for the STACKFORCE Example implementation.
 */
/******************************************************************************/

/*!@} end of SF_MCU_TIMER_RTOS */
/******************************************************************************/

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/
/******************************************************************************/
/*! \addtogroup SF_MCU_TIMER_RTOS_API
 *  @{ */

/**
  @brief  Performs the initialization of the rtos related timer.

  @param i_ticksPerSecond  Timer ticks per second
  @param ps_eb             Pointer to the error handler

  @return On successful initialization the function should return @c true.
          Just in case the initialization wasn't possible for any reason, this
          function should return @c false.
*/
bool sf_mcu_timerRtos_init(uint16_t i_ticksPerSecond, Error_Block* ps_eb);

 /*!@} end of SF_MCU_TIMER_RTOS_API */
/******************************************************************************/

#endif /* __SF_MCU_TIMER_RTOS_H__ */
