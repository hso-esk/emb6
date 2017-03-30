 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       sf_mcu_timer.h
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      API to the timer interface.
 */

#ifndef __SF_MCU_TIMER_H__
#define __SF_MCU_TIMER_H__
#ifndef __DECL_SF_MCU_TIMER_H__
#define __DECL_SF_MCU_TIMER_H__ extern
#endif /* __DECL_SF_MCU_TIMER_H__ */

/*! @defgroup sf_mcu_timer sf_mcu_timer driver
    @{
    @ingroup sf_mcu

    This sections describes the STACKFORCE mcu timer implementation. */

/*==============================================================================
                         CALLBACKS
==============================================================================*/
/*============================================================================*/
/*!
 * \brief Callback which indicates a timer event..
 *
 * \param l_count     Elapsed time in us
 */
/*============================================================================*/
typedef bool (*fp_mcu_timer_cb)(uint32_t l_count);

/******************************************************************************/

/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/
/*============================================================================*/
/*!
 * \brief  Initializes the timer.
 *
 * \param  i_ticksPerSecond Timer interrupt period.
 *
 * \return Returns @ref true, if successfully, @ref false otherwise.
 */
/*============================================================================*/
bool sf_mcu_timer_init(uint16_t i_ticksPerSecond);

/*============================================================================*/
/*!
 * \brief  Sets the callback for the timer event.
 *
 * \param  fp_tmr Function to call in case of an interrupt.
 *
 * \return Returns @ref true, if successfully, @ref false otherwise.
 */
/*============================================================================*/
bool sf_mcu_timer_setCallback(fp_mcu_timer_cb fp_tmr);

/*============================================================================*/
/*!
 * \brief  Enables the timer interrupts.
 */
/*============================================================================*/
void sf_mcu_timer_enable(void);

/*============================================================================*/
/*!
 * \brief  Disables the timer interrupts.
 */
/*============================================================================*/
void sf_mcu_timer_disable(void);

 /*!@} end of sf_mcu_timer */
#endif /* __SF_MCU_TIMER_H__ */
