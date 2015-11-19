#ifndef __INT_H__
#define __INT_H__

/*============================================================================*/
/**
 * \file    int.h
 *
 * \author  Manuel Schappacher
 *
 * \brief   Interrupts
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"
#include "mcu.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Falling edge for interrupts */
#define INT_EDGE_FALLING                0
/** Rising edge for interrupts */
#define INT_EDGE_RISING                 1

/*============================================================================*/
/*                              TYPEDEFS                                      */
/*============================================================================*/

/** Prototype of an interrupt callback */
typedef void (*pf_int_cb)( void* );

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available interrupt sources
 */
typedef enum E_INT_IRQ_SRC_T
{
  /* Timer 1 */
  E_INT_IRQ_SRC_T1,
  /* Timer 2 */
  E_INT_IRQ_SRC_T2,
  /* Port 1 */
  E_INT_IRQ_SRC_P1,
  /* Port 2 */
  E_INT_IRQ_SRC_P2,
  /* USCI_A0 */
  E_INT_IRQ_SRC_UART0,
  /* USCI_A1 */
  E_INT_IRQ_SRC_UART1,
  /* USCI_A3 */
  E_INT_IRQ_SRC_UART3,
  /* RTC */
  E_INT_IRQ_SRC_RTC,

  E_IRQ_SRC_MAX
} e_int_irq_src_t;


/*============================================================================*/
/*                               FUNCTIONS                                    */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Function initializes the Interrupts.
 */
/*============================================================================*/
void int_init( void );

/*============================================================================*/
/**
 * @brief   Register an interrupt.
 *
 *          This function registers an interrupt for a specific interrupt
 *          source. By registering an interrupt the according IRQ will be
 *          enabled and the registered callback function will be invoked on
 *          every IRQ.
 *
 * @param   e_irqSrc    Interrupt source to register a calback for.
 * @param   pf_cb       Function to be registered for the interrupt.
 *
 */
/*============================================================================*/
void int_irqRegister( e_int_irq_src_t e_irqSrc, pf_int_cb pf_cb );


/*============================================================================*/
/**
 * @brief   Unregister an interrupts.
 *
 *          This function unregisters an interrupt for a specific interrupt
 *          source. By unregistering an interrupt the according IRQ will be
 *          disabled.
 *
 * @param   e_irqSrc    Interrupt source to unregister.
 */
/*============================================================================*/
void int_irqUnregister( e_int_irq_src_t e_irqSrc );


/*============================================================================*/
/**
 * @brief   Set the current power mode.
 *
 *          This function sets the current power mode. This is needed that in
 *          case of an interrupt the power mode can be left. Therefore the
 *          according power mode needs to be set before
 *
 * @param   e_pm        Power mode to set.
 */
/*============================================================================*/
void int_setPm( e_mcu_pm_t e_pm );

#endif /* #ifndef __INT_H__ */
