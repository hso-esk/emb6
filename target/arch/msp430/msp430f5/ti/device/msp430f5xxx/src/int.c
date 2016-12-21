/*============================================================================*/
/**
 * \file    int.c
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
#include <stdlib.h>
#include <stdint.h>
#include <msp430.h>
#include "targetconfig.h"
#include "hal_types.h"
#include "int.h"
#include "mcu.h"
#include "io.h"


/*============================================================================*/
/*                                 MACROS                                     */
/*============================================================================*/

/** Macro to exit LPM on interrupt */
#define INT_CHECK_LPM_EXIT()   do{                                      \
                                        switch( ge_int_pm )             \
                                        {                               \
                                          case E_MCU_PM_0:              \
                                            LPM0_EXIT;                  \
                                            break;                      \
                                          case E_MCU_PM_1:              \
                                            LPM1_EXIT;                  \
                                            break;                      \
                                          case E_MCU_PM_2:              \
                                            LPM2_EXIT;                  \
                                            break;                      \
                                          case E_MCU_PM_3:              \
                                            LPM3_EXIT;                  \
                                            break;                      \
                                          case E_MCU_PM_4:              \
                                            LPM4_EXIT;                  \
                                            break;                      \
                                          default:                      \
                                            break;                      \
                                        }                               \
                                        ge_int_pm =  E_MCU_PM_AM;       \
                                       }while(0);


/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

/** Interrupt table */
pf_int_cb gpf_int_irq_table[E_IRQ_SRC_MAX] = { NULL };

/** selected power mode */
e_mcu_pm_t ge_int_pm;


/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/

/*=============================================================================
 *  int_init()
 *============================================================================*/
void int_init( void )
{
  int i = 0;

  /* initialize the interrupt callbacks */
  for( i = 0; i < E_IRQ_SRC_MAX; i++ )
    gpf_int_irq_table[i] = NULL;

  /* initialize power mode */
  ge_int_pm = E_MCU_PM_AM;
}

/*=============================================================================
 *  int_irqRegisterPort()
 *============================================================================*/
void int_irqRegister( e_int_irq_src_t e_irqSrc, pf_int_cb pf_cb )
{
  /* register Interrupt */
  gpf_int_irq_table[e_irqSrc] = pf_cb;
}/* int_irqRegisterPort() */


/*=============================================================================
 *  int_irqUnregister()
 *============================================================================*/
void int_irqUnregister( e_int_irq_src_t e_irqSrc )
{
  /* unregister interrupt */
  gpf_int_irq_table[e_irqSrc] = NULL;
}/* int_irqUnregister() */


/*=============================================================================
 *  int_setPm()
 *============================================================================*/
void int_setPm( e_mcu_pm_t e_pm )
{
  ge_int_pm = e_pm;
}/* int_setPm() */

/*============================================================================*/
/*                         INTERRUPT HANDLER                                  */
/*============================================================================*/

#if( TARGET_CONFIG_PORT1 == TRUE )
/*=============================================================================
 *  int_irqPort1Handler()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=PORT1_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(PORT1_VECTOR)))
#endif
void int_irqPort1Handler( void )
{
  s_io_port_desc_t* ps_portDesc = NULL;
  e_io_port_t e_port = E_IO_PORT_P1;
  uint8_t uc_flags = 0;

  /* get according port */
  ps_portDesc = &gps_io_port[e_port];

  /* get interrupt flags */
  uc_flags = *ps_portDesc->PIFG;
  uc_flags &= *ps_portDesc->PIE;

  if( uc_flags )
  {
    /* clear interrupt mask for port */
    *ps_portDesc->PIFG &= ~uc_flags;

    if (gpf_int_irq_table[E_INT_IRQ_SRC_P1] != NULL)
      gpf_int_irq_table[E_INT_IRQ_SRC_P1]( &uc_flags );
  }

  if( uc_flags & TARGET_CONFIG_LPM_EXIT_IOP1 )
    INT_CHECK_LPM_EXIT();
}
#endif /* #if( TARGET_CONFIG_PORT1 == TRUE ) */

#if( TARGET_CONFIG_PORT2 == TRUE )
/*=============================================================================
 *  int_irqPort2Handler()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=PORT2_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(PORT2_VECTOR)))
#endif
void int_irqPort2Handler (void)
{
  s_io_port_desc_t* ps_portDesc = NULL;
  e_io_port_t e_port = E_IO_PORT_P2;
  uint8_t uc_flags = 0;

  /* get according port */
  ps_portDesc = &gps_io_port[e_port];

  /* get interrupt flags */
  uc_flags = *ps_portDesc->PIFG;
  uc_flags &= *ps_portDesc->PIE;

  if( uc_flags )
  {
    if (gpf_int_irq_table[E_INT_IRQ_SRC_P2] != NULL)
      gpf_int_irq_table[E_INT_IRQ_SRC_P2]( &uc_flags );

    /* clear interrupt mask for port */
    *ps_portDesc->PIFG &= ~uc_flags;
  }

  if( uc_flags & TARGET_CONFIG_LPM_EXIT_IOP2 )
    INT_CHECK_LPM_EXIT();
}
#endif /* #if( TARGET_CONFIG_PORT2 == TRUE ) */

/*=============================================================================
 *  int_irqTimerA()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(TIMER1_A0_VECTOR)))
#endif
void int_irqTimerA (void)
{
  if (gpf_int_irq_table[E_INT_IRQ_SRC_T1] != NULL)
    gpf_int_irq_table[E_INT_IRQ_SRC_T1]( NULL );

#if( TARGET_CONFIG_LPM_EXIT_TIMER0 == TRUE )
  INT_CHECK_LPM_EXIT();
#endif /* #if( TARGET_CONFIG_LPM_EXIT_TIMER0 == TRUE ) */
}

/*=============================================================================
 *  int_irqTimerB()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=TIMER0_B0_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(TIMER0_B0_VECTOR)))
#endif
void int_irqTimerB (void)
{
  if (gpf_int_irq_table[E_INT_IRQ_SRC_T2] != NULL)
    gpf_int_irq_table[E_INT_IRQ_SRC_T2]( NULL );

#if( TARGET_CONFIG_LPM_EXIT_TIMER1 == TRUE )
  INT_CHECK_LPM_EXIT();
#endif /* #if( TARGET_CONFIG_LPM_EXIT_TIMER1 == TRUE ) */
}

#if( TARGET_CONFIG_UART0 == TRUE )
/*=============================================================================
 *  int_irqUART0()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=USCI_A0_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(USCI_A0_VECTOR)))
#endif
void int_irqUART0 (void)
{
  if (gpf_int_irq_table[E_INT_IRQ_SRC_UART0] != NULL)
    gpf_int_irq_table[E_INT_IRQ_SRC_UART0]( NULL );

#if( TARGET_CONFIG_LPM_EXIT_UART0 == TRUE )
  INT_CHECK_LPM_EXIT();
#endif /* #if( TARGET_CONFIG_LPM_EXIT_UART0 == TRUE ) */
}
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

#if( TARGET_CONFIG_UART1 == TRUE )
/*=============================================================================
 *  int_irqUART1()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=USCI_A1_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(USCI_A1_VECTOR)))
#endif
void int_irqUART1 (void)
{
  if (gpf_int_irq_table[E_INT_IRQ_SRC_UART1] != NULL)
    gpf_int_irq_table[E_INT_IRQ_SRC_UART1]( NULL );

#if( TARGET_CONFIG_LPM_EXIT_UART1 == TRUE )
  INT_CHECK_LPM_EXIT();
#endif /* #if( TARGET_CONFIG_LPM_EXIT_UART1 == TRUE ) */
}
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */

#if( TARGET_CONFIG_UART3 == TRUE )
/*=============================================================================
 *  int_irqUART3()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=USCI_A3_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(USCI_A3_VECTOR)))
#endif
void int_irqUART3 (void)
{
  if (gpf_int_irq_table[E_INT_IRQ_SRC_UART3] != NULL)
    gpf_int_irq_table[E_INT_IRQ_SRC_UART3]( NULL );

#if( TARGET_CONFIG_LPM_EXIT_UART3 == TRUE )
  INT_CHECK_LPM_EXIT();
#endif /* #if( TARGET_CONFIG_LPM_EXIT_UART3 == TRUE ) */
}
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

/*=============================================================================
 *  int_irqRTC()
 *============================================================================*/
#ifdef IAR_COMPILER
#pragma vector=RTC_VECTOR
__interrupt
#endif
#ifdef GCC_COMPILER
__attribute__((__interrupt__(RTC_VECTOR)))
#endif
void RTC_ISR(void)
{
  if (gpf_int_irq_table[E_INT_IRQ_SRC_RTC] != NULL)
    gpf_int_irq_table[E_INT_IRQ_SRC_RTC]( NULL );

#if( TARGET_CONFIG_LPM_EXIT_RTC == TRUE )
  INT_CHECK_LPM_EXIT();
#endif /* #if( TARGET_CONFIG_LPM_EXIT_RTC == TRUE ) */
}
