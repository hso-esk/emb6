#ifndef __IO_H__
#define __IO_H__
#ifndef __DECL_IO_H__
#define __DECL_IO_H__ extern
#endif /* #define __DECL_IO_H__ extern */

/*============================================================================*/
/**
 * \file    io.h
 *
 * \author  Tobias Neff
 *
 * \brief   GPIOs
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <msp430.h>
#include <stdint.h>
#include "hal_types.h"
#include "targetconfig.h"
#include "int.h"

/*============================================================================*/
/*                             ENUMERATIONS                                   */
/*============================================================================*/

/**
 * Available Ports
 */
typedef enum E_IO_PORT_T
{
#if (TARGET_CONFIG_PORT1 == TRUE)
  /** Port 1 */
  E_IO_PORT_P1,
#endif /* #if (TARGET_CONFIG_PORT1 == TRUE) */
#if (TARGET_CONFIG_PORT2 == TRUE)
  /** Port 2 */
  E_IO_PORT_P2,
#endif /* #if (TARGET_CONFIG_PORT2 == TRUE) */
#if (TARGET_CONFIG_PORT3 == TRUE)
  /** Port 3 */
  E_IO_PORT_P3,
#endif /* #if (TARGET_CONFIG_PORT3 == TRUE) */
#if (TARGET_CONFIG_PORT4 == TRUE)
  /** Port 4 */
  E_IO_PORT_P4,
#endif /* #if (TARGET_CONFIG_PORT4 == TRUE) */
#if (TARGET_CONFIG_PORT5 == TRUE)
  /** Port 5 */
  E_IO_PORT_P5,
#endif /* #if (TARGET_CONFIG_PORT5 == TRUE) */
#if (TARGET_CONFIG_PORT6 == TRUE)
  /** Port 6 */
  E_IO_PORT_P6,
#endif /* #if (TARGET_CONFIG_PORT6 == TRUE) */
#if (TARGET_CONFIG_PORT7 == TRUE)
  /** Port 7 */
  E_IO_PORT_P7,
#endif /* #if (TARGET_CONFIG_PORT7 == TRUE) */
#if (TARGET_CONFIG_PORT8 == TRUE)
  /** Port 8 */
  E_IO_PORT_P8,
#endif /* #if (TARGET_CONFIG_PORT8 == TRUE) */
#if (TARGET_CONFIG_PORT9 == TRUE)
  /** Port 9 */
  E_IO_PORT_P9,
#endif /* #if (TARGET_CONFIG_PORT9 == TRUE) */
#if (TARGET_CONFIG_PORT10 == TRUE)
  /** Port 10 */
  E_IO_PORT_P10,
#endif /* #if (TARGET_CONFIG_PORT10 == TRUE) */

  E_IO_PORT_MAX
} e_io_port_t;


/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/

/**
 * Description of a Port.
 *
 * Defines all the elements to access a ports fuctions and to configure
 * a port properly.
 */
typedef struct S_IO_PORT_DESC_T
{
  /** Port function selection */
  REG8B PSEL;
  /** Port direction */
  REG8B PDIR;
  /** Port input register */
  REG8B_CONST PIN;
  /** Port output register */
  REG8B POUT;
  /** resitor enable */
  REG8B PREN;
  /** drive stregth */
  REG8B PDS;
  /** interrupt flag */
  REG8B PIFG;
  /** interrupt enable */
  REG8B PIE;
  /** interrupt edge */
  REG8B PIES;

  /** port index */
  e_io_port_t e_port;
  /** interrupt source */
  e_int_irq_src_t e_irqSrc;
  /** interrupt handler to use */
  pf_int_cb pf_isr;

} s_io_port_desc_t;


/**
 * Description of a Pin.
 *
 * Defines all the elements to access a Pins fuctions and to configure
 * a Pin properly.
 */
typedef struct S_IO_PIN_DESC_T
{
  /** Port  */
  s_io_port_desc_t* PORT;
  /** PIN */
  uint8_t PIN;
    /** PIN */
  uint8_t MSK;

} s_io_pin_desc_t;


/** Prototype of an io callback */
typedef void (*pf_io_cb)( void *p_arg);

/*============================================================================*/
/*                               VARIABLES                                    */
/*============================================================================*/

/** external IO variable */
__DECL_IO_H__ s_io_port_desc_t gps_io_port[E_IO_PORT_MAX];

/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief    Function initializes the IO-Pins of the MSP430
 *
 *           This function initializes all the IO ports that are used. The
 *           module configures the ports according to the settings within
 *           the target configuration.
 *
 */
/*============================================================================*/
void io_init( void );


/*============================================================================*/
/**
 * @brief    Set an IO pin.
 *
 *           This function can be used to set the output of a
 *           specific GPIO pin. Before the pin will be set the function checks
 *           if the pin was configured as output before.
 *
 * @param    ps_pin     Pin to set.
 *
 * @return   0 on success or -1 on error (e.g. IO is configured as input).
 */
/*============================================================================*/
int8_t io_set( s_io_pin_desc_t* ps_pin );


/*============================================================================*/
/**
 * @brief    Clear an IO pin
 *
 *           This function can be used to clear the output of a
 *           specific GPIO pin. Before the pin will be cleared the function checks
 *           if the pin was configured as output before.
 *
 * @param    ps_pin     Pin to clear.
 *
 * @return   0 on success or -1 on error (e.g. IO is configured as input).
 */
/*============================================================================*/
int8_t io_clear( s_io_pin_desc_t* ps_pin );


/*============================================================================*/
/**
 * @brief    Get value of an IO pin
 *
 *           This function can be used to get the configuration of a
 *           specific GPIO pin. Before the pin will be set the function checks
 *           if the pin was configured as input before.
 *
 * @param    ps_pin     Pin to set.
 *
 * @return   0 or 1 on success (depends on if pin is set or not) or -1 on
 *           error (e.g. IO is configured as output).
 */
/*============================================================================*/
int8_t io_get( s_io_pin_desc_t* ps_pin );


/*============================================================================*/
/**
 * @brief    Function initializes and enables the interrupts.
 *
 *           This function enables the interrupt on a specific pin. A callback
 *           function must be provided that will be called every time the
 *           interrupt occurs.
 *
 * @param    ps_pin     Struct with port and pin of the interrupt to be conencted.
 * @param    uc_edge    Trigger interrupt on falling or rising edge. Possible
 *                      values are @ref INT_EDGE_FALLING and @ref INT_EDGE_RISING.
 * @param    pf_cb      Callback to register for the interrupt
 *
 * @return   none
 */
/*============================================================================*/
void io_extiRegister(s_io_pin_desc_t *ps_pin, uint8_t uc_edge, pf_io_cb pf_cb);


/*============================================================================*/
/**
 * @brief    Function clears an interrupt.
 *
 *           Used to clear the interrupt flag of an interrupt after the
 *           interrupt occured.
 *
 * @param    ps_pin     Struct with port and pin of the interrupt to be cleared.
 *
 * @return   none
 */
/*============================================================================*/
void io_extiClear(s_io_pin_desc_t *ps_pin);


/*============================================================================*/
/**
 * @brief    Function clear an interrupt.
 *
 *           Used to clear the interrupt for a specific pin.
 *
 * @param    ps_pin     Struct with port and pin of the interrupt to be disabled.
 *
 * @return   none
 */
/*============================================================================*/
void io_extiEnable(s_io_pin_desc_t *ps_pin);


/*============================================================================*/
/**
 * @brief    Function disables an interrupt.
 *
 *           Used to disable the interrupt for a specific pin.
 *
 * @param    ps_pin     Struct with port and pin of the interrupt to be disabled.
 *
 * @return   none
 */
/*============================================================================*/
void io_extiDisable(s_io_pin_desc_t *ps_pin);


#endif /* #ifndef __IO_H__ */
