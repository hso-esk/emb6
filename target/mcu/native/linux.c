/*
 * --- License --------------------------------------------------------------*
 */
/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2016,
 * Hochschule Offenburg, University of Applied Sciences
 * Institute of reliable Embedded Systems and Communications Electronics.
 * All rights reserved.
 */

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       linux.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      PC emulation library for upper layers of emb::6.
 *
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#define _XOPEN_SOURCE
#define _XOPEN_SOURCE_EXTENDED
#define _POSIX_C_SOURCE         199309L

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/signal.h>
#include <stdlib.h>
#include "hal.h"


/*
 *  --- Macros ------------------------------------------------------------- *
 */
#define LOGGER_ENABLE           LOGGER_HAL
#include "logger.h"

#define NATIVE_TICK_SECONDS       ( 1000u )

/*
 * --- Type Definitions -----------------------------------------------------*
 */
typedef struct
{
  /** callback function */
  pf_hal_irqCb_t pf_cb;
  /** data pointer */
  void* p_data;

} s_hal_irq;


/*
 *  --- Local Variables ---------------------------------------------------- *
 */
/** Definition of the peripheral callback functions */
static s_hal_irq s_hal_irqs[EN_HAL_PERIPHIRQ_MAX];

static struct timespec tim = { 0, 0 };

#if DEMO_USE_EXTIF
static int fdm = -1;
static pf_hal_irqCb_t isr_rxCallb = NULL;
#endif /* #if DEMO_USE_EXTIF */


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */
#if DEMO_USE_EXTIF
static void _printAndExit( const char* rpc_reason );
static void signal_handler_IO (int status);
static void signal_handler_interrupt(int signum);
#endif /* #if DEMO_USE_EXTIF */


/*
 *  --- Local Functions ---------------------------------------------------- *
 */
#if DEMO_USE_EXTIF
/*---------------------------------------------------------------------------*/
/*
* putchar()
*/
int putchar (int __c)
{
    int ret = 0;
    char c = (char)__c;
    if( fdm != -1 )
        write( fdm, &c, 1 );
    return ret;
} /* putchar() */

/*---------------------------------------------------------------------------*/
/*
* _printAndExit()
*/
static void _printAndExit( const char* rpc_reason )
{
    perror(rpc_reason);
    perror("\n");
    exit( 1 );
} /* _printAndExit() */

/*---------------------------------------------------------------------------*/
/*
* signal_handler_IO()
*/
static void signal_handler_IO (int status)
{
    char bufin;
    int ret;

    do
    {
        ret=read(fdm, &bufin, 1);
        if((isr_rxCallb != NULL) && (ret > 0))
            isr_rxCallb(&bufin);
    } while( ret > 0 );
} /* signal_handler_IO() */

/*---------------------------------------------------------------------------*/
/*
* signal_handler_interrupt()
*/
static void signal_handler_interrupt(int signum)
{
    close(fdm);
    exit(1);
} /* signal_handler_interrupt() */
#endif /* #if DEMO_USE_EXTIF */


/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* hal_init()
*/
int8_t hal_init( void )
{
#if DEMO_USE_EXTIF
  struct sigaction saio;
  struct sigaction saint;
  const char *symlink_path;

  fdm = open("/dev/ptmx", O_RDWR);  /* open master */
  if( fdm < 0 )
  {
      _printAndExit( "Error on posix_openpt()" );
  }
  if( grantpt(fdm) < 0 )            /* change permission of slave */
  {
      _printAndExit( "Error on grantpt()" );
  }
  if( unlockpt(fdm) < 0 )          /* unlock slave */
  {
      _printAndExit( "Error on unlockpt()" );
  }

  saio.sa_handler = signal_handler_IO;
  saio.sa_flags = 0;
  saio.sa_restorer = NULL;
  sigaction(SIGIO,&saio,NULL);

  memset(&saint, 0, sizeof(struct sigaction));
  saint.sa_handler = signal_handler_interrupt;
  sigaction(SIGINT ,&saint,NULL);

  fcntl(fdm, F_SETFL, O_NDELAY | O_NONBLOCK | O_ASYNC);

  symlink_path = "/dev/6lbr/if";

  unlink(symlink_path);
  if ( symlink(ptsname(fdm), symlink_path) < 0 )
  {
      _printAndExit("Error on symlink");
  }
#endif /* DEMO_USE_EXTIF */

  return 0;
} /* hal_init() */

/*---------------------------------------------------------------------------*/
/*
* hal_enterCritical()
*/
int8_t hal_enterCritical( void )
{
  /* Not implemented */
  return -1;
} /* hal_enterCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_exitCritical()
*/
int8_t hal_exitCritical( void )
{
  /* Not implemented */
  return -1;
} /* hal_exitCritical() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStart()
*/
int8_t hal_watchdogStart( void )
{
  /* Not implemented */
  return -1;
} /* hal_watchdogStart() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogReset()
*/
int8_t hal_watchdogReset( void )
{
  /* Not implemented */
  return -1;
} /* hal_watchdogReset() */

/*---------------------------------------------------------------------------*/
/*
* hal_watchdogStop()
*/
int8_t hal_watchdogStop( void )
{
  /* Not implemented */
  return -1;
} /* hal_watchdogStop() */

/*---------------------------------------------------------------------------*/
/*
* hal_getrand()
*/
uint32_t hal_getrand( void )
{
  // We don't need special kind of seed or rand.
  srand(time(NULL));
  int r = rand();
  return ((uint8_t) r);
} /* hal_getrand() */

/*---------------------------------------------------------------------------*/
/*
* hal_getTick()
*/
clock_time_t hal_getTick( void )
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return ((tv.tv_sec * NATIVE_TICK_SECONDS + tv.tv_usec / NATIVE_TICK_SECONDS) & 0xffffffff);
} /* hal_getTick() */

/*---------------------------------------------------------------------------*/
/*
* hal_getSec()
*/
clock_time_t hal_getSec( void )
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec;
} /* hal_getSec() */

/*---------------------------------------------------------------------------*/
/*
* hal_getTRes()
*/
clock_time_t hal_getTRes( void )
{
  return NATIVE_TICK_SECONDS;
} /* hal_getTRes() */

/*---------------------------------------------------------------------------*/
/*
* hal_delayUs()
*/
int8_t hal_delayUs( uint32_t delay )
{
  tim.tv_nsec = delay * 1000;
  nanosleep(&tim, NULL);
  return 0;
} /* hal_delayUs() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinInit()
*/
void* hal_pinInit( en_hal_pin_t pin )
{
  return -1;
} /* hal_pinInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinSet()
*/
int8_t hal_pinSet( void* p_pin, uint8_t val )
{
  return -1;
} /* hal_pinSet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinGet()
*/
int8_t hal_pinGet( void* p_pin )
{
  return -1;
} /* hal_pinGet() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQRegister()
*/
int8_t hal_pinIRQRegister( void* p_pin, en_hal_irqedge_t edge,
    pf_hal_irqCb_t pf_cb )
{
  /* Not implemented */
  return -1;
} /* hal_pinIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQEnable()
*/
int8_t hal_pinIRQEnable( void* p_pin )
{
  /* Not implemented */
  return -1;
} /* hal_pinIRQEnable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQDisable()
*/
int8_t hal_pinIRQDisable( void* p_pin )
{
  /* Not implemented */
  return -1;
} /* hal_pinIRQDisable() */

/*---------------------------------------------------------------------------*/
/*
* hal_pinIRQClear()
*/
int8_t hal_pinIRQClear( void* p_pin )
{
  return -1;
} /* hal_pinIRQClear() */

#if defined(HAL_SUPPORT_SPI)
/*---------------------------------------------------------------------------*/
/*
* hal_spiInit()
*/
void* hal_spiInit( en_hal_spi_t spi )
{
  return NULL;
} /* hal_spiInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTRx()
*/
int32_t hal_spiTRx( void* p_spi, uint8_t* p_tx, uint8_t* p_rx, uint16_t len )
{
  return -1;
} /* hal_spiTRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiRx()
*/
int32_t hal_spiRx( void* p_spi, uint8_t * p_rx, uint16_t len )
{
  return -1;
} /* hal_spiRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_spiTx()
*/
int32_t hal_spiTx( void* p_spi, uint8_t* p_tx, uint16_t len )
{
  return -1;
} /* hal_spiTx() */
#endif /* #if defined(HAL_SUPPORT_SPI) */

#if defined(HAL_SUPPORT_UART)
/*---------------------------------------------------------------------------*/
/*
* hal_uartInit()
*/
void* hal_uartInit( en_hal_uart_t uart )
{
  return NULL;
} /* hal_uartInit() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartRx()
*/
int32_t hal_uartRx( void* p_uart, uint8_t * p_rx, uint16_t len )
{
  return 0;
} /* hal_uartRx() */

/*---------------------------------------------------------------------------*/
/*
* hal_uartTx()
*/
int32_t hal_uartTx( void* p_uart, uint8_t* p_tx, uint16_t len )
{
  return 0;
} /* hal_uartTx() */
#endif /* #if defined(HAL_SUPPORT_UART) */

/*---------------------------------------------------------------------------*/
/*
* hal_periphIRQRegister()
*/
int8_t hal_periphIRQRegister( en_hal_periphirq_t irq, pf_hal_irqCb_t pf_cb,
    void* p_data )
{
  /* set the callback and data pointer */
  s_hal_irqs[irq].pf_cb = pf_cb;
  s_hal_irqs[irq].p_data = p_data;

  return 0;
} /* hal_periphIRQRegister() */

/*---------------------------------------------------------------------------*/
/*
* hal_debugInit()
*/
int8_t hal_debugInit( void )
{
  return 0;
} /* hal_debugInit() */
