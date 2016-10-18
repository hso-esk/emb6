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
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
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
 */
/*============================================================================*/
/**
 * \addtogroup bsp
 * @{
 * \addtogroup mcu MCU HAL library
 * @{
 */
/**
 * \addtogroup linux
 * @{
 *
 * This is an PC emulation library for upper layers.
 *
 */
/*! \file   linux/linux.c

    \author Artem Yushev 

    \brief  This is an PC emulation library for upper layers.

   \version 0.0.1
*/
/*============================================================================*/
/*==============================================================================
                                     MACROS
==============================================================================*/
#define     LOGGER_ENABLE        LOGGER_HAL
#define     _POSIX_C_SOURCE      199309L

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/
#define _XOPEN_SOURCE
#define _XOPEN_SOURCE_EXTENDED
#include <stdio.h>
#include "target.h"
#include "hwinit.h"
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/signal.h>
#include <stdlib.h>

#include "logger.h"
/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/

/*==============================================================================
                           LOCAL FUNCTION PROTOTYPES
==============================================================================*/
#if DEMO_USE_EXTIF
static void _printAndExit( const char* rpc_reason );
static void signal_handler_IO (int status);
static void signal_handler_interrupt(int signum);
#endif /* #if DEMO_USE_EXTIF */

/*==============================================================================
                          VARIABLE DECLARATIONS
==============================================================================*/
static    struct timespec             tim = {0,0};
#if DEMO_USE_EXTIF
static int fdm = -1;
pfn_intCallb_t isr_rxCallb = NULL;
#endif /* #if DEMO_USE_EXTIF */
/*==============================================================================
                                LOCAL CONSTANTS
==============================================================================*/


/*==============================================================================
                                LOCAL FUNCTIONS
==============================================================================*/

#if DEMO_USE_EXTIF
int putchar (int __c)
{
    int ret = 0;
    char c = (char)__c;
    if( fdm != -1 )
        write( fdm, &c, 1 );
    return ret;
}

static void _printAndExit( const char* rpc_reason )
{
    perror(rpc_reason);
    perror("\n");
    exit( 1 );
}

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
}

static void signal_handler_interrupt(int signum)
{
    close(fdm);
    exit(1);
}

#endif /* #if DEMO_USE_EXTIF */

/*==============================================================================
                                 API FUNCTIONS
==============================================================================*/

/*==============================================================================
 hal_extIntInit()
 =============================================================================*/
void hal_extiRegister( en_targetExtInt_t e_extInt, en_targetIntEdge_t e_edge,
		pfn_intCallb_t pfn_intCallback )
{
    hal_enterCritical();

    if( pfn_intCallback != NULL )
    {
		switch( e_extInt )
		{
			case E_TARGET_RADIO_INT:
				break;
#if DEMO_USE_EXTIF
			case E_TARGET_USART_INT:
				isr_rxCallb = pfn_intCallback;
				break;
#endif /* DEMO_USE_EXTIF */
			default:
				break;
		}
    }
    hal_exitCritical();
} /* hal_extIntInit() */

/*==============================================================================
 hal_extiClear()
 =============================================================================*/
void hal_extiClear(en_targetExtInt_t e_extInt)
{
} /* hal_extiClear() */

/*==============================================================================
 hal_extiEnable()
 =============================================================================*/
void hal_extiEnable(en_targetExtInt_t e_extInt)
{
} /* hal_extiEnable() */

/*==============================================================================
 hal_extiDisable()
 =============================================================================*/
void hal_extiDisable(en_targetExtInt_t e_extInt)
{
} /* hal_extiDisable() */

/*==============================================================================
  hal_delay_us()
 =============================================================================*/
void    hal_delay_us(uint32_t l_delay)
{
    tim.tv_nsec = l_delay*1000;
    nanosleep(&tim, NULL);
} /* hal_delay_us() */


void hal_enterCritical(void){}
void hal_exitCritical(void){};
void hal_ledOff(uint16_t ui_led){}
void hal_ledOn(uint16_t ui_led){}
int8_t hal_init (void){

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
    printf("The slave side is named : %s\n", ptsname(fdm));

    symlink_path = "/dev/6lbr/if";

    unlink(symlink_path);
    if ( symlink(ptsname(fdm), symlink_path) < 0 )
    {
        _printAndExit("Error on symlink");
    }

#endif /* DEMO_USE_EXTIF */

    return 1;
}
void hal_watchdogReset(void){}
void hal_watchdogStart(void){}
void hal_watchdogStop(void){}


uint8_t    hal_getrand(void)
{
    // We don't need special kind of seed or rand.
    srand(time(NULL));
    int r = rand();
    return ((uint8_t) r);
}

clock_time_t hal_getTRes(void)
{
    return 1000;
}
/*==============================================================================
  hal_getTick()
 =============================================================================*/
uint32_t     hal_getTick(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ((ts.tv_sec * 1000) + (ts.tv_nsec/1000000));
//      struct timeval tv;
//      gettimeofday(&tv, NULL);
//      return ((tv.tv_sec * 1000 + tv.tv_usec / 1000) & 0xffffffff);

} /* hal_getTick() */

/*==============================================================================
  hal_getSec()
 =============================================================================*/
uint32_t     hal_getSec(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec);
//      struct timeval tv;
//      gettimeofday(&tv, NULL);
//      return tv.tv_sec;
} /* hal_getSec() */
/** @} */
/** @} */
/** @} */
