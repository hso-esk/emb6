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
 *  --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       main.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Main file used for all demo applications.
 *
 *              This file provides the entry for all demo applications. depending
 *              on the configuration it initializes the stack and the according
 *              modules before it starts operation.
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "board_conf.h"
#include "bsp.h"
#include "etimer.h"

#define  LOGGER_ENABLE        LOGGER_MAIN
#include "logger.h"

#if DEMO_USE_UDP
#include "demo_udp.h"
#endif /* #if DEMO_USE_UDP */

#if DEMO_USE_UDP_SOCKET
#include "demo_udp_socket.h"
#endif /* #if DEMO_USE_UDP_SOCKET */

#if DEMO_USE_UDP_SOCKET_SIMPLE
#include "demo_udp_socket_simple.h"
#endif /* #if DEMO_USE_UDP_SOCKET_SIMPLE */

#if DEMO_USE_LWM2M
#if CONF_USE_SERVER
#else
#include "demo_lwm2m_cli.h"
#endif /* #if CONF_USE_SERVER */
#endif /* #if DEMO_USE_LWM2M */

#if DEMO_USE_COAP
#if CONF_USE_SERVER
#include "demo_coap_srv.h"
#else
#include "demo_coap_cli.h"
#endif /* #if CONF_USE_SERVER */
#endif /* #if DEMO_USE_COAP */

#if DEMO_USE_MDNS
#if CONF_USE_SERVER
#include "demo_mdns_srv.h"
#else
#include "demo_mdns_cli.h"
#endif /* #if CONF_USE_SERVER */
#endif /* #if DEMO_USE_MDNS */

#if DEMO_USE_SERIALAPI
#include "demo_serialapi.h"
#endif /* #if DEMO_USE_SERIALAPI */

#if DEMO_USE_UDPALIVE
#include "demo_udp_alive.h"
#endif /* #if DEMO_USE_UDPALIVE */

#if DEMO_USE_APTB
#include "demo_aptb.h"
#endif /* #if DEMO_USE_APTB */

#if DEMO_USE_EXTIF
#include "slip_radio.h"
#include "slip.h"
#endif /* #if DEMO_USE_EXTIF */

#if DEMO_USE_DTLS
#if CONF_USE_SERVER
#include "demo_dtls_srv.h"
#else
#include "demo_dtls_cli.h"
#endif /* #if CONF_USE_SERVER */
#endif /* #if DEMO_USE_DTLS */

#if DEMO_USE_6TISCH
#include "demo_6tisch.h"
#endif /* #if DEMO_USE_6TISCH */

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
#endif /* #if UIP_CONF_IPV6_RPL */

#if USE_FREERTOS
 /* Scheduler includes. */
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#endif /* #if USE_FREERTOS */

#if USE_TI_RTOS
/* XDCtools Header files */
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "ti_rtos_src/Board.h"
#include "ti_rtos_src/sf_mcu_timerRtos.h"

#include "emb6_task.h"
#include "emb6_semaphore.h"

#endif /* #if USE_TI_RTOS */
/*
 * --- Macro Definitions --------------------------------------------------- *
 */

/** Maximum number of demos */
#ifndef EMB6_DEMOS_MAX
#define EMB6_DEMOS_MAX                      11
#endif /* #ifndef EMB6_DEMOS_MAX */

/** default delay for running the emb6 process in microseconds */
#ifndef EMB6_PROC_DELAY
#define EMB6_PROC_DELAY                     500
#endif /* #ifndef EMB6_PROC_DELAY */

#if USE_FREERTOS
/* Task priorities. */
#define mainEMB6_TASK_PRIORITY          ( tskIDLE_PRIORITY + 1 )
#define mainLED_TASK_PRIORITY           ( tskIDLE_PRIORITY + 2 )
#endif /* #if USE_FREERTOS */

#define EMB6_DEMO_SET( i,name, demos )      do{                                             \
                                              if( (i < EMB6_DEMOS_MAX) && (i >= 0) )        \
                                              {                                             \
                                                if( i > 0 )                                 \
                                                  demos[i-1].p_next = &demos[i];            \
                                                demos[i].pf_conf = demo_##name##Conf;       \
                                                demos[i++].pf_init = demo_##name##Init;     \
                                              }                                             \
                                              else                                          \
                                                i = -1;                                     \
                                            }while(0);

#if USE_FREERTOS & USE_TI_RTOS
#error Please choose only one RTOS
#endif
/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/**
 * emb6 task parameters
 */
typedef struct
{
  /** MAC address */
  uint16_t ui_macAddr;
  /** Demos */
  s_demo_t* p_demos;
}s_emb6_startup_params_t;

#if USE_FREERTOS
/**
 * LED task parameters
 */
typedef struct
{
    int en;
    int delay;

}s_led_task_param_t;
#endif /* #if USE_FREERTOS */


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Demo configurations */
static s_demo_t emb6_demos[EMB6_DEMOS_MAX];

/** parameters for emb6 startup */
static s_emb6_startup_params_t emb6_startupParams;

#if USE_FREERTOS
/** parameters for the LED Taks */
static s_led_task_param_t ledTaskParams;
#endif /* #if USE_FREERTOS */


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Parse a string including a mac address. For more information please refer to the
 * function definition. */
static uint16_t loc_parseMac( const char* mac, uint16_t defaultMac );

/* Configures the stack. For more information please refer to the
 * function definition. */
static void loc_stackConf(uint16_t mac_addr_word);

/**
 * emb6 task.
 */
#if USE_TI_RTOS
static void emb6_task(int argc, char **argv);
#else
static void emb6_task( void* p_params );
#endif /* #if USE_TI_RTOS */

#if USE_FREERTOS
/* FreeRTOS LED task. For more information please refer to the
 * function definition. */
static void vLEDTask( void *pvParameters );

/* Configure the hardware as required by the demo. For more information please
 * refer to the function definition. */
static void prvSetupHardware( void );

/* Put the CPU into the least low power low power mode. For more
 * information please refer to the function definition. */
static void prvLowPowerMode1( void );
#endif /* #if USE_FREERTOS */

/*
 *  --- Local Functions ---------------------------------------------------- *
 */


/**
 * \brief   Parse a string including a mac address.
 *
 *          This function parses a string containing a mac address in the
 *          format "0xAA" or "AA".
 *
 * \param   mac         String to parse.
 * \param   defaultMac  Default mac to use if string can not be parsed.
 *
 * \return  The parsed mac address on success or the defaul address on error.
 */
static uint16_t loc_parseMac( const char* mac, uint16_t defaultMac )
{
    int mac_addr_word;

    if( !mac )
      return defaultMac;

    if( sscanf(mac, "0x%X", &mac_addr_word) )
        return (uint16_t)mac_addr_word;

    if( sscanf(mac, "%X", &mac_addr_word) )
        return (uint16_t)mac_addr_word;

    return defaultMac;
}


/**
 * \brief   Configure the stack from board configuration parameters.
 *
 *          This function uses the parameters from the board configuration
 *          or from the compile command to configure the stack.
 *
 * \param   mac_addr_word   Last two bytes of the mac address.
 */
static void loc_stackConf( uint16_t mac_addr_word )
{
    /* set last byte of mac address */
    mac_phy_config.mac_address[7] = (uint8_t)mac_addr_word;
    mac_phy_config.mac_address[6] = (uint8_t)(mac_addr_word >> 8);

    /* initial TX Power Output in dBm */
    mac_phy_config.init_power = TX_POWER;

    /* initial RX Sensitivity in dBm */
    mac_phy_config.init_sensitivity = RX_SENSITIVITY;

    /* initial wireless mode  */
    mac_phy_config.modulation = MODULATION;
}


/**
 * \brief   Set the demo applications.
 *
 *          This function sets the demo applications according
 *          according to the build configuration. This will be used
 *          by the stack to configure and to initialize the according
 *          demos
 *
 * \return  Pointer to the demo structures on success or NULL on error.
 */
static s_demo_t* loc_demoAppsSet( void )
{
    int16_t ret = 0;
#if DEMO_USE_EXTIF
    EMB6_DEMO_SET( ret, extif, emb6_demos );
#endif /* #if DEMO_USE_EXTIF */

#if DEMO_USE_COAP
    EMB6_DEMO_SET( ret, coap, emb6_demos );
#endif /* #if DEMO_USE_COAP */

#if DEMO_USE_LWM2M
    EMB6_DEMO_SET( ret, lwm2m, emb6_demos );
#endif /* #if DEMO_USE_LWM2M */

#if DEMO_USE_MDNS
    EMB6_DEMO_SET( ret, mdns, emb6_demos );
#endif /* #if DEMO_USE_MDNS */

#if DEMO_USE_UDPALIVE
    EMB6_DEMO_SET( ret, udpAlive, emb6_demos );
#endif /* #if DEMO_USE_UDPALIVE */

#if DEMO_USE_UDP_SOCKET
    EMB6_DEMO_SET( ret, udpSocket, emb6_demos );
#endif /* #if DEMO_USE_UDP_SOCKET */

#if DEMO_USE_UDP_SOCKET_SIMPLE
    EMB6_DEMO_SET( ret, udpSocketSimple, emb6_demos );
#endif /* #if DEMO_USE_UDP_SOCKET_SIMPLE */

#if DEMO_USE_APTB
    EMB6_DEMO_SET( ret, aptb, emb6_demos );
#endif /* #if DEMO_USE_APTB */

#if DEMO_USE_6TISCH
    EMB6_DEMO_SET( ret, 6tisch, emb6_demos );
#endif /* #if DEMO_USE_6TISCH */

#if DEMO_USE_DTLS
    EMB6_DEMO_SET( ret, dtls, emb6_demos );
#endif /* #if DEMO_USE_DTLS */
    if( ret > 0 )
      return emb6_demos;
    else
      return NULL;
}


/**
 * \brief   Main emb6 task.
 *
 *          This function represents the main emb6 tasks. FIrst of all
 *          the board support package will be initialized to provide
 *          a well working hardware. As next steps the stack and demo
 *          applications will be configured before the stack is
 *          initialized. Finally the demo applications will be initialized
 *          and the stack is operating.
 *
 * \param   p_params      Parameters used to execute the stack.
 */
#if USE_TI_RTOS
static void emb6_task(int argc, char **argv)
#else
static void emb6_task( void* p_params )
#endif /* #if USE_TI_RTOS */
{
    s_ns_t s_ns;
#if USE_TI_RTOS
    s_emb6_startup_params_t* ps_params = &emb6_startupParams;
#else
    s_emb6_startup_params_t* ps_params = p_params;
#endif /* #if USE_TI_RTOS */
    uint8_t ret;
    e_nsErr_t err;

    /* Initialize variables */
    err = NETSTK_ERR_NONE;
    memset( &s_ns, 0, sizeof(s_ns) );

    /* Configure stack parameters */
    loc_stackConf( ps_params->ui_macAddr );

    /* check demo configuration */
    if( ps_params->p_demos == NULL )
    {
      /* no demos configured */
      err = NETSTK_ERR_INIT;
      emb6_errorHandler( &err );
    }

    /* Initialize BSP */
    ret = bsp_init( &s_ns );
    if( ret != 0 )
    {
        /* no recovery possible, call global error handler. */
        err = NETSTK_ERR_INIT;
        emb6_errorHandler( &err );
    }

    /* Initialize stack */
    emb6_init( &s_ns, ps_params->p_demos, &err );
    if( err != NETSTK_ERR_NONE )
    {
        /* no recovery possible, call global error handler. */
        emb6_errorHandler(&err);
    }

    /* Show that stack has been launched */
    bsp_led(HAL_LED0, EN_BSP_LED_OP_ON);
    bsp_delayUs(2000000);
    bsp_led(HAL_LED0, EN_BSP_LED_OP_OFF);

    while(1)
    {
        /* run the emb6 stack */
#if USE_FREERTOS
        emb6_process(-1);
#else
        bsp_watchdog( EN_BSP_WD_START );
        emb6_process(EMB6_PROC_DELAY);
        bsp_watchdog( EN_BSP_WD_RESET );
#endif /* #if USE_FREERTOS */
    }

    /* the program should never come here */
    return;
}


#if USE_FREERTOS
/*-----------------------------------------------------------*/
static void vLEDTask( void *pvParameters )
{
    s_led_task_param_t* p_params = (s_led_task_param_t*)pvParameters;
    while(1)
    {
        if( p_params->en )
        {
            bsp_led( HAL_LED0, EN_BSP_LED_OP_TOGGLE );
            bsp_led( HAL_LED1, EN_BSP_LED_OP_TOGGLE );
        }

        vTaskDelay( p_params->delay / portTICK_PERIOD_MS );
    }
}

/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
    /* Use the idle task to place the CPU into a low power mode.  Greater power
    saving could be achieved by not including any demo tasks that never block. */
    prvLowPowerMode1();
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{

    /* This function will be called if a task overflows its stack, if
    configCHECK_FOR_STACK_OVERFLOW != 0.  It might be that the function
    parameters have been corrupted, depending on the severity of the stack
    overflow.  When this is the case pxCurrentTCB can be inspected in the
    debugger to find the offending task. */
    for( ;; );
}

/*-----------------------------------------------------------*/
static void prvSetupHardware( void )
{
    /* Nothing to do */
}

/*-----------------------------------------------------------*/
static void prvLowPowerMode1( void )
{
    /* not implemented */
}
#endif /* #if USE_FREERTOS */

/*
 *  --- Main function ------------------------------------------------------ *
 */
#if defined(MAIN_WITH_ARGS)
int main(int argc, char **argv)
#else
int main(void)
#endif /* #if defined(MAIN_WITH_ARGS) */
{
    /* set startup parameter to zero */
    memset(&emb6_startupParams, 0, sizeof(emb6_startupParams));

#if defined(MAIN_WITH_ARGS)
    if (argc > 1) {
      emb6_startupParams.ui_macAddr = loc_parseMac(argv[1], MAC_ADDR_WORD);
    }
    else
#endif /* #if defined(MAIN_WITH_ARGS) */
	  emb6_startupParams.ui_macAddr = loc_parseMac(NULL, MAC_ADDR_WORD);
    emb6_startupParams.p_demos = loc_demoAppsSet();

#if USE_FREERTOS
    ledTaskParams.en = 1;
    ledTaskParams.delay = 1000;

    /* Perform the necessary hardware configuration. */
    prvSetupHardware();

    /* Create EMB6 task. */
    xTaskCreate( emb6_task, "EMB6Task", 2048, &emb6_startupParams, mainEMB6_TASK_PRIORITY, NULL );


    /* Create LED task. */
    xTaskCreate( vLEDTask, "LEDTask", configMINIMAL_STACK_SIZE, &ledTaskParams, mainLED_TASK_PRIORITY, NULL );

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* The scheduler should now be running the tasks so the following code should
    never be reached.  If it is reached then there was insufficient heap space
    for the idle task to be created.  In this case the heap size is set by
    configTOTAL_HEAP_SIZE in FreeRTOSConfig.h. */
    for( ;; );
#elif USE_TI_RTOS
    Error_Block eb;

    /* Call board init functions */
    Board_initGeneral();
    Board_initUART();

    /* Initialize error parameters */
    Error_init(&eb);
    /* Initialize serial task */
    emb6_task_init( (ti_sysbios_knl_Task_FuncPtr) &emb6_task, &eb);
    /* Initialize semaphore to pend task */
    semaphore_init(&eb);
    /* Initialize the periodical clock source of the wmbus stack */
    sf_mcu_timerRtos_init(2000U, &eb);

    /* Start BIOS */
    BIOS_start();
#else
    emb6_task( &emb6_startupParams );
#endif /* #if USE_FREERTOS */
}

