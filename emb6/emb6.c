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
 *  \file       emb6.h
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      Implementation of the main emb6 interface.
 *
 *              This files provides the main interface for emb6. This includes
 *              the according functions to run and configure emb6.
 */

/*
 *  --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "tcpip.h"
#include "bsp.h"
#include "evproc.h"
#include "queuebuf.h"
#include "linkaddr.h"
#include "ctimer.h"
#include "rt_tmr.h"
#include "random.h"

#if NETSTACK_CONF_WITH_IPV6
#include "uip-ds6.h"
#endif

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
#endif

#if NETSTK_CFG_LPM_ENABLED
#include "lpm.h"
#endif

#define LOGGER_ENABLE           LOGGER_CORE
#include "logger.h"


/*
 *  --- Type Definitions -----------------------------------------------------*
 */


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Pointer to the stack structure */
static s_ns_t* ps_stack;
/** Pointer to the demo structures */
static s_demo_t* ps_dms;


/*
 *  --- Global Variables ---------------------------------------------------- *
 */

/** Host L2 address */
#if UIP_CONF_LL_802154
uip_lladdr_t uip_lladdr;
#else /*UIP_CONF_LL_802154*/
uip_lladdr_t uip_lladdr = {{0x00,0x06,0x98,0x00,0x02,0x32}};
#endif /*UIP_CONF_LL_802154*/


/** RPL default Configuration */
s_rpl_conf_t rpl_config = {

    .DIOintmin = 10,
    .DIOintdoub = 12,
    /* This value decides which DAG instance we should
     * participate in by default. */
    .defInst = 0x1e,
    /* Initial metric attributed to a link when the ETX is unknown */
    .linkMetric = 2,
    .defRouteTimeUnit = 0xffff,
    .defRouteTime = 0xff,

};


/** PHY/MAC default Configuration */
s_mac_phy_conf_t mac_phy_config = {

    /* by default the configuration is not yet saved */
    .is_saved  = FALSE,

#if DEMO_USE_EXTIF
    /* set extif mac address */
    .mac_address = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
#else
    /* set default mac address */
    .mac_address = {0x00, 0x50, 0xc2, 0xff, 0xfe, 0xa8, 0xdd, 0xdd},
#endif

    /* set default pan id */
    .pan_id = 0xABCD,

    /* initial transmit power */
    .init_power = 11,

    /* initial sensivity */
    .init_sensitivity = -100,

    /* modulation mode */
    .modulation = MODULATION_BPSK20,

    /* CRC size (16Bit or 32it) */
    .fcs_len = 2,

    /* IEEE802.15.4g operation mode */
    .op_mode = NETSTK_RF_OP_MODE_1,

    /* IEEE802.15.4g channel selection */
    .chan_num = 26,

#if (NETSTK_CFG_WOR_EN == TRUE)
    /* Length of the preamble used for WoR */
    .preamble_len = 24,
#else
    /* Default preamble length */
    .preamble_len = 4,
#endif /* #if (NETSTK_CFG_WOR_EN == TRUE) */

#if (NETSTK_CFG_LOW_POWER_MODE_EN == TRUE)
    /** sleep period in ticks in Low-Power mode */
    .sleepTimeout = 200,
#endif /* #if (NETSTK_CFG_LOW_POWER_MODE_EN == TRUE) */
};


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/* Initialize the stack structure. For further information refer to
 * the function definition. */
static int8_t loc_stackInit( s_ns_t* ps_ns );

/* Configure the stack demos. For further information refer to
 * the function definition. */
static int8_t loc_demoConf( s_ns_t* ps_ns, s_demo_t* p_demos );

/* Initialize the stack demos. For further information refer to
 * the function definition. */
static int8_t loc_demoInit( s_demo_t* p_demos );

#if EMB6_INIT_ROOT == TRUE
/* Initialize the DAGROOT. For further information refer to
 * the function definition. */
static int8_t loc_dagRootInit( void );
#endif /* #if EMB6_INIT_ROOT == TRUE */

/* Set the stack status. For further information refer to
 * the function definition. */
static void loc_set_status( e_stack_status_t status );

/** Called by the stack in case new data was available from the RX interface.
 * For further details have a look at the function definitions. */
static void loc_event_callback( c_event_t ev, p_data_t data );

#if (NETSTK_CFG_LPM_ENABLED == TRUE)
static int32_t loc_stackIdle(void)
{
    e_nsErr_t err;
    uint8_t isStackBusy = FALSE;
    uint8_t isNetstkBusy = FALSE;
    en_evprocResCode_t evprocState = E_QUEUE_EMPTY;
    clock_time_t nextTimerEvent = 0;
    int32_t ret = LPM_MOD_BUSY;

    /* check if the netstack is busy
     * the netstack submodule should first check if it is busy.
     * If yes, the submodule should declare the stack is busy.
     * If not, the submodule shall forward the command to the next lower layer. */
    ps_stack->dllc->ioctrl(NETSTK_CMD_IS_BUSY, &isNetstkBusy, &err);

    /* check if there is any pending event to be processed */
    evprocState = evproc_nextEvent();

    /* check if the stack is busy */
    isStackBusy = (isNetstkBusy == TRUE) ||
                  (evprocState != E_QUEUE_EMPTY);

    /* enter sleep if the stack is not busy */
    if (isStackBusy == FALSE) {
        /* obtain next timer event to determine sleep duration */
        nextTimerEvent = etimer_nextEvent();
        if (nextTimerEvent == TMR_NOT_ACTIVE) {
            ret = LPM_MOD_IDLE;
            TRACE_LOG_ERR("LPM should never come here");
        }
        else {
            /* compute sleeping time */
            nextTimerEvent -= bsp_getTick();
            ret = nextTimerEvent;
        }
    }
    return ret;
}
#endif /* #if (NETSTK_CFG_LPM_ENABLED == TRUE) */


/*
 *  --- Local Functions ---------------------------------------------------- *
 */


/**
 * \brief   Initialize the network stack.
 *
 *          This function initializes the stack. This includes the
 *          initialization of the different layers.
 *
 * \param   ps_ns   Stack structure to initialize.
 *
 * \return  0 on success or nagtive value on error.
 */
static int8_t loc_stackInit( s_ns_t* ps_ns )
{
  uint8_t ret = 0;
  uint8_t is_valid;
  e_nsErr_t err;

  /*
   * Verify stack submodule drivers
   */
  is_valid = (ps_ns->rf     != NULL) &&
             (ps_ns->phy    != NULL) &&
             (ps_ns->mac    != NULL) &&
             (ps_ns->dllc   != NULL) &&
             (ps_ns->dllsec != NULL) &&
             (ps_ns->hc     != NULL);
  if (is_valid) {
    /*
     * Netstack submodule initializations
     */
    ps_ns->rf->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->phy->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->mac->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->dllc->init(ps_ns, &err);
    if (err != NETSTK_ERR_NONE) {
      emb6_errorHandler(&err);
    }

    ps_ns->dllsec->init(ps_ns);
    ps_ns->hc->init(ps_ns);
    ps_ns->frame->init(ps_ns);

    /* Initialize TCP/IP stack */
    tcpip_init();
  }
  else
  {
    /* invalid configuration can not be initialized */
    ret = -1;
  }


#if EMB6_INIT_ROOT == TRUE
    if ( (ret != 0) || !loc_dagRootInit()) {
        ret = -1;
    }
#endif /* #if EMB6_INIT_ROOT == TRUE */

    return ret;
}


/**
 * \brief Configure selected demos.
 *
 *        This function configures all the demos given in the list of
 *        selected demos.
 *
 * \param ps_ns     Stack structure used to configure.
 * \param p_demos   Demos to configure.
 *
 * \return  0 on success or nagative value on error.
 */
static int8_t loc_demoConf( s_ns_t* ps_ns, s_demo_t* p_demos )
{
  s_demo_t* p_d = p_demos;

  EMB6_ASSERT_RET( ps_ns != NULL, 0 );

  while( p_d != NULL )
  {
    /* configure current demo and switch
     * to next demo */
    p_d->pf_conf( ps_ns );
    p_d = p_d->p_next;
  }

  return 0;
}


/**
 * \brief Initialize selected demos.
 *
 *        This function initializes all the demos given in the list of
 *        selected demos.
 *
 * \param p_demos   Demos to initialize.
 *
 * \return  0 on success or negative value on error.
 */
static int8_t loc_demoInit( s_demo_t* p_demos )
{
  s_demo_t* p_d = p_demos;

  EMB6_ASSERT_RET( (p_demos != NULL), -1 );

  while( p_d != NULL )
  {
    /* initialize current demo and switch
     * to next demo */
    p_d->pf_init();
    p_d = p_d->p_next;
  }

  return 0;
}



#if EMB6_INIT_ROOT==TRUE
/**
 * \brief   Initialize the DAGRoot.
 *
 *          If the node acts as a DAGRoot it requires some soecific
 *          initializations which are performed here.
 *
 * \return  1 on success or 0 on error.
 */
static int8_t loc_dagRootInit( void )
{
    uip_ipaddr_t un_ipaddr;
    struct uip_ds6_addr *root_if;
    uint16_t pi_netPrefix[4] = {NETWORK_PREFIX_DODAG};

    /* Mask future ip address with prefix of a net */
    uip_ip6addr(&un_ipaddr, pi_netPrefix[0],pi_netPrefix[1],\
                            pi_netPrefix[2],pi_netPrefix[3], 0, 0, 0, 0);
    /* Add MAC address in the end of IP address */
    uip_ds6_set_addr_iid(&un_ipaddr,(uip_lladdr_t *)&uip_lladdr.addr);
    /* Add new IP address to the list of interfaces */
    uip_ds6_addr_add(&un_ipaddr, 0, ADDR_MANUAL);

    root_if = uip_ds6_get_global(-1);
    if(root_if != NULL) {
        rpl_dag_t *dag;
        dag = rpl_set_root(rpl_config.defInst,(uip_ip6addr_t *)&un_ipaddr);
        rpl_set_prefix(dag, &un_ipaddr, 64);
        LOG_INFO("created a new RPL dag");
    } else {
        LOG_INFO("failed to create a new RPL DAG");
        return 0;
    }
    return 1;
}
#endif /* DEMO_USE_DAG_ROOT  */


/**
 * \brief   Set the stack status.
 *
 *          This function sets the according status variable and
 *          puts an according event to the event queue to inform
 *          othe rmodules.
 *
 */
static void loc_set_status( e_stack_status_t status )
{
    EMB6_ASSERT_RET( (ps_stack != NULL), );

    /* set internal status */
    ps_stack->status = status;

    /* generate according event and execute immediately */
    evproc_putEvent( E_EVPROC_TAIL, EVENT_TYPE_STATUS_CHANGE,
            (void*)&ps_stack->status );
}


/**
 * \brief   Callback function for receiving events.
 *
 *          This function is called every time a new event was generated
 *          that this module has registered to before.
 *
 * \param   ev    The type of the event.
 * \param   data  Extra data.
 */
void loc_event_callback( c_event_t ev, p_data_t data )
{
  e_nsErr_t err;

  if( ev == EVENT_TYPE_STATUS_CHANGE )
  {
    evproc_regCallback( EVENT_TYPE_REQ_INIT, loc_event_callback );
    evproc_regCallback( EVENT_TYPE_REQ_START, loc_event_callback );
    evproc_regCallback( EVENT_TYPE_REQ_STOP, loc_event_callback );
  }
  else if( ev == EVENT_TYPE_REQ_INIT )
  {
    /* reinitialize the stack */
    emb6_init( NULL, NULL, &err );
  }
  else if( ev == EVENT_TYPE_REQ_STOP )
  {
    /* stop the stack */
    emb6_stop( &err );
  }
  else if( ev == EVENT_TYPE_REQ_START )
  {
    /* start the stack */
    emb6_start( &err );
  }

}

/*
 * --- Global Function Definitions ----------------------------------------- *
 */

/*---------------------------------------------------------------------------*/
/*
* emb6_init()
*/
void emb6_init( s_ns_t* ps_ns, s_demo_t* ps_demos, e_nsErr_t* p_err )
{
    uint8_t ret;
    e_nsErr_t err;
    s_ns_t* ps_nsTmp;
    s_demo_t* ps_dmsTmp;

    EMB6_ASSERT_FN( (p_err != NULL), emb6_errorHandler( p_err ) );
    EMB6_ASSERT_RETS( ((ps_ns != NULL) || (ps_stack != NULL) ),
            ,(*p_err), NETSTK_ERR_INVALID_ARGUMENT );
    EMB6_ASSERT_RETS( ((ps_demos != NULL) || (ps_dms != NULL) ),
            ,(*p_err), NETSTK_ERR_INVALID_ARGUMENT );

    /* set return error code to default */
    *p_err = NETSTK_ERR_NONE;

    ps_nsTmp = (ps_ns != NULL) ? ps_ns : ps_stack;
    ps_dmsTmp = (ps_demos != NULL) ? ps_demos : ps_dms;

    /* configure demo applications */
    ret = loc_demoConf( ps_nsTmp, ps_dmsTmp );
    if( ret != 0 )
    {
        *p_err = NETSTK_ERR_INIT;
        LOG_ERR("Failed to initialize emb6 demos");
        emb6_errorHandler(&err);
    }

    /* Initialize stack protocols */
    evproc_init();
    queuebuf_init();

    /* initialize timer */
    etimer_init();
    ctimer_init();
    rt_tmr_init();

    /* initialize trace */
#if (TRACE_CFG_EN == TRUE)
    trace_init();
    trace_start();
    trace_printf("Trace started\n");
#endif

    /* initialize netstack */
    ret = loc_stackInit( ps_nsTmp );
    if( ret != 0 )
    {
        *p_err = NETSTK_ERR_INIT;
        LOG_ERR("Failed to initialize emb6 stack");
        emb6_errorHandler(&err);
    }
    else
    {
        /* set local stack pointer */
        ps_stack = ps_nsTmp;
    }

#if (NETSTK_CFG_LPM_ENABLED == TRUE)
    /* initialize Low-Power-Management */
    lpm_init();

    /* register low-power management callback for the stack */
    lpm_register(loc_stackIdle);
#endif /* #if (NETSTK_CFG_LPM_ENABLED == TRUE) */

    /* initialize demo applications */
    ret = loc_demoInit( ps_dmsTmp );
    if( ret != 0 )
    {
        *p_err = NETSTK_ERR_INIT;
        LOG_ERR("Failed to initialize emb6 demos");
        emb6_errorHandler(&err);
    }
    else
    {
      /* set local demo pointer */
      ps_dms = ps_dmsTmp;
    }

    /* turn the stack on */
    ps_stack->dllc->on(&err);
    if (err != NETSTK_ERR_NONE)
    {
      /* error when enabling stack */
      emb6_errorHandler(&err);
    }

    /* register to initialization request events */
    evproc_regCallback( EVENT_TYPE_STATUS_CHANGE, loc_event_callback );
    evproc_regCallback( EVENT_TYPE_REQ_INIT, loc_event_callback );
    evproc_regCallback( EVENT_TYPE_REQ_START, loc_event_callback );
    evproc_regCallback( EVENT_TYPE_REQ_STOP, loc_event_callback );

#if EMB6_NO_AUTOSTART != TRUE
    /* enable stack per default */
    loc_set_status( STACK_STATUS_ACTIVE );
#else
    loc_set_status( STACK_STATUS_IDLE );
#endif /* #if EMB6_NO_AUTOSTART != TRUE */

} /* emb6_init() */


/*---------------------------------------------------------------------------*/
/*
* emb6_process()
*/
void emb6_process( int32_t us_delay )
{
    uint8_t runLoop = (us_delay < 0) ? FALSE : TRUE;
    uint32_t delay = runLoop ? us_delay : 0;

    /* Attention: emb6 main process loop !! do not change !! */
    do
    {
        if( ps_stack != NULL )
        {
          evproc_nextEvent();
          etimer_request_poll();
          bsp_delayUs(delay);

#if (NETSTK_CFG_LPM_ENABLED == TRUE)
          /* low power manager handler entry */
          lpm_entry();
#endif /* #if (NETSTK_CFG_LPM_ENABLED == TRUE) */

#if UIP_CONF_IPV6_RPL
#if EMB6_INIT_ROOT==TRUE
          if( emb6_getStatus() == STACK_STATUS_ACTIVE )
            loc_set_status( STACK_STATUS_NETWORK );
#else
          /* check if we have a connection to a DAGRoot */
          if( (emb6_getStatus() == STACK_STATUS_ACTIVE) &&
              (rpl_get_any_dag() != NULL ) )
          {
              loc_set_status( STACK_STATUS_NETWORK );
          }

          /* check if we have a connection to a DAGRoot */
          if( (emb6_getStatus() == STACK_STATUS_NETWORK) &&
              (rpl_get_any_dag() == NULL ) )
          {
              loc_set_status( STACK_STATUS_ACTIVE );
          }
#endif /* #if EMB6_INIT_ROOT==TRUE */
#endif

        }
    }while(runLoop);

} /* emb6_process() */


/*---------------------------------------------------------------------------*/
/*
* emb6_get()
*/
const s_ns_t* emb6_get( void )
{
    /* return pointer to the current stack structure */
    return ps_stack;

} /* emb6_get() */


/*---------------------------------------------------------------------------*/
/*
* emb6_get()
*/
e_stack_status_t emb6_getStatus( void )
{
    if( ps_stack != NULL )
        /* return current status */
        return ps_stack->status;
    else
        /*return error */
      return STACK_STATUS_ERROR;

} /* emb6_get() */


/*---------------------------------------------------------------------------*/
/*
* emb6_start()
*/
void emb6_start( e_nsErr_t *p_err )
{
    e_nsErr_t err = NETSTK_ERR_FATAL;

    if( (ps_stack != NULL) &&
        (ps_stack->status != STACK_STATUS_ACTIVE) )
    {
        /* reinitialize stack with the given
         * parameters and configurations */
        emb6_init( NULL, NULL, &err );

        /* turn the stack on */
        ps_stack->dllc->on( &err );
        if(err != NETSTK_ERR_NONE)
        {
           e_nsErr_t errStop;
          /* stop stack */
          emb6_stop( &errStop );
        }
        else
        {
          /* enable stack */
          loc_set_status( STACK_STATUS_ACTIVE );
        }
    }

} /* emb6_start() */


/*---------------------------------------------------------------------------*/
/*
* emb6_stop()
*/
void emb6_stop( e_nsErr_t *p_err )
{
    if( ps_stack != NULL )
    {
        /* reset all events */
        evproc_init();

        /* disable MAC */
        ps_stack->dllc->off( p_err );

        /* disable stack */
        loc_set_status( STACK_STATUS_IDLE );
    }

} /* emb6_stop() */


/*---------------------------------------------------------------------------*/
/*
* emb6_errorHandler()
*/
void emb6_errorHandler( e_nsErr_t* p_err )
{
    /* turns LEDs on to indicate error */
    bsp_led(HAL_LED0, EN_BSP_LED_OP_ON);
    LOG_ERR("Program failed");

    /* set error status */
    if( ps_stack != NULL )
        loc_set_status( STACK_STATUS_ERROR );

    /* TODO missing error handling */
    while (1) {
    }

} /* emb6_errorHandler() */

