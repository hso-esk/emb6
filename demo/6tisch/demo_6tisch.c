/*
 * Copyright (c) 2015, SICS Swedish ICT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * \file
 *         A RPL+TSCH node able to act as either a simple node (6ln),
 *         DAG Root (6dr) or DAG Root with security (6dr-sec)
 *         Press use button at startup to configure.
 *
 * \author Simon Duquennoy <simonduq@sics.se>
 */

#include "demo_6tisch.h"
#include "rpl.h"
#include "uip-ds6-route.h"
#include "tsch.h"
#include "tsch-schedule.h"
#include "rpl-private.h"
#if WITH_ORCHESTRA
#include "orchestra.h"
#endif /* WITH_ORCHESTRA */

#if (HAL_SUPPORT_RTIMER != TRUE)
#error 6tisch does not work without rtimer module
#endif

#define LOGGER_ENABLE           LOGGER_DEMO_6TISCH
#if LOGGER_ENABLE == TRUE
#define DEBUG DEBUG_PRINT
#define LOGGER_SUBSYSTEM        "6TISCH"
#endif /* #if LOGGER_ENABLE == TRUE */
#include    "logger.h"
#include "uip-debug.h"

/*
 *  --- Local Variables ---------------------------------------------------- *
 */

static struct ctimer ct;

/*
 *  --- Local Functions ---------------------------------------------------- *
 */

static void print_network_status(void)
{
  int i;
  uint8_t state;
  uip_ds6_defrt_t *default_route;
#if RPL_WITH_STORING
  uip_ds6_route_t *route;
#endif /* RPL_WITH_STORING */
#if RPL_WITH_NON_STORING
  rpl_ns_node_t *link;
#endif /* RPL_WITH_NON_STORING */

  LOG_RAW("--- Network status ---\n");

  /* Our IPv6 addresses */
  LOG_RAW("- Server IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      LOG_RAW("-- ");
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      LOG_RAW("\n");
    }
  }

  /* Our default route */
  LOG_RAW("- Default route:\n");
  default_route = uip_ds6_defrt_lookup(uip_ds6_defrt_choose());
  if(default_route != NULL) {
    LOG_RAW("-- ");
    PRINT6ADDR(&default_route->ipaddr);
    LOG_RAW(" (lifetime: %lu seconds)\n", (unsigned long)default_route->lifetime.interval);
  } else {
    LOG_RAW("-- None\n");
  }

#if RPL_WITH_STORING
  /* Our routing entries */
  LOG_RAW("- Routing entries (%u in total):\n", uip_ds6_route_num_routes());
  route = uip_ds6_route_head();
  while(route != NULL) {
	LOG_RAW("-- ");
	PRINT6ADDR(&route->ipaddr);
    LOG_RAW(" via ");
    PRINT6ADDR(uip_ds6_route_nexthop(route));
    LOG_RAW(" (lifetime: %lu seconds)\n", (unsigned long)route->state.lifetime);
    route = uip_ds6_route_next(route);
  }
#endif

#if RPL_WITH_NON_STORING
  /* Our routing links */
  PRINTF("- Routing links (%u in total):\n", rpl_ns_num_nodes());
  link = rpl_ns_node_head();
  while(link != NULL) {
    uip_ipaddr_t child_ipaddr;
    uip_ipaddr_t parent_ipaddr;
    rpl_ns_get_node_global_addr(&child_ipaddr, link);
    rpl_ns_get_node_global_addr(&parent_ipaddr, link->parent);
    LOG_RAW("-- ");
    PRINT6ADDR(&child_ipaddr);
    if(link->parent == NULL) {
      memset(&parent_ipaddr, 0, sizeof(parent_ipaddr));
      LOG_RAW(" --- DODAG root ");
    } else {
      LOG_RAW(" to ");
      PRINT6ADDR(&parent_ipaddr);
    }
    LOG_RAW(" (lifetime: %lu seconds)\n", (unsigned long)link->lifetime);
    link = rpl_ns_node_next(link);
  }
#endif

  LOG_RAW("----------------------\n");
}

static void net_init(uip_ipaddr_t *br_prefix)
{
  uip_ipaddr_t global_ipaddr;

  if(br_prefix) { /* We are RPL root. Will be set automatically
                     as TSCH pan coordinator via the tsch-rpl module */
    memcpy(&global_ipaddr, br_prefix, 16);
    uip_ds6_set_addr_iid(&global_ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&global_ipaddr, 0, ADDR_AUTOCONF);
    rpl_set_root(RPL_DEFAULT_INSTANCE, &global_ipaddr);
    rpl_set_prefix(rpl_get_any_dag(), br_prefix, 64);
    rpl_repair_root(RPL_DEFAULT_INSTANCE);
  }

  /* FIXME Normally all layers should be enabled during emb6 init */
  //NETSTACK_MAC.on();
}

static  void demo_6tisch_cb_logger(void *ptr)
{
	print_network_status();
	tsch_schedule_print();
	ctimer_set(&ct, bsp_getTRes() * 15, demo_6tisch_cb_logger, NULL);
}



/*
 * --- Global Function Definitions ----------------------------------------- *
 */


/*---------------------------------------------------------------------------*/
/*
* demo_6tischConf()
*/
int8_t demo_6tischConf(s_ns_t *p_netstk)
{
  int8_t ret = -1;

  if (p_netstk != NULL) {
    if (p_netstk->c_configured == FALSE) {
      p_netstk->hc = &hc_driver_sicslowpan;
      p_netstk->frame = &framer_802154;
      p_netstk->dllsec = &dllsec_tsch_adaptive_driver;
      p_netstk->dllc = &dll_tsch_adaptive_driver;
      p_netstk->mac = &mac_tsch_adaptive_driver;
      p_netstk->phy   = &phy_driver_null;
      ret = 0;
    } else {
      if ((p_netstk->hc == &hc_driver_sicslowpan) &&
          (p_netstk->frame == &framer_802154)) {
          p_netstk->dllsec = &dllsec_tsch_adaptive_driver;
          p_netstk->dllc = &dll_tsch_adaptive_driver;
          p_netstk->mac = &mac_tsch_adaptive_driver;
          p_netstk->phy   = &phy_driver_null;
        ret = 0;
      } else {
        p_netstk = NULL;
        ret = -1;
      }
    }
  }
  return ret;
} /* demo_6tischConf() */

/*---------------------------------------------------------------------------*/
/*
* demo_6tischInit()
*/
int8_t demo_6tischInit(void)
{
	  /* 3 possible roles:
	   * - role_6ln: simple node, will join any network, secured or not
	   * - role_6dr: DAG root, will advertise (unsecured) beacons
	   * - role_6dr_sec: DAG root, will advertise secured beacons
	   * */
	  static int is_coordinator = 0;
	  static enum { role_6ln, role_6dr, role_6dr_sec } node_role;

	  /* selecting the configured  role for the 6tisch network */
#ifdef TISCH_ROLE_6LN
	  node_role = role_6ln;
#elif TISCH_ROLE_6DR
	  node_role = role_6dr;
#elif TISCH_ROLE_6DR_SEC
	  node_role = role_6dr_sec;
#endif

	  LOG_RAW("Init: node starting with role %s\n",
	         node_role == role_6ln ? "6ln" : (node_role == role_6dr) ? "6dr" : "6dr-sec");

	  tsch_set_pan_secured(LLSEC802154_ENABLED && (node_role == role_6dr_sec));
	  is_coordinator = node_role > role_6ln;

	  if(is_coordinator) {
	    uip_ipaddr_t prefix;
	    uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
	    net_init(&prefix);
	    tsch_set_coordinator(1);
	  } else {
	    net_init(NULL);
	  }

	#if WITH_ORCHESTRA
	  orchestra_init();
	#endif /* WITH_ORCHESTRA */
  /* start the ctimer for logging */
		ctimer_set(&ct, bsp_getTRes() * 15, demo_6tisch_cb_logger, NULL);
  /* Always success */
  return 0;
} /* demo_6tischInit() */

/*---------------------------------------------------------------------------*/
